"""
重投影误差验证器
将 LiDAR 点云投影到图像，计算到图像边缘的距离
"""
from __future__ import annotations
import logging
from typing import Dict

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt

from ..core.calib_result import CalibResult, CameraIntrinsic, FisheyeIntrinsic
from ..core.sensor_config import SensorConfig
from ..core.data_manager import DataManager

logger = logging.getLogger(__name__)


class ReprojectionValidator:
    """LiDAR-Camera 重投影误差验证。"""

    def validate(
        self,
        data_mgr: DataManager,
        lidar_id: str,
        cam_id: str,
        sensors: Dict[str, SensorConfig],
        intrinsics: Dict,
        extrinsic: CalibResult,
    ) -> Dict:
        """
        计算 LiDAR 点投影到图像的边缘对齐误差。

        Returns:
            {mean_error_px, median_error_px, std_error_px,
             max_error_px, pct_within_1px, pct_within_3px}
        """
        intr = intrinsics.get(cam_id)
        if intr is None:
            return {"mean_error_px": float('inf'), "note": "no_intrinsic"}

        K, D = self._get_KD(intr)
        R = extrinsic.rotation
        t = extrinsic.translation.reshape(3, 1)

        all_errors = []
        frame_count = 0

        sensor_lidar = sensors[lidar_id]
        sensor_cam = sensors[cam_id]

        for ts, img, pts in self._load_synced_frames(data_mgr, sensor_lidar, sensor_cam):
            errors = self._compute_edge_errors(pts, img, R, t, K, D)
            all_errors.extend(errors)
            frame_count += 1
            if frame_count >= 20:
                break

        if not all_errors:
            return {"mean_error_px": float('inf'), "note": "no_frames"}

        errs = np.array(all_errors)
        metrics = {
            "mean_error_px": float(np.mean(errs)),
            "median_error_px": float(np.median(errs)),
            "std_error_px": float(np.std(errs)),
            "max_error_px": float(np.max(errs)),
            "pct_within_1px": float(np.mean(errs < 1.0) * 100),
            "pct_within_3px": float(np.mean(errs < 3.0) * 100),
            "n_points": len(errs),
            "n_frames": frame_count,
        }
        logger.info(f"  Reprojection [{lidar_id}-{cam_id}]: "
                    f"mean={metrics['mean_error_px']:.2f}px  "
                    f"median={metrics['median_error_px']:.2f}px  "
                    f"<1px={metrics['pct_within_1px']:.1f}%")
        return metrics

    def _compute_edge_errors(
        self, pts: np.ndarray, img: np.ndarray,
        R: np.ndarray, t: np.ndarray,
        K: np.ndarray, D: np.ndarray,
    ):
        """计算各 LiDAR 投影点到最近边缘的距离。"""
        pts_cam = (R @ pts[:, :3].T + t).T  # N×3
        mask = pts_cam[:, 2] > 0.5
        pts_cam = pts_cam[mask]
        if len(pts_cam) == 0:
            return []

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 30, 100)

        # 距离变换: 到最近边缘的欧氏距离
        dist_map = distance_transform_edt(edges == 0).astype(np.float32)

        # 投影
        pts_2d, _ = cv2.projectPoints(
            pts_cam, np.zeros(3), np.zeros(3), K, D)
        pts_2d = pts_2d.reshape(-1, 2)

        H, W = img.shape[:2]
        errors = []
        step = max(1, len(pts_2d) // 200)  # 采样上限 200 点
        for u, v in pts_2d[::step]:
            ui, vi = int(round(u)), int(round(v))
            if 0 <= ui < W and 0 <= vi < H:
                errors.append(float(dist_map[vi, ui]))

        return errors

    def _get_KD(self, intr):
        if isinstance(intr, CameraIntrinsic):
            return intr.K, intr.dist_coeffs
        elif isinstance(intr, FisheyeIntrinsic):
            p = intr.params
            K = np.array([
                [p.get("fx", 500), 0, p.get("cx", 320)],
                [0, p.get("fy", 500), p.get("cy", 240)],
                [0, 0, 1]], dtype=np.float64)
            return K, np.zeros(4)
        return np.eye(3) * 500, np.zeros(4)

    def _load_synced_frames(self, data_mgr, sensor_lidar, sensor_cam):
        for ts, data_lidar, data_cam in data_mgr.iter_synced_frames(
                sensor_lidar.topic, sensor_cam.topic,
                sync_threshold_ms=50.0, max_pairs=30):
            yield ts, data_cam, data_lidar
