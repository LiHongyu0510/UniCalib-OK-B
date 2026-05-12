"""
边缘对齐验证器
将 LiDAR 深度不连续处投影到图像，检验与图像边缘的对齐程度
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


class EdgeAlignmentValidator:
    """LiDAR 深度边缘与图像边缘对齐验证。"""

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
        计算 LiDAR 深度不连续处与图像边缘的对齐误差。

        LiDAR 深度不连续点 = 相邻点深度差 > threshold (对应物体边缘)。
        """
        intr = intrinsics.get(cam_id)
        if intr is None:
            return {"edge_align_error_px": float('inf'), "note": "no_intrinsic"}

        K, D = self._get_KD(intr)
        R = extrinsic.rotation
        t = extrinsic.translation.reshape(3, 1)

        sensor_lidar = sensors[lidar_id]
        sensor_cam = sensors[cam_id]

        all_errors = []
        frame_count = 0

        for ts, data_lidar, data_cam in data_mgr.iter_synced_frames(
                sensor_lidar.topic, sensor_cam.topic,
                sync_threshold_ms=50.0, max_pairs=15):
            errors = self._compute_edge_align(data_lidar, data_cam, R, t, K, D)
            all_errors.extend(errors)
            frame_count += 1
            if frame_count >= 10:
                break

        if not all_errors:
            return {"edge_align_error_px": float('inf'), "n_points": 0}

        errs = np.array(all_errors)
        metrics = {
            "edge_align_error_px": float(np.mean(errs)),
            "edge_align_median_px": float(np.median(errs)),
            "pct_within_2px": float(np.mean(errs < 2.0) * 100),
            "n_edge_points": len(errs),
        }
        logger.info(f"  Edge alignment [{lidar_id}-{cam_id}]: "
                    f"mean={metrics['edge_align_error_px']:.2f}px  "
                    f"<2px={metrics['pct_within_2px']:.1f}%")
        return metrics

    def _compute_edge_align(
        self, pts: np.ndarray, img: np.ndarray,
        R: np.ndarray, t: np.ndarray,
        K: np.ndarray, D: np.ndarray,
        depth_thresh: float = 0.5,
    ):
        """提取 LiDAR 深度不连续点，投影后计算到图像边缘的距离。"""
        pts_cam = (R @ pts[:, :3].T + t).T
        mask = pts_cam[:, 2] > 0.3
        pts_cam_f = pts_cam[mask]
        if len(pts_cam_f) < 20:
            return []

        # 深度排序并找不连续点
        depths = pts_cam_f[:, 2]
        sorted_idx = np.argsort(np.arctan2(pts_cam_f[:, 1], pts_cam_f[:, 0]))
        sorted_depths = depths[sorted_idx]
        depth_diff = np.abs(np.diff(sorted_depths))
        edge_mask = np.where(depth_diff > depth_thresh)[0]

        if len(edge_mask) == 0:
            return []

        edge_pts = pts_cam_f[sorted_idx[edge_mask]]

        # 图像边缘检测
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 30, 100)
        dist_map = distance_transform_edt(edges == 0).astype(np.float32)

        # 投影边缘点
        pts_2d, _ = cv2.projectPoints(
            edge_pts, np.zeros(3), np.zeros(3), K, D)
        pts_2d = pts_2d.reshape(-1, 2)

        H, W = img.shape[:2]
        errors = []
        for u, v in pts_2d:
            ui, vi = int(round(u)), int(round(v))
            if 0 <= ui < W and 0 <= vi < H:
                errors.append(float(dist_map[vi, ui]))

        return errors

    def _get_KD(self, intr):
        if isinstance(intr, CameraIntrinsic):
            return intr.K, intr.dist_coeffs
        elif isinstance(intr, FisheyeIntrinsic):
            p = intr.params
            K = np.array([[p.get("fx", 500), 0, p.get("cx", 320)],
                           [0, p.get("fy", 500), p.get("cy", 240)],
                           [0, 0, 1]], dtype=np.float64)
            return K, np.zeros(4)
        return np.eye(3) * 500, np.zeros(4)
