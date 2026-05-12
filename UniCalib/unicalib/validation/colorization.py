"""
点云着色一致性验证器
若标定正确，3D 空间中相邻点在图像中颜色应相似
"""
from __future__ import annotations
import logging
from typing import Dict

import cv2
import numpy as np

from ..core.calib_result import CalibResult, CameraIntrinsic, FisheyeIntrinsic
from ..core.sensor_config import SensorConfig
from ..core.data_manager import DataManager

logger = logging.getLogger(__name__)


class ColorizationValidator:
    """点云着色质量验证。"""

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
        计算点云着色一致性分数。
        指标: 3D 邻近点在图像中的颜色方差（越小越好）。
        """
        intr = intrinsics.get(cam_id)
        if intr is None:
            return {"colorization_score": 0.0, "note": "no_intrinsic"}

        K, D = self._get_KD(intr)
        R = extrinsic.rotation
        t = extrinsic.translation.reshape(3, 1)

        sensor_lidar = sensors[lidar_id]
        sensor_cam = sensors[cam_id]

        consistency_scores = []
        frame_count = 0

        for ts, data_lidar, data_cam in data_mgr.iter_synced_frames(
                sensor_lidar.topic, sensor_cam.topic,
                sync_threshold_ms=50.0, max_pairs=10):
            img = data_cam
            pts = data_lidar

            score = self._compute_color_consistency(pts, img, R, t, K, D)
            if score is not None:
                consistency_scores.append(score)
            frame_count += 1
            if frame_count >= 5:
                break

        if not consistency_scores:
            return {"colorization_score": 0.0, "color_consistency": float('inf')}

        mean_consistency = float(np.mean(consistency_scores))
        colorization_score = float(1.0 / (1.0 + mean_consistency))

        logger.info(f"  Colorization [{lidar_id}-{cam_id}]: "
                    f"consistency={mean_consistency:.2f}  "
                    f"score={colorization_score:.3f}")
        return {
            "color_consistency": mean_consistency,
            "colorization_score": colorization_score,
        }

    def _compute_color_consistency(
        self, pts: np.ndarray, img: np.ndarray,
        R: np.ndarray, t: np.ndarray,
        K: np.ndarray, D: np.ndarray,
    ):
        """计算单帧着色一致性：邻近 3D 点颜色差异的均值。"""
        pts_cam = (R @ pts[:, :3].T + t).T
        mask = pts_cam[:, 2] > 0.5
        pts_cam = pts_cam[mask]
        pts_orig = pts[mask, :3]

        if len(pts_cam) < 50:
            return None

        # 投影
        pts_2d, _ = cv2.projectPoints(
            pts_cam, np.zeros(3), np.zeros(3), K, D)
        pts_2d = pts_2d.reshape(-1, 2)

        H, W = img.shape[:2]
        valid = (pts_2d[:, 0] >= 0) & (pts_2d[:, 0] < W) & \
                (pts_2d[:, 1] >= 0) & (pts_2d[:, 1] < H)

        if np.sum(valid) < 20:
            return None

        pts_2d_v = pts_2d[valid].astype(int)
        pts_3d_v = pts_orig[valid]

        # 采样颜色
        colors = img[pts_2d_v[:, 1], pts_2d_v[:, 0]].astype(np.float32)

        # KD-Tree 查找 3D 邻近点
        try:
            from sklearn.neighbors import KDTree
        except ImportError:
            return float(np.std(colors))

        tree = KDTree(pts_3d_v)
        step = max(1, len(pts_3d_v) // 100)
        diffs = []

        for i in range(0, len(pts_3d_v), step):
            neighbors_idx = tree.query(pts_3d_v[i:i+1], k=min(5, len(pts_3d_v)))[1][0]
            neighbor_colors = colors[neighbors_idx]
            color_std = np.std(neighbor_colors, axis=0).mean()
            diffs.append(float(color_std))

        return float(np.mean(diffs)) if diffs else None

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
