"""
可视化工具
生成点云着色图、投影叠加图、标定结果概览图
"""
from __future__ import annotations
import logging
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class CalibVisualizer:
    """标定结果可视化。"""

    def __init__(self, output_dir: str):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def draw_lidar_on_image(
        self,
        img: np.ndarray,
        pts_cam: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
        colormap: int = cv2.COLORMAP_JET,
        max_depth: float = 50.0,
    ) -> np.ndarray:
        """
        将相机坐标系下的 LiDAR 点云投影到图像并着色。
        深度越近颜色越红，越远越蓝。
        """
        mask = pts_cam[:, 2] > 0
        pts_f = pts_cam[mask]
        if len(pts_f) == 0:
            return img.copy()

        depths = pts_f[:, 2]
        pts_2d, _ = cv2.projectPoints(
            pts_f, np.zeros(3), np.zeros(3), K, D)
        pts_2d = pts_2d.reshape(-1, 2)

        result = img.copy()
        H, W = img.shape[:2]

        effective_max_depth = max_depth if max_depth > 0 else (float(np.max(depths)) or 1.0)
        depth_norm = np.clip(depths / effective_max_depth, 0, 1)
        colors_map = cv2.applyColorMap(
            (depth_norm * 255).astype(np.uint8).reshape(-1, 1),
            colormap
        ).reshape(-1, 3)

        for (u, v), color in zip(pts_2d, colors_map):
            ui, vi = int(round(u)), int(round(v))
            if 0 <= ui < W and 0 <= vi < H:
                cv2.circle(result, (ui, vi), 2,
                           (int(color[0]), int(color[1]), int(color[2])), -1)
        return result

    def save_projection_comparison(
        self, img: np.ndarray, pts: np.ndarray,
        R_before: np.ndarray, t_before: np.ndarray,
        R_after: np.ndarray, t_after: np.ndarray,
        K: np.ndarray, D: np.ndarray,
        filename: str = "projection_comparison.jpg",
    ):
        """保存标定前后的投影对比图。"""
        pts_cam_before = (R_before @ pts[:, :3].T + t_before.reshape(3, 1)).T
        pts_cam_after = (R_after @ pts[:, :3].T + t_after.reshape(3, 1)).T

        vis_before = self.draw_lidar_on_image(img, pts_cam_before, K, D)
        vis_after = self.draw_lidar_on_image(img, pts_cam_after, K, D)

        H, W = img.shape[:2]
        label_cfg = dict(fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.0,
                         color=(255, 255, 255), thickness=2)
        cv2.putText(vis_before, "Before Calibration", (20, 40), **label_cfg)
        cv2.putText(vis_after, "After Calibration", (20, 40), **label_cfg)

        comparison = np.hstack([vis_before, vis_after])
        out_path = self.output_dir / filename
        cv2.imwrite(str(out_path), comparison)
        logger.info(f"  Saved projection comparison: {out_path}")

    def save_bev_image(
        self, images: Dict[str, np.ndarray],
        extrinsics: Dict,  # {cam_id: (R, t)}
        intrinsics: Dict,
        bev_size: Tuple[int, int] = (800, 800),
        scale: float = 10.0,  # 像素/米
        filename: str = "bev.jpg",
    ):
        """
        生成鸟瞰图 (BEV) 可视化，用于环视相机标定验证。
        """
        bev = np.zeros((bev_size[1], bev_size[0], 3), dtype=np.uint8)
        cx, cy = bev_size[0] // 2, bev_size[1] // 2

        for cam_id, img in images.items():
            if cam_id not in extrinsics or cam_id not in intrinsics:
                continue
            R, t = extrinsics[cam_id]
            intr = intrinsics[cam_id]
            if not hasattr(intr, "K"):
                continue
            K = intr.K
            D = intr.dist_coeffs

            # 简化 BEV: 将每个像素反投影到地面平面 (z=0)
            h, w = img.shape[:2]
            step = 4
            for v in range(0, h, step):
                for u in range(0, w, step):
                    # 相机坐标系下的射线方向
                    ray_cam = np.linalg.inv(K) @ np.array([u, v, 1.0])
                    # 世界坐标系下的射线方向
                    ray_world = R.T @ ray_cam
                    # 与地面 z=0 的交点
                    if abs(ray_world[2]) < 1e-6:
                        continue
                    pos_world = -R.T @ t / ray_world[2] * ray_world
                    bx = int(cx + pos_world[0] * scale)
                    by = int(cy - pos_world[1] * scale)
                    if 0 <= bx < bev_size[0] and 0 <= by < bev_size[1]:
                        bev[by, bx] = img[v, u]

        out_path = self.output_dir / filename
        cv2.imwrite(str(out_path), bev)
        logger.info(f"  Saved BEV image: {out_path}")

    def save_calib_overview(
        self, sensors, intrinsics, extrinsics,
        filename: str = "calib_overview.png"
    ):
        """
        生成传感器标定关系概览图（使用 matplotlib）。
        """
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D

            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection="3d")

            colors = {"camera_pinhole": "blue", "camera_fisheye": "cyan",
                      "lidar": "red", "imu": "green"}
            markers = {"camera_pinhole": "s", "camera_fisheye": "^",
                       "lidar": "o", "imu": "D"}

            # 绘制各传感器位置
            for sid, sensor in sensors.items():
                color = colors.get(sensor.sensor_type.value, "gray")
                marker = markers.get(sensor.sensor_type.value, "o")
                ax.scatter(0, 0, 0, c=color, marker=marker, s=200,
                           label=f"{sid} ({sensor.sensor_type.value})")

            # 绘制标定关系箭头
            for (sa, sb), result in extrinsics.items():
                t = result.translation
                ax.quiver(0, 0, 0, t[0], t[1], t[2],
                          arrow_length_ratio=0.2, color="orange", alpha=0.7)
                mid = t * 0.5
                ax.text(mid[0], mid[1], mid[2], f"{sa}→{sb}",
                        fontsize=8, ha="center")

            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            ax.set_title("UniCalib 传感器标定关系概览")
            ax.legend(loc="upper left", fontsize=8)

            out_path = self.output_dir / filename
            plt.savefig(str(out_path), dpi=150, bbox_inches="tight")
            plt.close()
            logger.info(f"  Saved calib overview: {out_path}")

        except ImportError:
            logger.warning("matplotlib not available for calib overview plot.")
        except Exception as e:
            logger.warning(f"Failed to generate calib overview: {e}")
