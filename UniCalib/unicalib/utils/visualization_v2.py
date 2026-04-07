"""
增强版可视化工具 V2
支持：热力图投影、误差分布图、3D轨迹可视化、交互式图表
"""
from __future__ import annotations
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np
import cv2

logger = logging.getLogger(__name__)


class CalibVisualizerV2:
    """增强版标定结果可视化器。"""

    def __init__(self, output_dir: str):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def draw_lidar_heatmap(
        self,
        img: np.ndarray,
        pts_cam: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
        max_depth: float = 50.0,
        point_size: int = 3,
        alpha: float = 0.7,
        colormap: int = cv2.COLORMAP_JET,
        error_map: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        将 LiDAR 点云投影到图像生成热力图。
        
        Args:
            img: 原始图像
            pts_cam: 相机坐标系下的点云 (N×3)
            K: 相机内参矩阵
            D: 畸变系数
            max_depth: 最大深度（用于归一化）
            point_size: 点大小
            alpha: 透明度
            colormap: OpenCV 颜色映射
            error_map: 可选的重投影误差 (N,)，用于误差着色
        
        Returns:
            叠加了热力图的图像
        """
        mask = pts_cam[:, 2] > 0.1
        pts_f = pts_cam[mask]
        if len(pts_f) == 0:
            return img.copy()

        if error_map is not None:
            error_map = error_map[mask]

        # 投影到图像平面
        pts_2d, _ = cv2.projectPoints(
            pts_f, np.zeros(3), np.zeros(3), K, D
        )
        pts_2d = pts_2d.reshape(-1, 2)

        result = img.copy().astype(np.float32)
        H, W = img.shape[:2]

        # 归一化深度或误差
        if error_map is not None:
            values = np.clip(error_map / (np.percentile(error_map, 95) + 1e-6), 0, 1)
        else:
            values = np.clip(pts_f[:, 2] / max_depth, 0, 1)

        # 生成颜色映射
        colors_map = cv2.applyColorMap(
            (values * 255).astype(np.uint8).reshape(-1, 1), colormap
        ).reshape(-1, 3).astype(np.float32)

        # 绘制半透明点
        overlay = np.zeros_like(result)
        for (u, v), color in zip(pts_2d, colors_map):
            ui, vi = int(round(u)), int(round(v))
            if 1 <= ui < W-1 and 1 <= vi < H-1:
                cv2.circle(overlay, (ui, vi), point_size,
                          (int(color[0]), int(color[1]), int(color[2])), -1)

        result = result * (1 - alpha) + overlay * alpha
        return result.astype(np.uint8)

    def save_error_distribution_plot(
        self,
        errors: np.ndarray,
        title: str = "Reprojection Error Distribution",
        filename: str = "error_distribution.png",
        threshold: float = 1.0,
    ):
        """
        保存误差分布直方图和 CDF 曲线。
        
        Args:
            errors: 误差值数组
            title: 图表标题
            filename: 输出文件名
            threshold: 合格阈值线
        """
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

            # 直方图
            ax1.hist(errors, bins=50, color='steelblue', alpha=0.7, edgecolor='black')
            ax1.axvline(threshold, color='red', linestyle='--', 
                       linewidth=2, label=f'Threshold ({threshold}px)')
            ax1.axvline(np.median(errors), color='green', linestyle='--',
                       linewidth=2, label=f'Median ({np.median(errors):.2f}px)')
            ax1.set_xlabel('Error (pixels)', fontsize=12)
            ax1.set_ylabel('Count', fontsize=12)
            ax1.set_title('Error Histogram', fontsize=13, fontweight='bold')
            ax1.legend()
            ax1.grid(alpha=0.3)

            # CDF 曲线
            sorted_err = np.sort(errors)
            cdf = np.arange(1, len(sorted_err) + 1) / len(sorted_err)
            ax2.plot(sorted_err, cdf * 100, linewidth=2, color='steelblue')
            ax2.axhline(95, color='orange', linestyle='--', 
                       linewidth=1.5, label='95th Percentile')
            ax2.axvline(threshold, color='red', linestyle='--',
                       linewidth=2, label=f'Threshold ({threshold}px)')
            ax2.set_xlabel('Error (pixels)', fontsize=12)
            ax2.set_ylabel('Cumulative Percentage (%)', fontsize=12)
            ax2.set_title('Cumulative Distribution', fontsize=13, fontweight='bold')
            ax2.legend()
            ax2.grid(alpha=0.3)
            ax2.set_ylim([0, 100])

            # 统计信息文本框
            stats_text = (
                f"Mean: {np.mean(errors):.3f} px\n"
                f"Median: {np.median(errors):.3f} px\n"
                f"Std: {np.std(errors):.3f} px\n"
                f"Max: {np.max(errors):.3f} px\n"
                f"<1px: {np.mean(errors < 1.0) * 100:.1f}%\n"
                f"<2px: {np.mean(errors < 2.0) * 100:.1f}%"
            )
            ax1.text(0.98, 0.98, stats_text, transform=ax1.transAxes,
                    fontsize=10, verticalalignment='top',
                    horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

            plt.suptitle(title, fontsize=14, fontweight='bold', y=1.02)
            plt.tight_layout()

            out_path = self.output_dir / filename
            plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
            plt.close()
            logger.info(f"  Saved error distribution: {out_path}")

        except ImportError:
            logger.warning("matplotlib not available for error distribution plot.")

    def save_multi_frame_projection(
        self,
        images: List[np.ndarray],
        pts_list: List[np.ndarray],
        extrinsics_list: List[Tuple[np.ndarray, np.ndarray]],
        K: np.ndarray,
        D: np.ndarray,
        labels: Optional[List[str]] = None,
        filename: str = "multi_frame_comparison.jpg",
    ) -> np.ndarray:
        """
        生成多帧投影对比图（标定前后、不同迭代结果等）。
        
        Args:
            images: 图像列表
            pts_list: 对应的点云列表（世界坐标系）
            extrinsics_list: 外参列表 [(R, t), ...]
            K: 相机内参
            D: 畸变系数
            labels: 每个子图的标签
            filename: 输出文件名
        
        Returns:
            拼接后的对比图像
        """
        n_frames = len(images)
        if n_frames == 0:
            return None

        h, w = images[0].shape[:2]
        grid_cols = min(n_frames, 4)
        grid_rows = (n_frames + grid_cols - 1) // grid_cols

        result = np.zeros((grid_rows * h, grid_cols * w, 3), dtype=np.uint8)

        for i, (img, pts, (R, t)) in enumerate(zip(images, pts_list, extrinsics_list)):
            # 变换到相机坐标系
            pts_cam = (R @ pts[:, :3].T + t.reshape(3, 1)).T
            
            # 生成热力图
            vis = self.draw_lidar_heatmap(img, pts_cam, K, D)
            
            # 添加标签
            label = labels[i] if labels and i < len(labels) else f"Frame {i+1}"
            cv2.putText(vis, label, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            # 计算统计信息
            mask = pts_cam[:, 2] > 0
            valid_pts = pts_cam[mask]
            if len(valid_pts) > 0:
                stats = f"Points: {len(valid_pts)}"
                cv2.putText(vis, stats, (10, h - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            row = i // grid_cols
            col = i % grid_cols
            result[row*h:(row+1)*h, col*w:(col+1)*w] = vis

        out_path = self.output_dir / filename
        cv2.imwrite(str(out_path), result)
        logger.info(f"  Saved multi-frame comparison: {out_path}")
        return result

    def save_sensor_frustum(
        self,
        sensors: Dict,
        intrinsics: Dict,
        extrinsics: Dict,
        camera_ids: Optional[List[str]] = None,
        lidar_ids: Optional[List[str]] = None,
        filename: str = "sensor_frustums.png",
    ):
        """
        生成传感器视锥和坐标系 3D 概览图。
        
        Args:
            sensors: 传感器配置字典
            intrinsics: 内参结果字典
            extrinsics: 外参结果字典
            camera_ids: 要显示的相机ID列表
            lidar_ids: 要显示的LiDAR ID列表
            filename: 输出文件名
        """
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection

            fig = plt.figure(figsize=(14, 12))
            ax = fig.add_subplot(111, projection="3d")

            # 颜色配置
            camera_colors = ['blue', 'cyan', 'magenta', 'yellow', 'orange', 'purple']
            lidar_colors = ['red', 'darkred', 'coral']
            imu_color = 'green'

            # 绘制相机视锥
            if camera_ids is None:
                camera_ids = [sid for sid, s in sensors.items() 
                             if s.sensor_type.value in ['camera_pinhole', 'camera_fisheye']]
            
            for idx, cam_id in enumerate(camera_ids):
                if cam_id not in intrinsics or cam_id not in sensors:
                    continue
                
                intr = intrinsics[cam_id]
                sensor = sensors[cam_id]
                
                if not hasattr(intr, "K"):
                    continue
                
                K = intr.K
                fx, fy = K[0, 0], K[1, 1]
                cx, cy = K[0, 2], K[1, 2]
                w, h = getattr(sensor, 'resolution', [1920, 1080])

                # 查找相机到参考系的外参
                R, t = np.eye(3), np.zeros(3)
                for (sa, sb), ext_result in extrinsics.items():
                    if sb == cam_id and sa == 'imu0':
                        R = ext_result.rotation
                        t = ext_result.translation
                        break

                color = camera_colors[idx % len(camera_colors)]
                
                # 绘制视锥
                frustum_depth = 5.0  # 米
                corners_img = np.array([
                    [0, 0, 1], [w, 0, 1], [w, h, 1], [0, h, 1]
                ], dtype=np.float32)
                
                corners_cam = (np.linalg.inv(K) @ corners_img.T).T * frustum_depth
                
                # 变换到世界坐标系
                corners_world = (R.T @ (corners_cam.T - t.reshape(3, 1))).T
                
                # 绘制视锥线
                origin_world = -R.T @ t
                for corner in corners_world:
                    ax.plot([origin_world[0], corner[0]],
                           [origin_world[1], corner[1]],
                           [origin_world[2], corner[2]], color=color, alpha=0.5, linewidth=1)
                
                # 绘制视锥底面
                verts = [[tuple(origin_world), tuple(corners_world[0]), tuple(corners_world[1])],
                        [tuple(origin_world), tuple(corners_world[1]), tuple(corners_world[2])],
                        [tuple(origin_world), tuple(corners_world[2]), tuple(corners_world[3])],
                        [tuple(origin_world), tuple(corners_world[3]), tuple(corners_world[0])]]
                ax.add_collection3d(Poly3DCollection(verts, color=color, alpha=0.2))

                # 相机位置标记
                ax.scatter(*origin_world, c=color, marker='s', s=200, label=cam_id)

            # 绘制 LiDAR 位置
            if lidar_ids is None:
                lidar_ids = [sid for sid, s in sensors.items() if s.sensor_type.value == 'lidar']
            
            for idx, lidar_id in enumerate(lidar_ids):
                if lidar_id not in extrinsics:
                    continue
                
                for (sa, sb), ext_result in extrinsics.items():
                    if sb == lidar_id and sa == 'imu0':
                        R = ext_result.rotation
                        t = ext_result.translation
                        pos_world = -R.T @ t
                        color = lidar_colors[idx % len(lidar_colors)]
                        ax.scatter(*pos_world, c=color, marker='o', s=300, label=lidar_id)
                        break

            # 原点参考系
            ax.scatter(0, 0, 0, c=imu_color, marker='D', s=300, label='IMU (Reference)')

            # 绘制坐标轴
            axis_length = 1.0
            ax.quiver(0, 0, 0, axis_length, 0, 0, color='red', arrow_length_ratio=0.1, linewidth=2)
            ax.quiver(0, 0, 0, 0, axis_length, 0, color='green', arrow_length_ratio=0.1, linewidth=2)
            ax.quiver(0, 0, 0, 0, 0, axis_length, color='blue', arrow_length_ratio=0.1, linewidth=2)

            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_zlabel('Z (m)', fontsize=12)
            ax.set_title('Sensor Frustums and Coordinate Systems', fontsize=14, fontweight='bold')
            ax.legend(loc='upper left', fontsize=10)
            
            # 设置相等的比例
            max_range = 5.0
            ax.set_xlim([-max_range, max_range])
            ax.set_ylim([-max_range, max_range])
            ax.set_zlim([-max_range, max_range])
            
            plt.tight_layout()
            out_path = self.output_dir / filename
            plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
            plt.close()
            logger.info(f"  Saved sensor frustums: {out_path}")

        except ImportError:
            logger.warning("matplotlib not available for sensor frustum plot.")
        except Exception as e:
            logger.warning(f"Failed to generate sensor frustums: {e}")

    def save_residual_plot(
        self,
        iteration_errors: List[float],
        title: str = "Optimization Residual",
        filename: str = "residual_convergence.png",
    ):
        """
        保存优化残差收敛曲线。
        
        Args:
            iteration_errors: 每次迭代的误差列表
            title: 图表标题
            filename: 输出文件名
        """
        if len(iteration_errors) == 0:
            logger.warning("No iteration errors to plot")
            return

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, 1, figsize=(10, 6))
            
            iterations = np.arange(1, len(iteration_errors) + 1)
            
            # 主曲线
            ax.plot(iterations, iteration_errors, 'b-', linewidth=2, marker='o', markersize=4, label='Residual')
            
            # 对数尺度曲线
            ax2 = ax.twinx()
            ax2.semilogy(iterations, iteration_errors, 'r--', linewidth=1.5, alpha=0.6, label='Log Scale')
            
            ax.set_xlabel('Iteration', fontsize=12)
            ax.set_ylabel('Residual', fontsize=12, color='blue')
            ax2.set_ylabel('Residual (log)', fontsize=12, color='red')
            ax.set_title(title, fontsize=14, fontweight='bold')
            
            # 合并图例
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
            
            ax.grid(True, alpha=0.3)
            
            # 统计信息
            if len(iteration_errors) > 1:
                improvement = (iteration_errors[0] - iteration_errors[-1]) / iteration_errors[0] * 100
                stats_text = (
                    f"Initial: {iteration_errors[0]:.4f}\n"
                    f"Final: {iteration_errors[-1]:.4f}\n"
                    f"Improvement: {improvement:.1f}%"
                )
                ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                       fontsize=10, verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
            
            plt.tight_layout()
            out_path = self.output_dir / filename
            plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
            plt.close()
            logger.info(f"  Saved residual convergence: {out_path}")

        except ImportError:
            logger.warning("matplotlib not available for residual plot.")

    def save_pointcloud_alignment(
        self,
        pts_source: np.ndarray,
        pts_target: np.ndarray,
        transform: Tuple[np.ndarray, np.ndarray],
        filename: str = "pointcloud_alignment.png",
    ):
        """
        保存点云对齐可视化（变换前后对比）。
        
        Args:
            pts_source: 源点云 (N×3)
            pts_target: 目标点云 (M×3)
            transform: 变换矩阵 (R, t)
            filename: 输出文件名
        """
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D

            fig = plt.figure(figsize=(15, 6))
            
            R, t = transform
            pts_transformed = (R @ pts_source.T + t.reshape(3, 1)).T

            # 子图1：变换前
            ax1 = fig.add_subplot(121, projection="3d")
            ax1.scatter(pts_source[:, 0], pts_source[:, 1], pts_source[:, 2],
                       c='blue', s=1, alpha=0.6, label='Source')
            ax1.scatter(pts_target[:, 0], pts_target[:, 1], pts_target[:, 2],
                       c='red', s=1, alpha=0.6, label='Target')
            ax1.set_xlabel('X')
            ax1.set_ylabel('Y')
            ax1.set_zlabel('Z')
            ax1.set_title('Before Alignment', fontweight='bold')
            ax1.legend()

            # 子图2：变换后
            ax2 = fig.add_subplot(122, projection="3d")
            ax2.scatter(pts_transformed[:, 0], pts_transformed[:, 1], pts_transformed[:, 2],
                       c='green', s=1, alpha=0.6, label='Transformed Source')
            ax2.scatter(pts_target[:, 0], pts_target[:, 1], pts_target[:, 2],
                       c='red', s=1, alpha=0.6, label='Target')
            ax2.set_xlabel('X')
            ax2.set_ylabel('Y')
            ax2.set_zlabel('Z')
            ax2.set_title('After Alignment', fontweight='bold')
            ax2.legend()

            plt.suptitle('Point Cloud Alignment Visualization', fontsize=14, fontweight='bold')
            plt.tight_layout()
            
            out_path = self.output_dir / filename
            plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
            plt.close()
            logger.info(f"  Saved pointcloud alignment: {out_path}")

        except ImportError:
            logger.warning("matplotlib not available for pointcloud alignment plot.")
        except Exception as e:
            logger.warning(f"Failed to generate pointcloud alignment: {e}")
