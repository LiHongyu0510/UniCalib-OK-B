"""
实时可视化工具
基于 Open3D 实现实时点云渲染、传感器坐标系可视化
"""
from __future__ import annotations
import logging
from typing import Dict, Optional, Tuple
import threading
import time
from queue import Queue

import numpy as np

logger = logging.getLogger(__name__)


class RealtimeVisualizer:
    """基于 Open3D 的实时可视化器。"""

    def __init__(self, window_name: str = "UniCalib Realtime Visualization"):
        self.window_name = window_name
        self.running = False
        self.queue = Queue(maxsize=10)
        self.thread = None
        self._try_init_open3d()

    def _try_init_open3d(self):
        """尝试初始化 Open3D。"""
        try:
            import open3d as o3d
            self.o3d = o3d
            self.vis = self.o3d.visualization.Visualizer()
            self.vis.create_window(window_name=self.window_name, width=1280, height=720)
            self.running = True
            
            # 初始化几何体
            self.geometries = {}
            
            # 启动渲染线程
            self.thread = threading.Thread(target=self._render_loop, daemon=True)
            self.thread.start()
            logger.info("Open3D realtime visualizer initialized successfully")
        except ImportError:
            self.o3d = None
            self.vis = None
            logger.warning("Open3D not available, realtime visualization disabled")
        except Exception as e:
            self.o3d = None
            self.vis = None
            logger.error(f"Failed to initialize Open3D: {e}")

    def is_available(self) -> bool:
        """检查可视化器是否可用。"""
        return self.o3d is not None and self.vis is not None

    def update_pointcloud(
        self,
        name: str,
        points: np.ndarray,
        colors: Optional[np.ndarray] = None,
        point_size: float = 2.0,
    ):
        """
        更新点云数据。
        
        Args:
            name: 点云名称（用于更新或创建）
            points: 点云坐标 (N×3)
            colors: 点云颜色 (N×3), 范围 [0,1]
            point_size: 点大小
        """
        if not self.is_available():
            return

        try:
            pcd = self.o3d.geometry.PointCloud()
            pcd.points = self.o3d.utility.Vector3dVector(points)
            
            if colors is not None:
                pcd.colors = self.o3d.utility.Vector3dVector(colors)
            else:
                # 默认深度着色
                depths = np.linalg.norm(points, axis=1)
                max_depth = np.percentile(depths, 95) + 1e-6
                norm_depth = np.clip(depths / max_depth, 0, 1)
                colors = np.zeros_like(points)
                colors[:, 0] = norm_depth  # R
                colors[:, 1] = 1.0 - norm_depth  # G
                colors[:, 2] = 0.2  # B
                pcd.colors = self.o3d.utility.Vector3dVector(colors)
            
            self.queue.put(('update_pointcloud', name, pcd, point_size))
            
        except Exception as e:
            logger.error(f"Failed to update pointcloud {name}: {e}")

    def update_frustum(
        self,
        name: str,
        R: np.ndarray,
        t: np.ndarray,
        K: np.ndarray,
        image_size: Tuple[int, int],
        depth: float = 5.0,
        color: Tuple[float, float, float] = (0.0, 1.0, 1.0),
    ):
        """
        更新相机视锥。
        
        Args:
            name: 视锥名称
            R: 旋转矩阵 (3×3)
            t: 平移向量 (3,)
            K: 相机内参 (3×3)
            image_size: 图像尺寸 (width, height)
            depth: 视锥深度
            color: RGB 颜色
        """
        if not self.is_available():
            return

        try:
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            w, h = image_size

            # 图像角点
            corners_img = np.array([
                [0, 0, 1], [w, 0, 1], [w, h, 1], [0, h, 1]
            ], dtype=np.float64)
            
            # 变换到相机坐标系
            corners_cam = (np.linalg.inv(K) @ corners_img.T).T * depth
            corners_world = (R.T @ (corners_cam.T - t.reshape(3, 1))).T
            origin_world = -R.T @ t

            # 创建线条
            lines = []
            for corner in corners_world:
                lines.append(self.o3d.geometry.LineSet())
                lines[-1].points = self.o3d.utility.Vector2dVector([origin_world, corner])
                lines[-1].lines = self.o3d.utility.Vector2iVector([[0, 1]])
                lines[-1].colors = self.o3d.utility.Vector3dVector([list(color)])

            self.queue.put(('update_frustum', name, lines))
            
        except Exception as e:
            logger.error(f"Failed to update frustum {name}: {e}")

    def update_coordinate_frame(
        self,
        name: str,
        R: np.ndarray,
        t: np.ndarray,
        scale: float = 1.0,
    ):
        """
        更新坐标系。
        
        Args:
            name: 坐标系名称
            R: 旋转矩阵
            t: 平移向量
            scale: 坐标轴长度
        """
        if not self.is_available():
            return

        try:
            origin = -R.T @ t
            
            # 创建三条坐标轴线
            lines = []
            colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # RGB for XYZ
            
            for i in range(3):
                line = self.o3d.geometry.LineSet()
                axis_end = origin + R.T[:, i] * scale
                line.points = self.o3d.utility.Vector2dVector([origin, axis_end])
                line.lines = self.o3d.utility.Vector2iVector([[0, 1]])
                line.colors = self.o3d.utility.Vector3dVector([colors[i]])
                lines.append(line)
            
            self.queue.put(('update_coordinate', name, lines))
            
        except Exception as e:
            logger.error(f"Failed to update coordinate frame {name}: {e}")

    def add_trajectory(
        self,
        name: str,
        positions: np.ndarray,
        color: Tuple[float, float, float] = (1.0, 1.0, 0.0),
        line_width: float = 2.0,
    ):
        """
        添加轨迹线。
        
        Args:
            name: 轨迹名称
            positions: 轨迹点坐标 (N×3)
            color: 线条颜色
            line_width: 线宽
        """
        if not self.is_available():
            return

        try:
            line = self.o3d.geometry.LineSet()
            line.points = self.o3d.utility.Vector3dVector(positions)
            
            # 创建线条索引
            lines = []
            for i in range(len(positions) - 1):
                lines.append([i, i + 1])
            line.lines = self.o3d.utility.Vector2iVector(lines)
            
            # 设置颜色
            colors = np.tile(color, (len(lines), 1))
            line.colors = self.o3d.utility.Vector3dVector(colors)
            
            self.queue.put(('add_trajectory', name, line))
            
        except Exception as e:
            logger.error(f"Failed to add trajectory {name}: {e}")

    def clear_geometry(self, name: Optional[str] = None):
        """
        清除几何体。
        
        Args:
            name: 要清除的几何体名称，None表示全部清除
        """
        if not self.is_available():
            return

        self.queue.put(('clear', name))

    def set_view_parameters(
        self,
        front: Tuple[float, float, float] = (0.0, 0.0, -1.0),
        lookat: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        up: Tuple[float, float, float] = (0.0, -1.0, 0.0),
        zoom: float = 0.5,
    ):
        """设置相机视角参数。"""
        if not self.is_available():
            return

        self.queue.put(('set_view', front, lookat, up, zoom))

    def _render_loop(self):
        """渲染循环（在独立线程中运行）。"""
        while self.running:
            try:
                # 处理队列中的更新
                while not self.queue.empty():
                    self._process_queue_item()
                
                # 渲染一帧
                if self.vis.poll_events():
                    self.vis.update_renderer()
                
                time.sleep(0.01)  # 10ms
                
            except Exception as e:
                logger.error(f"Render loop error: {e}")
                time.sleep(0.1)

    def _process_queue_item(self):
        """处理队列中的单个更新项。"""
        try:
            item = self.queue.get_nowait()
            cmd = item[0]
            
            if cmd == 'update_pointcloud':
                _, name, pcd, point_size = item
                if name in self.geometries:
                    self.vis.remove_geometry(self.geometries[name], reset_bounding_box=False)
                self.geometries[name] = pcd
                self.vis.add_geometry(pcd, reset_bounding_box=False)
                
            elif cmd == 'update_frustum':
                _, name, lines = item
                for i, line in enumerate(lines):
                    geo_name = f"{name}_line_{i}"
                    if geo_name in self.geometries:
                        self.vis.remove_geometry(self.geometries[geo_name], reset_bounding_box=False)
                    self.geometries[geo_name] = line
                    self.vis.add_geometry(line, reset_bounding_box=False)
                    
            elif cmd == 'update_coordinate':
                _, name, lines = item
                for i, line in enumerate(lines):
                    geo_name = f"{name}_axis_{i}"
                    if geo_name in self.geometries:
                        self.vis.remove_geometry(self.geometries[geo_name], reset_bounding_box=False)
                    self.geometries[geo_name] = line
                    self.vis.add_geometry(line, reset_bounding_box=False)
                    
            elif cmd == 'add_trajectory':
                _, name, line = item
                if name in self.geometries:
                    self.vis.remove_geometry(self.geometries[name], reset_bounding_box=False)
                self.geometries[name] = line
                self.vis.add_geometry(line, reset_bounding_box=False)
                
            elif cmd == 'clear':
                _, name = item
                if name is None:
                    # 清除所有
                    for geo_name, geo in self.geometries.items():
                        self.vis.remove_geometry(geo, reset_bounding_box=False)
                    self.geometries.clear()
                elif name in self.geometries:
                    self.vis.remove_geometry(self.geometries[name], reset_bounding_box=False)
                    del self.geometries[name]
                    
            elif cmd == 'set_view':
                _, front, lookat, up, zoom = item
                self.vis.get_view_control().set_front(front)
                self.vis.get_view_control().set_lookat(lookat)
                self.vis.get_view_control().set_up(up)
                self.vis.get_view_control().set_zoom(zoom)
            
        except Exception as e:
            logger.error(f"Failed to process queue item: {e}")

    def close(self):
        """关闭可视化器。"""
        self.running = False
        if self.vis is not None:
            self.vis.destroy_window()
        if self.thread is not None:
            self.thread.join(timeout=1.0)


class ProcessVisualizer:
    """标定过程可视化器（显示优化进度、残差等）。"""

    def __init__(self, max_iterations: int = 100):
        self.max_iterations = max_iterations
        self.iteration_errors = []
        self._try_init()

    def _try_init(self):
        """尝试初始化绘图库。"""
        try:
            import matplotlib
            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
            
            self.plt = plt
            self.fig, self.ax = self.plt.subplots(figsize=(10, 6))
            self.ax.set_xlim(0, self.max_iterations)
            self.ax.set_ylim(0, 1.0)
            self.ax.set_xlabel('Iteration')
            self.ax.set_ylabel('Residual')
            self.ax.set_title('Calibration Optimization Progress')
            self.ax.grid(True, alpha=0.3)
            
            self.line, = self.ax.plot([], [], 'b-', linewidth=2, marker='o')
            self.plt.ion()
            self.plt.show()
            
            logger.info("Process visualizer initialized")
            
        except ImportError:
            self.plt = None
            logger.warning("matplotlib not available for process visualization")
        except Exception as e:
            self.plt = None
            logger.error(f"Failed to initialize process visualizer: {e}")

    def update(self, iteration: int, error: float):
        """更新优化进度。"""
        self.iteration_errors.append(error)
        
        if self.plt is not None:
            try:
                self.line.set_data(range(len(self.iteration_errors)), self.iteration_errors)
                self.ax.set_xlim(0, max(self.max_iterations, len(self.iteration_errors)))
                self.ax.set_ylim(0, max(self.iteration_errors) * 1.1)
                self.plt.pause(0.01)
            except Exception as e:
                logger.error(f"Failed to update process visualizer: {e}")

    def save(self, filename: str):
        """保存当前图表。"""
        if self.plt is not None:
            try:
                self.plt.savefig(filename, dpi=150, bbox_inches='tight')
                logger.info(f"Process visualization saved: {filename}")
            except Exception as e:
                logger.error(f"Failed to save process visualization: {e}")

    def close(self):
        """关闭可视化器。"""
        if self.plt is not None:
            self.plt.close()


class HybridVisualizer:
    """混合可视化器：结合实时3D可视化和过程图表。"""

    def __init__(self, enable_3d: bool = True, enable_process: bool = True):
        self.rt_viz = RealtimeVisualizer() if enable_3d else None
        self.proc_viz = ProcessVisualizer() if enable_process else None
        self.iteration = 0

    def update_pointcloud(self, *args, **kwargs):
        """更新点云。"""
        if self.rt_viz is not None:
            self.rt_viz.update_pointcloud(*args, **kwargs)

    def update_process(self, error: float):
        """更新优化过程。"""
        if self.proc_viz is not None:
            self.iteration += 1
            self.proc_viz.update(self.iteration, error)

    def close(self):
        """关闭所有可视化器。"""
        if self.rt_viz is not None:
            self.rt_viz.close()
        if self.proc_viz is not None:
            self.proc_viz.close()
