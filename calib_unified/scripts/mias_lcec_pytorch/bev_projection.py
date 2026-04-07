"""
BEV (Bird's Eye View) 投影模块

将 3D 点云投影到 2D BEV 图像，用于 Overlap Transformer 特征提取。
支持多种投影方式和特征通道。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

_LOG = logging.getLogger(__name__)


@dataclass
class BEVConfig:
    """BEV 投影配置"""

    # 分辨率 (米/像素)
    resolution: float = 0.1
    # BEV 图像尺寸
    width: int = 1024
    height: int = 1024
    # 高度范围 (米)
    height_min: float = -3.0
    height_max: float = 3.0
    # 点云范围 (米)，None 表示自动计算
    x_min: Optional[float] = None
    x_max: Optional[float] = None
    y_min: Optional[float] = None
    y_max: Optional[float] = None
    # 特征通道数
    feature_channels: int = 5
    # 是否使用强度值
    use_intensity: bool = True


class BEVProjector:
    """
    BEV 投影器

    将 3D 点云投影到 2D BEV 图像，支持多通道特征：
    - 通道 0: 点数 (归一化)
    - 通道 1: 高度均值
    - 通道 2: 高度最大值
    - 通道 3: 强度均值 (如果有)
    - 通道 4: 深度/距离
    """

    def __init__(self, config: Optional[BEVConfig] = None):
        self.config = config or BEVConfig()

    def project(
        self,
        points: np.ndarray,
        intensities: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, dict]:
        """
        将点云投影到 BEV 图像

        Args:
            points: (N, 3+) 点云坐标，至少包含 x, y, z
            intensities: (N,) 可选的强度值

        Returns:
            bev_image: (H, W, C) BEV 特征图像
            meta: 元数据字典，包含范围、分辨率等信息
        """
        cfg = self.config

        # 提取 xyz
        xyz = points[:, :3].astype(np.float64)

        # 过滤高度范围
        height_mask = (xyz[:, 2] >= cfg.height_min) & (xyz[:, 2] <= cfg.height_max)
        xyz_filtered = xyz[height_mask]

        if len(xyz_filtered) == 0:
            _LOG.warning("[BEV] 过滤后无有效点 (height_range=[%.1f, %.1f] 输入点数=%d)",
                         cfg.height_min, cfg.height_max, len(points))
            return np.zeros((cfg.height, cfg.width, cfg.feature_channels), dtype=np.float32), {}

        # 过滤强度
        intensities_filtered = None
        if intensities is not None and len(intensities) == len(points):
            intensities_filtered = intensities[height_mask]

        # 计算范围
        x_min = cfg.x_min if cfg.x_min is not None else xyz_filtered[:, 0].min()
        x_max = cfg.x_max if cfg.x_max is not None else xyz_filtered[:, 0].max()
        y_min = cfg.y_min if cfg.y_min is not None else xyz_filtered[:, 1].min()
        y_max = cfg.y_max if cfg.y_max is not None else xyz_filtered[:, 1].max()

        # 添加边距
        margin = cfg.resolution * 2
        x_min -= margin
        x_max += margin
        y_min -= margin
        y_max += margin

        # 计算像素坐标
        # BEV 图像: x -> width, y -> height (y 轴向下)
        pixel_x = ((xyz_filtered[:, 0] - x_min) / cfg.resolution).astype(np.int32)
        pixel_y = ((xyz_filtered[:, 1] - y_min) / cfg.resolution).astype(np.int32)

        # 过滤超出范围的点
        valid_mask = (
            (pixel_x >= 0) & (pixel_x < cfg.width) &
            (pixel_y >= 0) & (pixel_y < cfg.height)
        )
        pixel_x = pixel_x[valid_mask]
        pixel_y = pixel_y[valid_mask]
        z_values = xyz_filtered[valid_mask, 2]
        dist_values = np.sqrt(
            xyz_filtered[valid_mask, 0] ** 2 +
            xyz_filtered[valid_mask, 1] ** 2 +
            xyz_filtered[valid_mask, 2] ** 2
        )

        intensities_valid = None
        if intensities_filtered is not None:
            intensities_valid = intensities_filtered[valid_mask]

        # 初始化 BEV 特征图
        C = cfg.feature_channels
        bev = np.zeros((cfg.height, cfg.width, C), dtype=np.float32)

        # 通道 0: 点数 (稍后归一化)
        # 通道 1: 高度均值
        # 通道 2: 高度最大值
        # 通道 3: 强度均值
        # 通道 4: 距离均值

        # 使用 bincount 统计每个像素
        if len(pixel_x) > 0:
            # 线性索引
            linear_idx = pixel_y * cfg.width + pixel_x

            # 点数统计
            counts = np.bincount(linear_idx, minlength=cfg.width * cfg.height)
            counts = counts.reshape(cfg.height, cfg.width)
            bev[:, :, 0] = counts.astype(np.float32)

            # 高度均值
            z_sum = np.bincount(linear_idx, weights=z_values, minlength=cfg.width * cfg.height)
            z_sum = z_sum.reshape(cfg.height, cfg.width)
            mask = counts > 0
            bev[mask, 1] = (z_sum[mask] / counts[mask]).astype(np.float32)

            # 高度最大值 - 需要逐像素处理
            z_max = np.full((cfg.height, cfg.width), cfg.height_min, dtype=np.float32)
            for i in range(len(pixel_x)):
                py, px = pixel_y[i], pixel_x[i]
                if z_values[i] > z_max[py, px]:
                    z_max[py, px] = z_values[i]
            bev[:, :, 2] = z_max

            # 强度均值
            if intensities_valid is not None and C > 3:
                i_sum = np.bincount(
                    linear_idx, weights=intensities_valid, minlength=cfg.width * cfg.height
                )
                i_sum = i_sum.reshape(cfg.height, cfg.width)
                bev[mask, 3] = (i_sum[mask] / counts[mask]).astype(np.float32)

            # 距离均值
            if C > 4:
                d_sum = np.bincount(linear_idx, weights=dist_values, minlength=cfg.width * cfg.height)
                d_sum = d_sum.reshape(cfg.height, cfg.width)
                bev[mask, 4] = (d_sum[mask] / counts[mask]).astype(np.float32)

        # 归一化点数
        max_count = bev[:, :, 0].max()
        if max_count > 0:
            bev[:, :, 0] = bev[:, :, 0] / max_count

        # 归一化其他通道
        for c in [1, 2, 3, 4]:
            if c < C:
                v_min, v_max = bev[:, :, c].min(), bev[:, :, c].max()
                if v_max > v_min:
                    bev[:, :, c] = (bev[:, :, c] - v_min) / (v_max - v_min)

        meta = {
            "x_range": (x_min, x_max),
            "y_range": (y_min, y_max),
            "resolution": cfg.resolution,
            "width": cfg.width,
            "height": cfg.height,
            "n_points": len(xyz_filtered),
            "n_valid": len(pixel_x),
        }
        _LOG.debug("[BEV] 投影完成 shape=%s x_range=(%.2f,%.2f) y_range=(%.2f,%.2f) n_valid=%d",
                   (cfg.height, cfg.width, cfg.feature_channels), x_min, x_max, y_min, y_max, len(pixel_x))

        return bev, meta

    def get_point_pixel_mapping(
        self,
        points: np.ndarray,
        meta: dict,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取点云到 BEV 像素的映射关系

        Args:
            points: (N, 3) 点云坐标
            meta: project() 返回的元数据

        Returns:
            pixel_coords: (M, 2) 有效点的像素坐标 (y, x)
            valid_indices: (M,) 有效点的索引
        """
        xyz = points[:, :3].astype(np.float64)

        x_min, x_max = meta["x_range"]
        y_min, y_max = meta["y_range"]
        resolution = meta["resolution"]
        width, height = meta["width"], meta["height"]

        # 计算像素坐标
        pixel_x = ((xyz[:, 0] - x_min) / resolution).astype(np.int32)
        pixel_y = ((xyz[:, 1] - y_min) / resolution).astype(np.int32)

        # 过滤有效点
        valid_mask = (
            (pixel_x >= 0) & (pixel_x < width) &
            (pixel_y >= 0) & (pixel_y < height) &
            (xyz[:, 2] >= self.config.height_min) &
            (xyz[:, 2] <= self.config.height_max)
        )

        valid_indices = np.where(valid_mask)[0]
        pixel_coords = np.column_stack([pixel_y[valid_mask], pixel_x[valid_mask]])

        return pixel_coords, valid_indices


def point_cloud_to_bev(
    points: np.ndarray,
    resolution: float = 0.1,
    width: int = 1024,
    height: int = 1024,
    height_range: Tuple[float, float] = (-3.0, 3.0),
    intensities: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, dict]:
    """
    便捷函数：将点云投影到 BEV 图像

    Args:
        points: (N, 3+) 点云坐标
        resolution: 分辨率 (米/像素)
        width: BEV 图像宽度
        height: BEV 图像高度
        height_range: 高度范围 (min, max)
        intensities: 可选的强度值

    Returns:
        bev_image: (H, W, 5) BEV 特征图像
        meta: 元数据字典
    """
    config = BEVConfig(
        resolution=resolution,
        width=width,
        height=height,
        height_min=height_range[0],
        height_max=height_range[1],
    )
    projector = BEVProjector(config)
    return projector.project(points, intensities)


def load_pcd_as_bev(
    pcd_path: str,
    config: Optional[BEVConfig] = None,
) -> Tuple[np.ndarray, np.ndarray, dict]:
    """
    从 PCD 文件加载点云并转换为 BEV

    Args:
        pcd_path: PCD 文件路径
        config: BEV 配置

    Returns:
        points: (N, 3+) 点云坐标
        bev_image: (H, W, C) BEV 特征图像
        meta: 元数据字典
    """
    # 读取 PCD
    points = _read_pcd(pcd_path)

    # 投影到 BEV
    config = config or BEVConfig()
    projector = BEVProjector(config)
    bev, meta = projector.project(points)

    return points, bev, meta


def _read_pcd(pcd_path: str) -> np.ndarray:
    """
    读取 PCD 文件 (支持 ASCII 和二进制格式)
    """
    with open(pcd_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("utf-8", errors="ignore").strip()
            header_lines.append(line)
            if line.lower().startswith("data"):
                break

        # 解析 header
        header = {}
        for line in header_lines:
            parts = line.split()
            if len(parts) >= 2:
                key = parts[0].lower()
                if key == "points":
                    header["points"] = int(parts[1])
                elif key == "fields":
                    header["fields"] = parts[1:]
                elif key == "size":
                    header["sizes"] = [int(x) for x in parts[1:]]
                elif key == "type":
                    header["types"] = parts[1:]
                elif key == "data":
                    header["data_type"] = parts[1].lower()

        n_points = header.get("points", 0)
        fields = header.get("fields", ["x", "y", "z"])
        data_type = header.get("data_type", "ascii")

        if data_type == "ascii":
            # ASCII 格式
            points = []
            for _ in range(n_points):
                line = f.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    values = line.split()
                    if len(values) >= 3:
                        points.append([float(values[0]), float(values[1]), float(values[2])])
            return np.array(points, dtype=np.float64)
        else:
            # 二进制格式 - 简化处理，只读取 xyz
            import struct

            points = []
            # 假设每个点 16 字节 (x, y, z, intensity 或 padding)
            for _ in range(n_points):
                data = f.read(16)
                if len(data) >= 12:
                    x, y, z = struct.unpack("fff", data[:12])
                    points.append([x, y, z])
            return np.array(points, dtype=np.float64)
