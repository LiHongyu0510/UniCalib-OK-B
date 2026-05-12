"""
传感器配置数据结构
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from enum import Enum, auto


class SensorType(Enum):
    CAMERA_PINHOLE = "camera_pinhole"
    CAMERA_FISHEYE = "camera_fisheye"
    LIDAR = "lidar"
    IMU = "imu"


class LidarType(Enum):
    SPINNING = "spinning"       # Velodyne, Ouster 旋转式
    SOLID_STATE = "solid_state"  # Livox 固态LiDAR


class CalibStage(Enum):
    INTRINSIC = "intrinsic"
    COARSE_EXTRINSIC = "coarse_extrinsic"
    FINE_EXTRINSIC = "fine_extrinsic"
    VALIDATION = "validation"


@dataclass
class SensorConfig:
    """单个传感器的配置"""
    sensor_id: str
    sensor_type: SensorType
    topic: str
    frame_id: str

    # 传感器特定参数
    rate: Optional[float] = None           # 采样频率 Hz
    resolution: Optional[Tuple[int, int]] = None  # 图像分辨率 [W, H]
    lidar_type: Optional[LidarType] = None

    # 已知先验（可选）
    intrinsic_prior: Optional[Dict] = None

    def __post_init__(self):
        if isinstance(self.sensor_type, str):
            self.sensor_type = SensorType(self.sensor_type)
        if isinstance(self.lidar_type, str) and self.lidar_type:
            self.lidar_type = LidarType(self.lidar_type)
        if self.resolution and isinstance(self.resolution, list):
            self.resolution = tuple(self.resolution)

    def is_camera(self) -> bool:
        return self.sensor_type in (SensorType.CAMERA_PINHOLE, SensorType.CAMERA_FISHEYE)

    def is_fisheye(self) -> bool:
        return self.sensor_type == SensorType.CAMERA_FISHEYE

    def is_lidar(self) -> bool:
        return self.sensor_type == SensorType.LIDAR

    def is_imu(self) -> bool:
        return self.sensor_type == SensorType.IMU


@dataclass
class CalibPair:
    """标定传感器对配置"""
    sensor_a: str
    sensor_b: str
    method_coarse: str   # 粗标定方法名
    method_fine: str     # 精标定方法名
    priority: int = 0    # 标定优先级 (值小的先执行)

    def key(self) -> Tuple[str, str]:
        return (self.sensor_a, self.sensor_b)


def auto_infer_calib_pairs(sensors: Dict[str, SensorConfig]) -> List[CalibPair]:
    """
    根据传感器类型组合自动推断所需的标定对和最优方法。

    标定优先级:
      1 = IMU-LiDAR     (最先，提供运动先验)
      2 = LiDAR-Camera  (依赖IMU先验)
      3 = Camera-Camera (最后，依赖各相机内参)
    """
    from itertools import combinations

    pairs: List[CalibPair] = []
    sensor_list = list(sensors.values())

    for sa, sb in combinations(sensor_list, 2):
        pair = _select_method(sa, sb)
        if pair is not None:
            pairs.append(pair)

    pairs.sort(key=lambda p: p.priority)
    return pairs


def _select_method(sa: SensorConfig, sb: SensorConfig) -> Optional[CalibPair]:
    """根据传感器类型对选择最优标定方法。"""
    type_pair = frozenset({sa.sensor_type, sb.sensor_type})

    # IMU + LiDAR → learn-to-calibrate (RL粗) + iKalibr B-spline (精)
    if type_pair == frozenset({SensorType.IMU, SensorType.LIDAR}):
        return CalibPair(
            sa.sensor_id, sb.sensor_id,
            method_coarse="l2calib_rl_init",
            method_fine="ikalibr_bspline",
            priority=1,
        )

    # LiDAR + 普通相机 → MIAS-LCEC (无靶粗) + MIAS-LCEC (精化)
    if type_pair == frozenset({SensorType.LIDAR, SensorType.CAMERA_PINHOLE}):
        return CalibPair(
            sa.sensor_id, sb.sensor_id,
            method_coarse="mias_lcec_coarse",
            method_fine="mias_lcec_fine",
            priority=2,
        )

    # LiDAR + 鱼眼相机 → MIAS-LCEC (粗) + iKalibr B-spline (精)
    if type_pair == frozenset({SensorType.LIDAR, SensorType.CAMERA_FISHEYE}):
        return CalibPair(
            sa.sensor_id, sb.sensor_id,
            method_coarse="mias_lcec_coarse",
            method_fine="ikalibr_bspline",
            priority=2,
        )

    # 普通相机 + 普通相机 → 特征匹配 (粗) + click_calib 全局BA (精)
    if type_pair == frozenset({SensorType.CAMERA_PINHOLE}):
        return CalibPair(
            sa.sensor_id, sb.sensor_id,
            method_coarse="feature_matching",
            method_fine="click_calib_ba",
            priority=3,
        )

    # 鱼眼相机 + 鱼眼相机 → 特征匹配 (粗) + click_calib 全局BA (精)
    if type_pair == frozenset({SensorType.CAMERA_FISHEYE}):
        return CalibPair(
            sa.sensor_id, sb.sensor_id,
            method_coarse="feature_matching",
            method_fine="click_calib_ba",
            priority=3,
        )

    # 普通相机 + 鱼眼相机 → 特征匹配 (粗) + click_calib 全局BA (精)
    if type_pair == frozenset({SensorType.CAMERA_PINHOLE, SensorType.CAMERA_FISHEYE}):
        return CalibPair(
            sa.sensor_id, sb.sensor_id,
            method_coarse="feature_matching",
            method_fine="click_calib_ba",
            priority=3,
        )

    return None
