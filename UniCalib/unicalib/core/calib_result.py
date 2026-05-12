"""
标定结果数据结构
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
import json
import yaml


@dataclass
class CameraIntrinsic:
    """相机内参 (针孔模型)"""
    K: np.ndarray           # 3×3 内参矩阵
    dist_coeffs: np.ndarray # 畸变系数 [k1,k2,p1,p2,k3] 或更多
    image_size: Tuple[int, int]  # (width, height)
    reprojection_error: float = float('inf')
    method: str = ""

    def to_dict(self) -> dict:
        return {
            "K": self.K.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "image_size": list(self.image_size),
            "reprojection_error": self.reprojection_error,
            "method": self.method,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CameraIntrinsic":
        return cls(
            K=np.array(d["K"]),
            dist_coeffs=np.array(d["dist_coeffs"]),
            image_size=tuple(d["image_size"]),
            reprojection_error=d.get("reprojection_error", float('inf')),
            method=d.get("method", ""),
        )

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


@dataclass
class FisheyeIntrinsic:
    """鱼眼相机内参 (多模型支持)"""
    model_type: str      # "eucm" / "double_sphere" / "kannala_brandt" / "equidistant"
    params: Dict[str, float]   # 模型参数 (依模型而异)
    image_size: Tuple[int, int]
    reprojection_error: float = float('inf')
    method: str = ""

    def to_dict(self) -> dict:
        return {
            "model_type": self.model_type,
            "params": self.params,
            "image_size": list(self.image_size),
            "reprojection_error": self.reprojection_error,
            "method": self.method,
        }


@dataclass
class IMUIntrinsic:
    """IMU 内参"""
    # 噪声参数 (Allan 方差得到)
    gyro_noise: float = 0.0      # 角度随机游走 ARW [rad/s/√Hz]
    gyro_bias_instability: float = 0.0  # 零偏不稳定性 [rad/s]
    accel_noise: float = 0.0     # 速度随机游走 VRW [m/s²/√Hz]
    accel_bias_instability: float = 0.0

    # 零偏
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # 误差模型矩阵 (完整12参数模型)
    Ma: np.ndarray = field(default_factory=lambda: np.eye(3))   # 加速度计非正交
    Sa: np.ndarray = field(default_factory=lambda: np.eye(3))   # 加速度计比例
    Mg: np.ndarray = field(default_factory=lambda: np.eye(3))   # 陀螺仪非正交
    Sg: np.ndarray = field(default_factory=lambda: np.eye(3))   # 陀螺仪比例
    Tg: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))  # g-sensitivity

    method: str = ""

    def to_dict(self) -> dict:
        return {
            "gyro_noise": self.gyro_noise,
            "gyro_bias_instability": self.gyro_bias_instability,
            "accel_noise": self.accel_noise,
            "accel_bias_instability": self.accel_bias_instability,
            "gyro_bias": self.gyro_bias.tolist(),
            "accel_bias": self.accel_bias.tolist(),
            "Ma": self.Ma.tolist(),
            "Sa": self.Sa.tolist(),
            "Mg": self.Mg.tolist(),
            "Sg": self.Sg.tolist(),
            "Tg": self.Tg.tolist(),
            "method": self.method,
        }


# 联合内参结果类型
IntrinsicResult = CameraIntrinsic | FisheyeIntrinsic | IMUIntrinsic


@dataclass
class CalibResult:
    """传感器对外参标定结果"""
    pair: Tuple[str, str]        # (sensor_a_id, sensor_b_id)
    rotation: np.ndarray         # 3×3 旋转矩阵 (sensor_a → sensor_b)
    translation: np.ndarray      # 3×1 平移向量
    time_offset: float = 0.0     # 时间偏移 td [秒], sensor_b 相对于 sensor_a
    reprojection_error: float = float('inf')
    confidence: float = 0.0
    method_used: str = ""

    def rotation_euler_deg(self) -> np.ndarray:
        """返回 ZYX 欧拉角 [°]"""
        from scipy.spatial.transform import Rotation
        return Rotation.from_matrix(self.rotation).as_euler('zyx', degrees=True)

    def to_dict(self) -> dict:
        return {
            "pair": list(self.pair),
            "rotation": self.rotation.tolist(),
            "translation": self.translation.tolist(),
            "time_offset": self.time_offset,
            "reprojection_error": self.reprojection_error,
            "confidence": self.confidence,
            "method_used": self.method_used,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CalibResult":
        return cls(
            pair=tuple(d["pair"]),
            rotation=np.array(d["rotation"]),
            translation=np.array(d["translation"]),
            time_offset=d.get("time_offset", 0.0),
            reprojection_error=d.get("reprojection_error", float('inf')),
            confidence=d.get("confidence", 0.0),
            method_used=d.get("method_used", ""),
        )


@dataclass
class ValidationMetrics:
    """单个传感器对的验证指标"""
    pair_key: str
    mean_error_px: float = float('inf')
    median_error_px: float = float('inf')
    std_error_px: float = float('inf')
    max_error_px: float = float('inf')
    pct_within_1px: float = 0.0
    pct_within_3px: float = 0.0
    pass_threshold: bool = False
    extra: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ValidationReport:
    """全局验证报告"""
    metrics: Dict[str, ValidationMetrics] = field(default_factory=dict)
    overall_pass: bool = False
    summary: str = ""

    def add_metric(self, key: str, data: dict):
        m = ValidationMetrics(pair_key=key)
        m.mean_error_px = data.get("mean_error_px", float('inf'))
        m.median_error_px = data.get("median_error_px", float('inf'))
        m.std_error_px = data.get("std_error_px", float('inf'))
        m.max_error_px = data.get("max_error_px", float('inf'))
        m.pct_within_1px = data.get("pct_within_1px", 0.0)
        m.pct_within_3px = data.get("pct_within_3px", 0.0)
        m.extra = {k: v for k, v in data.items()
                   if k not in {"mean_error_px", "median_error_px",
                                "std_error_px", "max_error_px",
                                "pct_within_1px", "pct_within_3px"}}
        self.metrics[key] = m

    def to_dict(self) -> dict:
        return {
            "overall_pass": self.overall_pass,
            "summary": self.summary,
            "metrics": {
                k: {
                    "mean_error_px": v.mean_error_px,
                    "median_error_px": v.median_error_px,
                    "std_error_px": v.std_error_px,
                    "max_error_px": v.max_error_px,
                    "pct_within_1px": v.pct_within_1px,
                    "pct_within_3px": v.pct_within_3px,
                    **v.extra,
                }
                for k, v in self.metrics.items()
            },
        }

    def save_yaml(self, path: str):
        with open(path, "w") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)
