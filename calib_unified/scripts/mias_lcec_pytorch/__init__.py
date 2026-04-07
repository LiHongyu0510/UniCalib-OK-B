"""
MIAS-LCEC 纯 PyTorch 实现

基于 OverlapTransformer + MobileSAM + C3M 算法，实现 LiDAR-Camera 外参粗标定。
彻底摆脱 .so 库和 Python 3.10 依赖。

核心组件:
- bev_projection: 点云 BEV 投影
- overlap_transformer: Overlap Transformer 网络定义
- mobile_sam_wrapper: MobileSAM 分割封装
- c3m_matching: Cross-Modal Mask Matching
- coarse_calib: 主入口
- initial_guess_estimator: 初始值评估模块
- adaptive_search: 自适应搜索配置
"""

from .coarse_calib import MIASLCECCoarseCalib
from .bev_projection import point_cloud_to_bev, BEVProjector
from .overlap_transformer import OverlapTransformer
from .c3m_matching import C3MMatcher
from .initial_guess_estimator import (
    InitialGuessEstimator,
    InitialGuessConfig,
    InitialGuessResult,
    SearchRange,
    VehicleMountPrior,
    SensorFOV,
    ConfidenceLevel,
    estimate_initial_guess,
    get_default_search_range_for_mounting,
)
from .quality_assessment import (
    QualityReport,
    QualityVerdict,
    QualityThresholds,
    assess_calibration_quality,
)

__all__ = [
    "MIASLCECCoarseCalib",
    "point_cloud_to_bev",
    "BEVProjector",
    "OverlapTransformer",
    "C3MMatcher",
    # 初始值评估模块
    "InitialGuessEstimator",
    "InitialGuessConfig",
    "InitialGuessResult",
    "SearchRange",
    "VehicleMountPrior",
    "SensorFOV",
    "ConfidenceLevel",
    "estimate_initial_guess",
    "get_default_search_range_for_mounting",
    "QualityReport",
    "QualityVerdict",
    "QualityThresholds",
    "assess_calibration_quality",
]

__version__ = "0.2.0"
