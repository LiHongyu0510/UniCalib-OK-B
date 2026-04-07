"""
初始值评估模块

在没有标定初始值的情况下，评估传感器配置并生成合理的搜索范围。
参考最新研究:
- MDPCalib (2024): 运动估计 + 深度点对应
- CalibRefine (2025): 迭代精化 + 注意力机制
- RAVES-Calib (2025): 自动特征对应

核心功能:
1. 基于传感器FOV估计重叠区域
2. 基于车辆安装位置先验生成搜索范围
3. 深度特征匹配估计粗略位姿
4. 置信度评估
"""

from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

_LOG = logging.getLogger(__name__)

# 延迟导入
_torch_available = False
try:
    import torch
    import torch.nn.functional as F
    _torch_available = True
except ImportError:
    pass


class ConfidenceLevel(Enum):
    """置信度等级"""
    HIGH = "high"      # > 0.8
    MEDIUM = "medium"  # 0.5 - 0.8
    LOW = "low"        # 0.3 - 0.5
    VERY_LOW = "very_low"  # < 0.3


@dataclass
class SensorFOV:
    """传感器视场角配置"""
    # 相机参数
    cam_h_fov_deg: float = 90.0   # 相机水平视场角 (度)
    cam_v_fov_deg: float = 60.0   # 相机垂直视场角 (度)
    # LiDAR参数
    lidar_h_fov_deg: float = 360.0  # LiDAR水平视场角 (度)
    lidar_v_fov_deg: float = 30.0   # LiDAR垂直视场角 (度)
    lidar_min_range: float = 0.3    # LiDAR最小距离 (米)
    lidar_max_range: float = 150.0  # LiDAR最大距离 (米)


@dataclass
class VehicleMountPrior:
    """车辆安装位置先验
    
    假设 LiDAR 安装在车顶中心，相机安装在前挡风玻璃/侧面/后方
    坐标系: 车辆坐标系，X-前, Y-左, Z-上
    """
    # LiDAR 安装高度 (相对于地面)
    lidar_height: float = 1.8
    # 相机安装位置 (相对于 LiDAR)
    cam_forward: float = 0.5    # 相机在 LiDAR 前方的距离，负值表示后方
    cam_left: float = 0.0       # 相机在 LiDAR 左侧的距离，负值表示右侧
    cam_up: float = -0.3        # 相机在 LiDAR 上方的距离，负值表示下方
    # 相机朝向 (相对于车辆前方)
    cam_yaw_deg: float = 0.0    # 相机偏航角 (度)，正值向左
    cam_pitch_deg: float = -15.0  # 相机俯仰角 (度)，负值向下


@dataclass
class SearchRange:
    """搜索范围配置"""
    # 平移搜索范围 (米)
    tx_range: List[float] = field(default_factory=lambda: [-0.5, 0.0, 0.5])
    ty_range: List[float] = field(default_factory=lambda: [-0.3, 0.0, 0.3])
    tz_range: List[float] = field(default_factory=lambda: [0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
    # 旋转搜索范围 (度)
    yaw_deg_range: List[float] = field(default_factory=lambda: [-6, -3, 0, 3, 6])
    pitch_deg_range: List[float] = field(default_factory=lambda: [-4, 0, 4])
    roll_deg_range: List[float] = field(default_factory=lambda: [0])
    
    # 搜索策略
    coarse_step: float = 0.5    # 粗搜索步长 (米)
    fine_step: float = 0.1      # 精搜索步长 (米)
    use_multiresolution: bool = True  # 是否使用多分辨率搜索
    
    def to_dict(self) -> Dict:
        """转换为字典格式"""
        return {
            "tx_range": self.tx_range,
            "ty_range": self.ty_range,
            "tz_range": self.tz_range,
            "yaw_deg_range": self.yaw_deg_range,
            "pitch_deg_range": self.pitch_deg_range,
            "roll_deg_range": self.roll_deg_range,
            "coarse_step": self.coarse_step,
            "fine_step": self.fine_step,
            "use_multiresolution": self.use_multiresolution,
        }
    
    @classmethod
    def from_vehicle_prior(cls, prior: VehicleMountPrior, multiplier: float = 1.5) -> "SearchRange":
        """基于车辆安装先验生成搜索范围"""
        # 平移范围: 基于先验位置，扩展 multiplier 倍
        tx_center = prior.cam_forward
        ty_center = prior.cam_left
        tz_center = -prior.cam_up  # 注意符号: cam_up 负值表示相机在下方
        
        base_range = 0.5 * multiplier
        
        return cls(
            tx_range=_generate_range(tx_center, base_range, step=0.2),
            ty_range=_generate_range(ty_center, base_range * 0.6, step=0.15),
            tz_range=_generate_range(max(tz_center, 0.5), 1.5 * multiplier, step=0.5),
            yaw_deg_range=_generate_range(prior.cam_yaw_deg, 10.0 * multiplier, step=3.0),
            pitch_deg_range=_generate_range(prior.cam_pitch_deg, 8.0 * multiplier, step=2.0),
            roll_deg_range=[0.0],
        )


def _generate_range(center: float, half_width: float, step: float) -> List[float]:
    """生成搜索范围列表"""
    n_steps = int(half_width / step)
    values = [center + i * step for i in range(-n_steps, n_steps + 1)]
    # 去重并排序
    return sorted(list(set(round(v, 3) for v in values)))


@dataclass
class InitialGuessResult:
    """初始值评估结果"""
    # 估计的外参 (如果有)
    R: Optional[np.ndarray] = None
    t: Optional[np.ndarray] = None
    # 置信度 (0-1)
    confidence: float = 0.0
    confidence_level: ConfidenceLevel = ConfidenceLevel.VERY_LOW
    # 推荐的搜索范围
    search_range: Optional[SearchRange] = None
    # 诊断信息
    diagnostics: Dict = field(default_factory=dict)
    
    def is_reliable(self, min_confidence: float = 0.5) -> bool:
        """判断结果是否可靠"""
        return self.confidence >= min_confidence
    
    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "R": self.R.tolist() if self.R is not None else None,
            "t": self.t.tolist() if self.t is not None else None,
            "confidence": self.confidence,
            "confidence_level": self.confidence_level.value,
            "search_range": self.search_range.to_dict() if self.search_range else None,
            "diagnostics": self.diagnostics,
        }


@dataclass
class InitialGuessConfig:
    """初始值评估配置"""
    # 传感器FOV
    sensor_fov: SensorFOV = field(default_factory=SensorFOV)
    # 车辆安装先验
    vehicle_prior: VehicleMountPrior = field(default_factory=VehicleMountPrior)
    # 评估参数
    min_confidence: float = 0.5
    feature_match_threshold: float = 0.7
    min_feature_matches: int = 8
    # 搜索范围倍数
    search_range_multiplier: float = 1.5
    # 是否启用深度特征匹配
    enable_deep_matching: bool = True
    # 是否使用安装先验
    use_mount_prior: bool = True


class InitialGuessEstimator:
    """
    初始值评估器
    
    在没有标定初始值的情况下:
    1. 基于传感器FOV估计重叠区域
    2. 基于车辆安装位置先验生成搜索范围
    3. (可选) 深度特征匹配估计粗略位姿
    4. 计算置信度
    """
    
    def __init__(self, config: Optional[InitialGuessConfig] = None):
        self.config = config or InitialGuessConfig()
        self._feature_matcher = None
        
    def estimate(
        self,
        points: np.ndarray,
        image: np.ndarray,
        camera_k: np.ndarray,
        bev_features: Optional[np.ndarray] = None,
        image_features: Optional[np.ndarray] = None,
        lidar_local_features: Optional[np.ndarray] = None,
    ) -> InitialGuessResult:
        """
        评估初始值
        
        Args:
            points: (N, 3) LiDAR点云
            image: (H, W, 3) 相机图像
            camera_k: (3, 3) 相机内参
            bev_features: (D,) 或 (H_bev, W_bev, D) BEV全局/局部特征
            image_features: (M, D) 图像特征
            lidar_local_features: (N_l, D) LiDAR局部特征
            
        Returns:
            InitialGuessResult: 评估结果
        """
        result = InitialGuessResult()
        result.diagnostics["input_points"] = len(points)
        result.diagnostics["image_shape"] = image.shape
        
        # Step 1: 基于FOV估计重叠区域
        overlap_score = self._estimate_fov_overlap()
        result.diagnostics["fov_overlap_score"] = overlap_score
        
        # Step 2: 生成搜索范围
        if self.config.use_mount_prior:
            search_range = SearchRange.from_vehicle_prior(
                self.config.vehicle_prior,
                self.config.search_range_multiplier
            )
        else:
            search_range = SearchRange()
        
        # 根据FOV重叠调整搜索范围
        if overlap_score < 0.3:
            # 低重叠，扩大搜索范围
            search_range = self._expand_search_range(search_range, factor=1.5)
            result.diagnostics["search_range_adjusted"] = "expanded due to low FOV overlap"
        
        result.search_range = search_range
        
        # Step 3: 深度特征匹配 (如果特征可用)
        if self.config.enable_deep_matching and bev_features is not None and image_features is not None:
            match_result = self._deep_feature_matching(
                points, image, camera_k,
                bev_features, image_features, lidar_local_features
            )
            if match_result is not None:
                R, t, confidence = match_result
                result.R = R
                result.t = t
                result.confidence = confidence
                result.diagnostics["deep_matching"] = "success"
            else:
                result.diagnostics["deep_matching"] = "failed"
        
        # Step 4: 计算综合置信度
        if result.confidence == 0.0:
            # 没有深度匹配结果，使用基于先验的置信度
            result.confidence = self._compute_prior_confidence(overlap_score, len(points))
            result.diagnostics["confidence_source"] = "prior"
        else:
            result.diagnostics["confidence_source"] = "deep_matching"
        
        # 确定置信度等级
        result.confidence_level = self._get_confidence_level(result.confidence)
        
        _LOG.info(
            "[InitialGuess] 评估完成: confidence=%.2f level=%s search_tx=%s search_tz=%s",
            result.confidence, result.confidence_level.value,
            result.search_range.tx_range[:3] if result.search_range else None,
            result.search_range.tz_range[:3] if result.search_range else None
        )
        
        return result
    
    def _estimate_fov_overlap(self) -> float:
        """
        估计传感器FOV重叠度
        
        Returns:
            overlap_score: 0-1之间的重叠分数
        """
        fov = self.config.sensor_fov
        
        # 简化计算: 假设相机在LiDAR前方
        # 重叠度主要取决于相机水平FOV与LiDAR前方FOV的重叠
        
        # LiDAR前方FOV (假设180度前方区域有效)
        lidar_front_fov = min(fov.lidar_h_fov_deg / 2, 180.0)
        
        # 相机FOV
        cam_fov = fov.cam_h_fov_deg
        
        # 重叠角
        overlap_angle = min(lidar_front_fov, cam_fov)
        
        # 重叠度: 相对于相机FOV
        overlap_score = overlap_angle / cam_fov if cam_fov > 0 else 0.0
        
        return min(1.0, overlap_score)
    
    def _expand_search_range(self, search_range: SearchRange, factor: float = 1.5) -> SearchRange:
        """扩展搜索范围"""
        def expand_list(lst: List[float], f: float) -> List[float]:
            if not lst:
                return lst
            center = (lst[0] + lst[-1]) / 2
            half_width = (lst[-1] - lst[0]) / 2 * f
            step = (lst[-1] - lst[0]) / max(1, len(lst) - 1)
            return _generate_range(center, half_width, step)
        
        return SearchRange(
            tx_range=expand_list(search_range.tx_range, factor),
            ty_range=expand_list(search_range.ty_range, factor),
            tz_range=expand_list(search_range.tz_range, factor),
            yaw_deg_range=expand_list(search_range.yaw_deg_range, factor),
            pitch_deg_range=expand_list(search_range.pitch_deg_range, factor),
            roll_deg_range=search_range.roll_deg_range,
            coarse_step=search_range.coarse_step * factor,
            fine_step=search_range.fine_step,
            use_multiresolution=search_range.use_multiresolution,
        )
    
    def _deep_feature_matching(
        self,
        points: np.ndarray,
        image: np.ndarray,
        camera_k: np.ndarray,
        bev_features: np.ndarray,
        image_features: np.ndarray,
        lidar_local_features: Optional[np.ndarray] = None,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """
        深度特征匹配估计粗略位姿
        
        Returns:
            (R, t, confidence) 或 None
        """
        try:
            import cv2
        except ImportError:
            _LOG.warning("[InitialGuess] cv2 不可用，跳过深度特征匹配")
            return None
        
        # 使用局部特征 (如果可用)
        if lidar_local_features is not None and len(lidar_local_features) > 0:
            feat_3d = lidar_local_features
        else:
            # 使用全局特征广播
            if bev_features.ndim == 1:
                # 全局特征，无法建立对应关系
                _LOG.debug("[InitialGuess] 仅有全局特征，跳过深度匹配")
                return None
            feat_3d = bev_features
        
        feat_2d = image_features
        
        if len(feat_3d) == 0 or len(feat_2d) == 0:
            return None
        
        # 计算特征相似度
        if _torch_available:
            similarity = self._compute_similarity_torch(feat_3d, feat_2d)
        else:
            similarity = self._compute_similarity_numpy(feat_3d, feat_2d)
        
        # 特征匹配
        matches = self._match_features(similarity)
        
        if len(matches) < self.config.min_feature_matches:
            _LOG.debug("[InitialGuess] 匹配数不足: %d < %d", len(matches), self.config.min_feature_matches)
            return None
        
        # 构建3D-2D对应
        obj_pts = []
        img_pts = []
        
        h, w = image.shape[:2]
        
        for i_3d, i_2d, score in matches:
            if score < self.config.feature_match_threshold:
                continue
            
            # 3D点
            if i_3d < len(points):
                obj_pts.append(points[i_3d])
            else:
                continue
            
            # 2D点: 使用特征位置的中心 (简化)
            # 实际应用中应该有更精确的位置
            cx = w / 2
            cy = h / 2
            img_pts.append([cx, cy])
        
        if len(obj_pts) < self.config.min_feature_matches:
            return None
        
        obj_pts = np.array(obj_pts, dtype=np.float64)
        img_pts = np.array(img_pts, dtype=np.float64)
        
        # PnP求解
        try:
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj_pts, img_pts, camera_k, None,
                reprojectionError=10.0,
                confidence=0.9,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            
            if not ok or inliers is None or len(inliers) < 6:
                return None
            
            if len(obj_pts) == 0:
                return None
            
            R, _ = cv2.Rodrigues(rvec)
            t = tvec.ravel()
            
            # 置信度基于内点率
            confidence = len(inliers) / len(obj_pts)
            
            return R, t, confidence
            
        except Exception as e:
            _LOG.debug("[InitialGuess] PnP失败: %s", e)
            return None
    
    def _compute_similarity_torch(self, feat_3d: np.ndarray, feat_2d: np.ndarray) -> np.ndarray:
        """使用PyTorch计算特征相似度"""
        f3 = torch.from_numpy(feat_3d).float()
        f2 = torch.from_numpy(feat_2d).float()
        
        if f3.ndim == 1:
            f3 = f3.unsqueeze(0)
        if f2.ndim == 1:
            f2 = f2.unsqueeze(0)
        
        f3 = F.normalize(f3, p=2, dim=1)
        f2 = F.normalize(f2, p=2, dim=1)
        
        similarity = torch.mm(f3, f2.T)
        return similarity.cpu().numpy()
    
    def _compute_similarity_numpy(self, feat_3d: np.ndarray, feat_2d: np.ndarray) -> np.ndarray:
        """使用NumPy计算特征相似度"""
        if feat_3d.ndim == 1:
            feat_3d = feat_3d.reshape(1, -1)
        if feat_2d.ndim == 1:
            feat_2d = feat_2d.reshape(1, -1)
        
        f3_norm = feat_3d / (np.linalg.norm(feat_3d, axis=1, keepdims=True) + 1e-8)
        f2_norm = feat_2d / (np.linalg.norm(feat_2d, axis=1, keepdims=True) + 1e-8)
        
        return np.dot(f3_norm, f2_norm.T)
    
    def _match_features(self, similarity: np.ndarray) -> List[Tuple[int, int, float]]:
        """特征匹配"""
        # 双向最近邻
        f3_to_f2 = np.argmax(similarity, axis=1)
        f2_to_f3 = np.argmax(similarity, axis=0)
        
        matches = []
        for i_3d, i_2d in enumerate(f3_to_f2):
            # 交叉验证
            if f2_to_f3[i_2d] == i_3d:
                score = similarity[i_3d, i_2d]
                matches.append((i_3d, i_2d, float(score)))
        
        # 按分数排序
        matches.sort(key=lambda x: x[2], reverse=True)
        return matches
    
    def _compute_prior_confidence(self, overlap_score: float, n_points: int) -> float:
        """
        基于先验计算置信度
        
        置信度 = 0.4 * FOV重叠度 + 0.3 * 点云密度分数 + 0.3 * 安装先验分数
        """
        # 点云密度分数 (归一化到0-1)
        density_score = min(1.0, n_points / 50000.0)
        
        # 安装先验分数 (假设使用标准安装配置)
        prior_score = 0.7  # 中等置信度
        
        confidence = 0.4 * overlap_score + 0.3 * density_score + 0.3 * prior_score
        
        return min(1.0, confidence)
    
    def _get_confidence_level(self, confidence: float) -> ConfidenceLevel:
        """确定置信度等级"""
        if confidence >= 0.8:
            return ConfidenceLevel.HIGH
        elif confidence >= 0.5:
            return ConfidenceLevel.MEDIUM
        elif confidence >= 0.3:
            return ConfidenceLevel.LOW
        else:
            return ConfidenceLevel.VERY_LOW


def estimate_initial_guess(
    points: np.ndarray,
    image: np.ndarray,
    camera_k: np.ndarray,
    config: Optional[InitialGuessConfig] = None,
    bev_features: Optional[np.ndarray] = None,
    image_features: Optional[np.ndarray] = None,
) -> InitialGuessResult:
    """
    便捷函数: 评估初始值
    
    Args:
        points: (N, 3) LiDAR点云
        image: (H, W, 3) 相机图像
        camera_k: (3, 3) 相机内参
        config: 评估配置
        bev_features: BEV特征
        image_features: 图像特征
        
    Returns:
        InitialGuessResult
    """
    estimator = InitialGuessEstimator(config)
    return estimator.estimate(
        points, image, camera_k,
        bev_features, image_features
    )


def get_default_search_range_for_mounting(
    cam_position: str = "front",
    multiplier: float = 1.5,
) -> SearchRange:
    """
    获取基于相机安装位置的默认搜索范围
    
    Args:
        cam_position: 相机位置 ("front", "left", "right", "rear")
        multiplier: 搜索范围倍数
        
    Returns:
        SearchRange
    """
    priors = {
        "front": VehicleMountPrior(
            cam_forward=0.5, cam_left=0.0, cam_up=-0.3,
            cam_yaw_deg=0.0, cam_pitch_deg=-15.0
        ),
        "left": VehicleMountPrior(
            cam_forward=0.0, cam_left=0.8, cam_up=-0.3,
            cam_yaw_deg=90.0, cam_pitch_deg=-15.0
        ),
        "right": VehicleMountPrior(
            cam_forward=0.0, cam_left=-0.8, cam_up=-0.3,
            cam_yaw_deg=-90.0, cam_pitch_deg=-15.0
        ),
        "rear": VehicleMountPrior(
            cam_forward=-0.5, cam_left=0.0, cam_up=-0.3,
            cam_yaw_deg=180.0, cam_pitch_deg=-15.0
        ),
    }
    
    prior = priors.get(cam_position, priors["front"])
    return SearchRange.from_vehicle_prior(prior, multiplier)
