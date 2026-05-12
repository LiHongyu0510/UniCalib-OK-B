"""
自适应搜索模块

提供基于传感器配置和数据特征的自适应搜索策略。

参考研究:
- CalibRefine (2025): 迭代精化 + 注意力机制
- MDPCalib (2024): 运动估计 + 深度点对应

核心功能:
1. 多分辨率搜索 (粗 -> 精)
2. 自适应搜索范围调整
3. 搜索空间剪枝
4. 并行搜索支持
"""

from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Callable
import concurrent.futures

import numpy as np

_LOG = logging.getLogger(__name__)

# 延迟导入
try:
    import torch
    _torch_available = True
except ImportError:
    _torch_available = False


@dataclass
class AdaptiveSearchConfig:
    """自适应搜索配置"""
    
    # 搜索范围
    tx_center: float = 0.0
    ty_center: float = 0.0
    tz_center: float = 1.5
    
    tx_half_width: float = 0.8
    ty_half_width: float = 0.5
    tz_half_width: float = 2.0
    
    # 旋转搜索范围 (度)
    yaw_center_deg: float = 0.0
    pitch_center_deg: float = 0.0
    roll_center_deg: float = 0.0
    
    yaw_half_width_deg: float = 10.0
    pitch_half_width_deg: float = 6.0
    roll_half_width_deg: float = 3.0
    
    # 多分辨率参数
    coarse_step: float = 0.5      # 粗搜索步长
    fine_step: float = 0.1        # 精搜索步长
    use_multiresolution: bool = True
    
    # 旋转步长 (度)
    rotation_coarse_step_deg: float = 3.0
    rotation_fine_step_deg: float = 1.0
    
    # 搜索策略
    max_hypotheses: int = 500     # 最大假设数
    parallel_workers: int = 4     # 并行工作线程数
    early_stop_threshold: float = 0.85  # 早停阈值 (内点率/置信度)
    
    # 约束条件
    min_points_in_view: int = 100    # 最小视野内点数
    min_masks_for_pnp: int = 4      # PnP所需最小mask数
    max_reproj_error_px: float = 10.0  # 最大重投影误差
    
    def get_tx_range(self, coarse: bool = True) -> List[float]:
        """获取平移X搜索范围"""
        step = self.coarse_step if coarse else self.fine_step
        return _generate_range(self.tx_center, self.tx_half_width, step)
    
    def get_ty_range(self, coarse: bool = True) -> List[float]:
        """获取平移Y搜索范围"""
        step = self.coarse_step if coarse else self.fine_step
        return _generate_range(self.ty_center, self.ty_half_width, step)
    
    def get_tz_range(self, coarse: bool = True) -> List[float]:
        """获取平移Z搜索范围"""
        step = self.coarse_step if coarse else self.fine_step
        return _generate_range(self.tz_center, self.tz_half_width, step)
    
    def get_yaw_range(self, coarse: bool = True) -> List[float]:
        """获取偏航角搜索范围 (度)"""
        step = self.rotation_coarse_step_deg if coarse else self.rotation_fine_step_deg
        return _generate_range(self.yaw_center_deg, self.yaw_half_width_deg, step)
    
    def get_pitch_range(self, coarse: bool = True) -> List[float]:
        """获取俯仰角搜索范围 (度)"""
        step = self.rotation_coarse_step_deg if coarse else self.rotation_fine_step_deg
        return _generate_range(self.pitch_center_deg, self.pitch_half_width_deg, step)
    
    def get_roll_range(self, coarse: bool = True) -> List[float]:
        """获取滚转角搜索范围 (度)"""
        step = self.rotation_coarse_step_deg if coarse else self.rotation_fine_step_deg
        return _generate_range(self.roll_center_deg, self.roll_half_width_deg, step)
    
    @classmethod
    def from_initial_guess_result(cls, result: "InitialGuessResult") -> "AdaptiveSearchConfig":
        """从初始值评估结果生成配置"""
        from .initial_guess_estimator import InitialGuessResult
        
        # 使用评估结果中的搜索范围
        sr = result.search_range
        
        return cls(
            tx_center=(sr.tx_range[0] + sr.tx_range[-1]) / 2 if sr.tx_range else 0.0,
            ty_center=(sr.ty_range[0] + sr.ty_range[-1]) / 2 if sr.ty_range else 0.0,
            tz_center=(sr.tz_range[0] + sr.tz_range[-1]) / 2 if sr.tz_range else 1.5,
            
            tx_half_width=(sr.tx_range[-1] - sr.tx_range[0]) / 2 if len(sr.tx_range) > 1 else 0.8,
            ty_half_width=(sr.ty_range[-1] - sr.ty_range[0]) / 2 if len(sr.ty_range) > 1 else 0.5,
            tz_half_width=(sr.tz_range[-1] - sr.tz_range[0]) / 2 if len(sr.tz_range) > 1 else 2.0,
            
            yaw_center_deg=(sr.yaw_deg_range[0] + sr.yaw_deg_range[-1]) / 2 if sr.yaw_deg_range else 0.0,
            pitch_center_deg=(sr.pitch_deg_range[0] + sr.pitch_deg_range[-1]) / 2 if sr.pitch_deg_range else 0.0,
            
            yaw_half_width_deg=(sr.yaw_deg_range[-1] - sr.yaw_deg_range[0]) / 2 if len(sr.yaw_deg_range) > 1 else 10.0,
            pitch_half_width_deg=(sr.pitch_deg_range[-1] - sr.pitch_deg_range[0]) / 2 if len(sr.pitch_deg_range) > 1 else 6.0,
            
            coarse_step=sr.coarse_step,
            fine_step=sr.fine_step,
            use_multiresolution=sr.use_multiresolution,
        )


def _generate_range(center: float, half_width: float, step: float) -> List[float]:
    """生成搜索范围列表"""
    if step <= 0:
        return [center]
    
    n_steps = int(half_width / step)
    values = []
    for i in range(-n_steps, n_steps + 1):
        values.append(center + i * step)
    
    # 去重并排序
    values = sorted(list(set(values)))
    return values


@dataclass
class HypothesisResult:
    """假设检验结果"""
    R: np.ndarray              # 旋转矩阵
    t: np.ndarray              # 平移向量
    score: float               # 评分 (内点率或NCC分数)
    n_inliers: int             # 内点数
    n_points_in_view: int       # 视野内点数
    
    def __lt__(self, other):
        """比较算子，用于排序"""
        return self.score < other.score


    
    def is_better_than(self, other: "HypothesisResult", min_improvement: float = 0.01) -> bool:
        """判断是否显著优于另一个结果"""
        return self.score > other.score + min_improvement


class AdaptiveSearcher:
    """
    自适应搜索器
    
    实现多分辨率搜索和自适应范围调整。
    """
    
    def __init__(self, config: Optional[AdaptiveSearchConfig] = None):
        self.config = config or AdaptiveSearchConfig()
        self._best_result: Optional[HypothesisResult] = None
        self._n_hypotheses_tested = 0
        
    def search(
        self,
        points: np.ndarray,
        masks: np.ndarray,
        camera_k: np.ndarray,
        camera_dist: Optional[np.ndarray] = None,
        evaluate_fn: Callable,
        initial_guess: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """
        执行自适应搜索
        
        Args:
            points: (N, 3) LiDAR点云
            masks: (M, H, W) 分割masks
            camera_k: (3, 3) 相机内参
            camera_dist: 畸变系数
            evaluate_fn: 评估函数, 输入(R, t), 返回(score, n_inliers, n_points_in_view)
            initial_guess: 初始猜测 (R, t)
            
        Returns:
            (R, t, score) 或 None
        """
        import time
        t_start = time.perf_counter()
        
        self._best_result = None
        self._n_hypotheses_tested = 0
        
        # 如果有初始猜测,先评估
        if initial_guess is not None:
            R_init, t_init = initial_guess
            score, n_inliers, n_points = evaluate_fn(R_init, t_init)
            self._best_result = HypothesisResult(R_init, t_init, score, n_inliers, n_points)
            _LOG.info("[AdaptiveSearch] 初始猜测评分: %.4f (inliers=%d)", score, n_inliers)
            
            # 如果初始猜测足够好，早停
            if score >= self.config.early_stop_threshold:
                _LOG.info("[AdaptiveSearch] 初始猜测已满足早停条件,提前返回")
                return R_init, t_init, score
        
        # 多分辨率搜索
        if self.config.use_multiresolution:
            # Stage 1: 粗搜索
            _LOG.info("[AdaptiveSearch] Stage 1: 粗搜索")
            self._search_stage(points, masks, camera_k, camera_dist, evaluate_fn, coarse=True)
            
            if self._best_result is not None and self._best_result.score >= self.config.early_stop_threshold:
                _LOG.info("[AdaptiveSearch] 粗搜索已满足早停条件")
            else:
                # Stage 2: 精搜索 (以最佳结果为中心)
                if self._best_result is not None:
                    self._refine_search_center(self._best_result)
                
                _LOG.info("[AdaptiveSearch] Stage 2: 精搜索")
                self._search_stage(points, masks, camera_k, camera_dist, evaluate_fn, coarse=False)
        else:
            # 单分辨率搜索
            self._search_stage(points, masks, camera_k, camera_dist, evaluate_fn, coarse=True)
        
        t_elapsed = time.perf_counter() - t_start
        
        if self._best_result is not None:
            _LOG.info(
                "[AdaptiveSearch] 搜索完成: 假设数=%d 耗时=%.2fs 最佳评分=%.4f inliers=%d",
                self._n_hypotheses_tested, t_elapsed,
                self._best_result.score, self._best_result.n_inliers
            )
            return self._best_result.R, self._best_result.t, self._best_result.score
        
        _LOG.warning("[AdaptiveSearch] 搜索失败: 无有效假设")
        return None
    
    def _search_stage(
        self,
        points: np.ndarray,
        masks: np.ndarray,
        camera_k: np.ndarray,
        camera_dist: Optional[np.ndarray],
        evaluate_fn: Callable,
        coarse: bool,
    ):
        """执行单阶段搜索"""
        cfg = self.config
        
        # 获取搜索范围
        tx_range = cfg.get_tx_range(coarse)
        ty_range = cfg.get_ty_range(coarse)
        tz_range = cfg.get_tz_range(coarse)
        yaw_range = cfg.get_yaw_range(coarse)
        pitch_range = cfg.get_pitch_range(coarse)
        roll_range = cfg.get_roll_range(coarse)
        
        # 生成所有假设
        hypotheses = []
        for tz in tz_range:
            for tx in tx_range:
                for ty in ty_range:
                    for yaw_deg in yaw_range:
                        for pitch_deg in pitch_range:
                            for roll_deg in roll_range:
                                R = _euler_deg_to_R(yaw_deg, pitch_deg, roll_deg)
                                t = np.array([tx, ty, tz], dtype=np.float64)
                                hypotheses.append((R, t))
        
        # 限制假设数
        if len(hypotheses) > cfg.max_hypotheses:
            # 随机采样
            indices = np.random.choice(len(hypotheses), cfg.max_hypotheses, replace=False)
            hypotheses = [hypotheses[i] for i in indices]
        
        _LOG.debug("[AdaptiveSearch] 阶段'%s'假设数: %d", "粗" if coarse else "精", len(hypotheses))
        
        # 并行评估假设
        self._evaluate_hypotheses_parallel(hypotheses, evaluate_fn)
    
    def _evaluate_hypotheses_parallel(
        self,
        hypotheses: List[Tuple[np.ndarray, np.ndarray]],
        evaluate_fn: Callable,
    ):
        """并行评估假设"""
        n_workers = self.config.parallel_workers
        
        def evaluate_one(hyp):
            R, t = hyp
            try:
                score, n_inliers, n_points = evaluate_fn(R, t)
                return HypothesisResult(R, t, score, n_inliers, n_points)
            except Exception as e:
                _LOG.debug("[AdaptiveSearch] 假设评估异常: %s", e)
                return None
        
        with concurrent.futures.ThreadPoolExecutor(max_workers=n_workers) as executor:
            futures = [executor.submit(evaluate_one, hyp) for hyp in hypotheses]
            
            for future in concurrent.futures.as_completed(futures):
                self._n_hypotheses_tested += 1
                result = future.result()
                
                if result is None:
                    continue
                
                # 更新最佳结果
                if self._best_result is None or result.is_better_than(self._best_result):
                    self._best_result = result
                    
                    # 检查早停
                    if result.score >= self.config.early_stop_threshold:
                        _LOG.info("[AdaptiveSearch] 达到早停阈值: %.4f", result.score)
                        # 取消剩余任务
                        for f in futures:
                            f.cancel()
                        break
    
    def _refine_search_center(self, best: HypothesisResult):
        """以最佳结果为中心精化搜索范围"""
        # 从旋转矩阵提取欧拉角
        yaw, pitch, roll = _R_to_euler_deg(best.R)
        
        # 更新配置中心
        self.config.tx_center = best.t[0]
        self.config.ty_center = best.t[1]
        self.config.tz_center = best.t[2]
        self.config.yaw_center_deg = yaw
        self.config.pitch_center_deg = pitch
        self.config.roll_center_deg = roll
        
        # 缩小搜索范围
        self.config.tx_half_width *= 0.3
        self.config.ty_half_width *= 0.3
        self.config.tz_half_width *= 0.3
        self.config.yaw_half_width_deg *= 0.3
        self.config.pitch_half_width_deg *= 0.3
        self.config.roll_half_width_deg *= 0.3
        
        _LOG.debug(
            "[AdaptiveSearch] 精化搜索中心: t=[%.2f, %.2f, %.2f] euler=[%.1f, %.1f, %.1f]",
            self.config.tx_center, self.config.ty_center, self.config.tz_center,
            self.config.yaw_center_deg, self.config.pitch_center_deg, self.config.roll_center_deg
        )


def _euler_deg_to_R(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """从欧拉角 (度) 构建旋转矩阵 R: 绕 ZYX 外旋， R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    y = np.radians(float(yaw_deg))
    p = np.radians(float(pitch_deg))
    r = np.radians(float(roll_deg))
    cz, sz = np.cos(y), np.sin(y)
    cy, sy = np.cos(p), np.sin(p)
    cx, sx = np.cos(r), np.sin(r)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def _R_to_euler_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """从旋转矩阵提取欧拉角 (度): ZYX 外旋"""
    import math
    
    sy = -R[2, 0]
    cy_squared = R[0, 0]**2 + R[1, 0]**2
    
    if cy_squared < 1e-10:
        # 万向节锁
        yaw = math.atan2(-R[0, 1], R[0, 0])
        pitch = math.asin(sy)
        roll = 0.0
    else:
        cy = math.sqrt(max(0, cy_squared))
        yaw = math.atan2(R[1, 0] / cy, R[0, 0] / cy)
        pitch = math.atan2(sy, cy)
        roll = math.atan2(R[2, 1] / cy, R[2, 2] / cy)
    
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


