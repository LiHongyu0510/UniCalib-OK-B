"""
C3M (Cross-Modal Mask Matching) 模块

基于论文: Online, Target-Free LiDAR-Camera Extrinsic Calibration via Cross-Modal Mask Matching
https://arxiv.org/abs/2404.18083

核心算法: 通过 MobileSAM 分割的 mask，在 LiDAR BEV 特征和图像特征之间建立对应关系，
然后使用 PnP 求解外参。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple

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


@dataclass
class C3MConfig:
    """C3M 匹配配置"""

    # 特征维度
    feature_dim: int = 256
    # 相似度阈值
    similarity_threshold: float = 0.7
    # 最小匹配数
    min_matches: int = 6
    # RANSAC 参数
    ransac_reproj_threshold: float = 8.0
    ransac_confidence: float = 0.95
    # 是否使用交叉验证
    use_cross_check: bool = True
    # 几何约束
    max_depth_diff: float = 2.0  # 相邻点最大深度差 (米)
    # 迭代精化 (CalibRefine 风格，SAM-only 时在网格搜索后再精化)
    use_iterative_refine: bool = False
    iterative_refine_max_iter: int = 3
    iterative_refine_convergence_threshold: float = 0.01


class C3MMatcher:
    """
    Cross-Modal Mask Matching (C3M) 匹配器

    在 LiDAR BEV 特征和图像 mask 特征之间建立对应关系。
    """

    def __init__(self, config: Optional[C3MConfig] = None):
        self.config = config or C3MConfig()

    def match(
        self,
        lidar_features: np.ndarray,
        image_features: np.ndarray,
        lidar_points: np.ndarray,
        image_masks: np.ndarray,
        camera_k: np.ndarray,
        initial_rt: Optional[Tuple[np.ndarray, np.ndarray]] = None,
        camera_dist: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        执行跨模态 mask matching

        Args:
            lidar_features: (N_l, D) LiDAR BEV 特征
            image_features: (N_i, D) 图像 mask 特征
            lidar_points: (N_l, 3) LiDAR 点云坐标
            image_masks: (N_i, H, W) 图像分割 masks
            camera_k: (3, 3) 相机内参矩阵
            initial_rt: 初始 (R, t)，用于投影约束
            camera_dist: 畸变系数，几何验证时与 solvePnP 一致

        Returns:
            obj_pts: (M, 3) 3D 物体点
            img_pts: (M, 2) 2D 图像点
            inlier_mask: (M,) 内点掩码
        """
        cfg = self.config

        n_lidar, n_img = len(lidar_features), len(image_features)
        _LOG.debug("[C3M] match 输入: lidar_feat=%d image_feat=%d lidar_pts=%d masks=%d min_matches=%d",
                   n_lidar, n_img, len(lidar_points), len(image_masks), cfg.min_matches)

        if len(lidar_features) == 0 or len(image_features) == 0:
            _LOG.warning("[C3M] 特征为空 — lidar=%d image=%d，无法匹配", n_lidar, n_img)
            return np.zeros((0, 3)), np.zeros((0, 2)), np.array([])

        # 1. 计算特征相似度
        if _torch_available:
            similarity = self._compute_similarity_torch(lidar_features, image_features)
        else:
            similarity = self._compute_similarity_numpy(lidar_features, image_features)
        _LOG.debug("[C3M] 相似度矩阵 shape=%s", similarity.shape)

        # 2. 特征匹配
        matches = self._feature_matching(similarity)
        _LOG.debug("[C3M] 特征匹配对数=%d (阈值=%.2f 交叉验证=%s)",
                   len(matches), cfg.similarity_threshold, cfg.use_cross_check)

        if len(matches) < cfg.min_matches:
            _LOG.warning("[C3M] 匹配数不足: %d < %d (相似度阈值=%.2f 可尝试降低)", len(matches), cfg.min_matches, cfg.similarity_threshold)
            return np.zeros((0, 3)), np.zeros((0, 2)), np.array([])

        # 3. 构建 3D-2D 对应
        obj_pts, img_pts = self._build_correspondences(
            matches, lidar_points, image_masks
        )

        if len(obj_pts) < cfg.min_matches:
            _LOG.warning("[C3M] 对应点不足: %d < %d (部分 mask 无有效像素)", len(obj_pts), cfg.min_matches)
            return np.zeros((0, 3)), np.zeros((0, 2)), np.array([])

        # 4. 几何验证 (可选)
        if initial_rt is not None:
            obj_pts, img_pts, inlier_mask = self._geometric_verification(
                obj_pts, img_pts, camera_k, initial_rt, camera_dist
            )
        else:
            inlier_mask = np.ones(len(obj_pts), dtype=bool)

        _LOG.info("[C3M] 匹配结果: %d 对应点, %d 内点", len(obj_pts), int(inlier_mask.sum()))

        return obj_pts, img_pts, inlier_mask

    def _compute_similarity_torch(
        self,
        lidar_features: np.ndarray,
        image_features: np.ndarray,
    ) -> np.ndarray:
        """使用 PyTorch 计算余弦相似度"""
        l_feat = torch.from_numpy(lidar_features).float()
        i_feat = torch.from_numpy(image_features).float()

        # L2 归一化
        l_feat = F.normalize(l_feat, p=2, dim=1)
        i_feat = F.normalize(i_feat, p=2, dim=1)

        # 余弦相似度
        similarity = torch.mm(l_feat, i_feat.T)

        return similarity.cpu().numpy()

    def _compute_similarity_numpy(
        self,
        lidar_features: np.ndarray,
        image_features: np.ndarray,
    ) -> np.ndarray:
        """使用 NumPy 计算余弦相似度"""
        # L2 归一化
        l_norm = np.linalg.norm(lidar_features, axis=1, keepdims=True) + 1e-8
        i_norm = np.linalg.norm(image_features, axis=1, keepdims=True) + 1e-8

        l_feat = lidar_features / l_norm
        i_feat = image_features / i_norm

        # 余弦相似度
        similarity = np.dot(l_feat, i_feat.T)

        return similarity

    def _feature_matching(
        self,
        similarity: np.ndarray,
    ) -> List[Tuple[int, int, float]]:
        """
        特征匹配 (双向最近邻 + 交叉验证)

        Returns:
            matches: [(lidar_idx, image_idx, score), ...]
        """
        cfg = self.config

        # LiDAR -> Image
        lidar_to_img = np.argmax(similarity, axis=1)
        lidar_scores = np.max(similarity, axis=1)

        # Image -> LiDAR
        img_to_lidar = np.argmax(similarity, axis=0)

        matches = []
        for lidar_idx, img_idx in enumerate(lidar_to_img):
            # 交叉验证
            if cfg.use_cross_check:
                if img_to_lidar[img_idx] != lidar_idx:
                    continue

            score = lidar_scores[lidar_idx]

            # 阈值过滤
            if score >= cfg.similarity_threshold:
                matches.append((lidar_idx, img_idx, score))

        # 按分数排序
        matches.sort(key=lambda x: x[2], reverse=True)

        return matches

    def _build_correspondences(
        self,
        matches: List[Tuple[int, int, float]],
        lidar_points: np.ndarray,
        image_masks: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        构建 3D-2D 对应关系

        对于每个匹配对，找到 LiDAR 点和图像 mask 内的像素点。
        """
        obj_pts_list = []
        img_pts_list = []

        img_h, img_w = image_masks[0].shape

        for lidar_idx, img_idx, score in matches:
            # LiDAR 点
            pts_3d = lidar_points[lidar_idx]

            # 图像 mask 的质心
            mask = image_masks[img_idx]
            y_coords, x_coords = np.where(mask)

            if len(x_coords) == 0:
                continue

            # 使用质心作为 2D 点
            cx = x_coords.mean()
            cy = y_coords.mean()

            obj_pts_list.append(pts_3d)
            img_pts_list.append([cx, cy])

        if not obj_pts_list:
            return np.zeros((0, 3)), np.zeros((0, 2))

        return np.array(obj_pts_list), np.array(img_pts_list)

    def _geometric_verification(
        self,
        obj_pts: np.ndarray,
        img_pts: np.ndarray,
        camera_k: np.ndarray,
        initial_rt: Tuple[np.ndarray, np.ndarray],
        camera_dist: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        几何验证：使用 cv2.projectPoints（与 solvePnPRansac 一致，含畸变）计算重投影误差。
        """
        try:
            import cv2
        except ImportError:
            # 无 cv2 时退化为针孔投影
            R, t = initial_rt
            pts_cam = (R @ obj_pts.T).T + t.ravel()
            depth_mask = pts_cam[:, 2] > 0
            pts_2d = (camera_k @ pts_cam.T).T
            pts_2d = pts_2d[:, :2] / (pts_2d[:, 2:3] + 1e-10)
            reproj_err = np.linalg.norm(pts_2d - img_pts, axis=1)
            inlier_mask = (reproj_err < self.config.ransac_reproj_threshold) & depth_mask
            return obj_pts, img_pts, inlier_mask

        R, t = initial_rt
        rvec, _ = cv2.Rodrigues(R.astype(np.float64))
        tvec = np.asarray(t, dtype=np.float64).ravel()
        if camera_dist is None:
            camera_dist = np.zeros(5, dtype=np.float64)
        pts_cam = (R @ obj_pts.T).T + tvec
        depth_mask = pts_cam[:, 2] > 0
        proj_2d, _ = cv2.projectPoints(
            obj_pts.astype(np.float64),
            rvec,
            tvec,
            camera_k.astype(np.float64),
            camera_dist.astype(np.float64),
        )
        pts_2d = proj_2d.reshape(-1, 2)
        reproj_err = np.linalg.norm(pts_2d - img_pts, axis=1)
        inlier_mask = (reproj_err < self.config.ransac_reproj_threshold) & depth_mask
        return obj_pts, img_pts, inlier_mask


def solve_pnp_ransac(
    obj_pts: np.ndarray,
    img_pts: np.ndarray,
    camera_k: np.ndarray,
    camera_dist: Optional[np.ndarray] = None,
    reproj_threshold: float = 8.0,
    confidence: float = 0.95,
    min_inliers: int = 6,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    使用 PnP + RANSAC 求解外参

    Args:
        obj_pts: (N, 3) 3D 物体点
        img_pts: (N, 2) 2D 图像点
        camera_k: (3, 3) 相机内参
        camera_dist: 畸变系数
        reproj_threshold: 重投影误差阈值
        confidence: 置信度
        min_inliers: 最小内点数

    Returns:
        (R, t, inlier_mask) 或 None
    """
    try:
        import cv2
    except ImportError:
        _LOG.error("[PnP] cv2 不可用")
        return None

    if len(obj_pts) < min_inliers:
        _LOG.warning("[PnP] 点数不足: %d < %d", len(obj_pts), min_inliers)
        return None

    obj_pts = obj_pts.astype(np.float64)
    img_pts = img_pts.astype(np.float64)

    if camera_dist is None:
        camera_dist = np.zeros(5, dtype=np.float64)

    try:
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts,
            img_pts,
            camera_k,
            camera_dist,
            reprojectionError=reproj_threshold,
            confidence=confidence,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok or inliers is None:
            _LOG.warning("[C3M-PnP] solvePnPRansac 失败 ok=%s inliers=%s", ok, inliers)
            return None

        if len(inliers) < min_inliers:
            _LOG.warning("[C3M-PnP] 内点不足: %d < %d (reproj_thresh=%.1f)", len(inliers), min_inliers, reproj_threshold)
            return None

        # 转换为旋转矩阵
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.ravel()

        # 内点掩码
        inlier_mask = np.zeros(len(obj_pts), dtype=bool)
        inlier_mask[inliers.flatten()] = True

        _LOG.info("[C3M-PnP] 成功 内点=%d/%d t=[%.4f, %.4f, %.4f]", len(inliers), len(obj_pts), t[0], t[1], t[2])

        return R.astype(np.float64), t.astype(np.float64), inlier_mask

    except Exception as e:
        import traceback
        _LOG.error("[C3M-PnP] 异常: %s", e)
        _LOG.debug("[C3M-PnP] traceback:\n%s", traceback.format_exc())
        return None


def match_and_solve(
    lidar_features: np.ndarray,
    image_features: np.ndarray,
    lidar_points: np.ndarray,
    image_masks: np.ndarray,
    camera_k: np.ndarray,
    config: Optional[C3MConfig] = None,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    便捷函数: 执行匹配 + PnP 求解

    Returns:
        (R, t) 或 None
    """
    config = config or C3MConfig()
    matcher = C3MMatcher(config)

    # 匹配
    obj_pts, img_pts, _ = matcher.match(
        lidar_features, image_features, lidar_points, image_masks, camera_k
    )

    if len(obj_pts) < config.min_matches:
        return None

    # PnP 求解
    result = solve_pnp_ransac(
        obj_pts, img_pts, camera_k,
        reproj_threshold=config.ransac_reproj_threshold,
        confidence=config.ransac_confidence,
        min_inliers=config.min_matches,
    )

    if result is None:
        return None

    R, t, _ = result
    return R, t
