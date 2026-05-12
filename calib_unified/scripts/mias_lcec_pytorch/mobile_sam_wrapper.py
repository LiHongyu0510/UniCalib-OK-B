"""
MobileSAM 分割封装

基于 MobileSAM (Faster Segment Anything) 进行图像分割，
用于 MIAS-LCEC 的图像分支特征提取。

参考: https://github.com/ChaoningZhang/MobileSAM
"""

from __future__ import annotations

import logging
import os
from typing import List, Optional, Tuple, Union

import numpy as np

_LOG = logging.getLogger(__name__)

# 延迟导入
_torch_available = False
_sam_available = False

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    _torch_available = True
except ImportError:
    pass

# 尝试导入 MobileSAM
try:
    from mobile_sam import SamPredictor, sam_model_registry
    _sam_available = True
except ImportError:
    try:
        # 尝试从 MIAS-LCEC 目录导入
        pass
    except ImportError:
        pass


class MobileSAMWrapper:
    """
    MobileSAM 封装器

    提供图像分割和特征提取功能，用于 C3M mask matching。
    """

    def __init__(
        self,
        checkpoint_path: Optional[str] = None,
        device: str = "cuda",
        model_type: str = "vit_t",
    ):
        """
        Args:
            checkpoint_path: MobileSAM 权重路径。如果为 None，会尝试自动查找。
            device: 计算设备
            model_type: 模型类型，默认 vit_t (tiny)
        """
        if not _torch_available:
            raise ImportError("torch 不可用，请安装: pip install torch")

        self.device = device
        self.model_type = model_type
        self.model = None
        self.predictor = None

        # 自动查找 checkpoint
        if checkpoint_path is None:
            checkpoint_path = self._find_checkpoint()

        if checkpoint_path is not None and os.path.isfile(checkpoint_path):
            self._load_model(checkpoint_path)
        else:
            _LOG.warning(
                "[MobileSAM] 未找到权重文件，分割功能不可用。"
                "请下载权重或设置 MOBILESAM_CHECKPOINT 环境变量。"
            )

    def _find_checkpoint(self) -> Optional[str]:
        """自动查找 MobileSAM 权重"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            # 环境变量
            os.environ.get("MOBILESAM_CHECKPOINT", ""),
            # 本仓库 scripts/weights（由 download_mobile_sam.sh 下载）
            os.path.join(script_dir, "..", "weights", "mobile_sam.pt"),
            # MIAS-LCEC/model/（与 pretrained_overlap_transformer.pth.tar 同目录）
            "/root/calib_ws/MIAS-LCEC/model/mobile_sam.pt",
            os.path.expanduser("~/MIAS-LCEC/model/mobile_sam.pt"),
            # 常见位置
            "/root/calib_ws/MIAS-LCEC/bin/MobileSAM/weights/mobile_sam.pt",
            "/root/calib_ws/MIAS-LCEC/bin/weights/mobile_sam.pt",
            os.path.expanduser("~/MIAS-LCEC/bin/MobileSAM/weights/mobile_sam.pt"),
        ]

        for path in candidates:
            if path and os.path.isfile(path):
                _LOG.info("[MobileSAM] 找到权重: %s", path)
                return path
        _LOG.debug("[MobileSAM] 未找到权重，已尝试: %s", [p for p in candidates if p])

        return None

    def _load_model(self, checkpoint_path: str):
        """加载 MobileSAM 模型"""
        try:
            if _sam_available:
                self.model = sam_model_registry[self.model_type](checkpoint=checkpoint_path)
                self.model.to(self.device)
                self.predictor = SamPredictor(self.model)
                _LOG.info("[MobileSAM] 成功加载模型: %s", checkpoint_path)
            else:
                _LOG.warning(
                    "[MobileSAM] mobile_sam 包未安装，尝试使用简化实现。"
                    "建议安装: pip install git+https://github.com/ChaoningZhang/MobileSAM.git"
                )
        except Exception as e:
            import traceback
            _LOG.error("[MobileSAM] 加载模型失败 path=%s: %s", checkpoint_path, e)
            _LOG.debug("[MobileSAM] traceback:\n%s", traceback.format_exc())
            self.model = None
            self.predictor = None

    def is_available(self) -> bool:
        """检查模型是否可用"""
        return self.predictor is not None

    def segment(
        self,
        image: np.ndarray,
        points_per_side: int = 16,
        pred_iou_thresh: float = 0.88,
        stability_score_thresh: float = 0.95,
        min_mask_region_area: int = 100,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        对图像进行自动分割

        Args:
            image: (H, W, 3) BGR 或 RGB 图像
            points_per_side: 每边采样点数
            pred_iou_thresh: IoU 阈值
            stability_score_thresh: 稳定性分数阈值
            min_mask_region_area: 最小 mask 区域面积

        Returns:
            masks: (N, H, W) 分割 masks
            scores: (N,) 每个mask的分数
        """
        if not self.is_available():
            _LOG.warning("[MobileSAM] 模型不可用，返回空分割")
            return np.zeros((0, image.shape[0], image.shape[1]), dtype=bool), np.array([])

        # 转换为 RGB
        if image.shape[2] == 3:
            # 假设是 BGR，转换为 RGB
            image_rgb = image[:, :, ::-1].copy()
        else:
            image_rgb = image

        self.predictor.set_image(image_rgb)

        # 自动生成网格点
        h, w = image.shape[:2]
        points = self._generate_grid_points(h, w, points_per_side)

        # 对每个点进行分割
        all_masks = []
        all_scores = []

        for point in points:
            input_point = np.array([point])
            input_label = np.array([1])  # 前景

            masks, scores, _ = self.predictor.predict(
                point_coords=input_point,
                point_labels=input_label,
                multimask_output=True,
            )

            # 选择最佳 mask
            best_idx = scores.argmax()
            mask = masks[best_idx]
            score = scores[best_idx]

            # 过滤
            if score >= stability_score_thresh and mask.sum() >= min_mask_region_area:
                all_masks.append(mask)
                all_scores.append(score)

        if not all_masks:
            return np.zeros((0, h, w), dtype=bool), np.array([])

        # NMS: 去除重叠的 mask
        masks, scores = self._nms_masks(
            np.stack(all_masks),
            np.array(all_scores),
            iou_threshold=0.8,
        )

        _LOG.info("[MobileSAM] 分割完成: %d 个 mask (points_per_side=%d 过滤后)", len(masks), points_per_side)

        return masks, scores

    def _generate_grid_points(
        self,
        height: int,
        width: int,
        points_per_side: int,
    ) -> List[Tuple[int, int]]:
        """生成网格采样点"""
        points = []
        for i in range(points_per_side):
            for j in range(points_per_side):
                x = int(width * (j + 0.5) / points_per_side)
                y = int(height * (i + 0.5) / points_per_side)
                points.append((x, y))
        return points

    def _nms_masks(
        self,
        masks: np.ndarray,
        scores: np.ndarray,
        iou_threshold: float = 0.8,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """对 masks 进行 NMS"""
        if len(masks) == 0:
            return masks, scores

        # 按分数排序
        order = scores.argsort()[::-1]
        masks = masks[order]
        scores = scores[order]

        keep = []
        for i in range(len(masks)):
            should_keep = True
            for j in keep:
                iou = self._compute_mask_iou(masks[i], masks[j])
                if iou > iou_threshold:
                    should_keep = False
                    break
            if should_keep:
                keep.append(i)

        return masks[keep], scores[keep]

    def _compute_mask_iou(self, mask1: np.ndarray, mask2: np.ndarray) -> float:
        """计算两个 mask 的 IoU"""
        intersection = np.logical_and(mask1, mask2).sum()
        union = np.logical_or(mask1, mask2).sum()
        return intersection / (union + 1e-8)

    def get_mask_features(
        self,
        image: np.ndarray,
        masks: np.ndarray,
    ) -> np.ndarray:
        """
        提取每个 mask 的特征向量

        Args:
            image: (H, W, 3) BGR 图像
            masks: (N, H, W) 分割 masks

        Returns:
            features: (N, D) 特征向量
        """
        if not _torch_available:
            raise ImportError("torch 不可用")

        if len(masks) == 0:
            return np.zeros((0, 256), dtype=np.float32)

        # 获取图像特征
        image_tensor = torch.from_numpy(image).float().permute(2, 0, 1).unsqueeze(0)
        image_tensor = image_tensor.to(self.device) / 255.0

        # 使用 SAM 的图像编码器
        if self.predictor is not None and hasattr(self.predictor, 'features'):
            # 获取已编码的图像特征
            img_features = self.predictor.features  # (1, C, H', W')
        else:
            # 简化实现: 使用颜色直方图 + 纹理
            img_features = None

        features = []
        for mask in masks:
            feat = self._extract_single_mask_feature(image, mask, img_features)
            features.append(feat)

        return np.stack(features, axis=0)

    def _extract_single_mask_feature(
        self,
        image: np.ndarray,
        mask: np.ndarray,
        img_features: Optional[torch.Tensor] = None,
    ) -> np.ndarray:
        """提取单个 mask 的特征"""
        # 1. 颜色特征 (HSV 直方图)
        masked_pixels = image[mask]
        if len(masked_pixels) == 0:
            return np.zeros(256, dtype=np.float32)

        # HSV 直方图
        try:
            import cv2
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            hsv_masked = hsv[mask]
            h_hist = np.bincount(hsv_masked[:, 0] // 8, minlength=32).astype(np.float32)
            s_hist = np.bincount(hsv_masked[:, 1] // 16, minlength=16).astype(np.float32)
            v_hist = np.bincount(hsv_masked[:, 2] // 16, minlength=16).astype(np.float32)
            color_feat = np.concatenate([h_hist, s_hist, v_hist])
        except Exception:
            color_feat = np.zeros(64, dtype=np.float32)

        # 2. 几何特征
        y_coords, x_coords = np.where(mask)
        if len(x_coords) > 0:
            centroid = np.array([x_coords.mean(), y_coords.mean()])
            bbox = np.array([
                x_coords.min(), y_coords.min(),
                x_coords.max(), y_coords.max(),
            ])
            area = len(x_coords)
            aspect_ratio = (bbox[2] - bbox[0]) / (bbox[3] - bbox[1] + 1e-8)
            geom_feat = np.concatenate([
                centroid / np.array(image.shape[:2][::-1]),  # 归一化中心
                (bbox[2:] - bbox[:2]) / np.array(image.shape[:2]),  # 归一化尺寸
                [area / (image.shape[0] * image.shape[1])],  # 面积比例
                [aspect_ratio],
            ])
        else:
            geom_feat = np.zeros(7, dtype=np.float32)

        # 3. 如果有深度特征，使用它们
        deep_feat = np.zeros(185, dtype=np.float32)  # 补齐到 256
        if img_features is not None and _torch_available:
            try:
                # 下采样 mask 到特征图尺寸
                feat_h, feat_w = img_features.shape[2:]
                mask_small = cv2.resize(
                    mask.astype(np.float32),
                    (feat_w, feat_h),
                    interpolation=cv2.INTER_AREA,
                )
                mask_tensor = torch.from_numpy(mask_small).unsqueeze(0).unsqueeze(0)
                mask_tensor = mask_tensor.to(self.device)

                # 加权平均池化
                masked_feat = (img_features * mask_tensor).sum(dim=(2, 3))
                mask_sum = mask_tensor.sum() + 1e-8
                masked_feat = masked_feat / mask_sum

                deep_feat = masked_feat.cpu().numpy().flatten()[:185]
            except Exception as e:
                _LOG.debug("[MobileSAM] 深度特征提取失败: %s", e)

        return np.concatenate([color_feat, geom_feat, deep_feat])


class SimplifiedMobileSAM:
    """
    简化版 MobileSAM 实现

    当 mobile_sam 包不可用时，使用 OpenCV 边缘检测 + 超像素作为替代。
    """

    def __init__(self, device: str = "cuda"):
        self.device = device
        _LOG.info("[SimplifiedMobileSAM] 使用简化实现 (OpenCV 超像素)")

    def is_available(self) -> bool:
        return True

    def segment(
        self,
        image: np.ndarray,
        n_segments: int = 50,
        compactness: float = 10.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        使用 SLIC 超像素进行分割

        Args:
            image: (H, W, 3) BGR 图像
            n_segments: 分割区域数
            compactness: 紧凑度

        Returns:
            masks: (N, H, W) 分割 masks
            scores: (N,) 分数 (均为 1.0)
        """
        try:
            import cv2
            from skimage.segmentation import slic
            from skimage.color import rgb2lab
        except ImportError:
            _LOG.warning("[SimplifiedMobileSAM] skimage 不可用")
            return np.zeros((0, image.shape[0], image.shape[1]), dtype=bool), np.array([])

        # 转换为 RGB
        if image.shape[2] == 3:
            image_rgb = image[:, :, ::-1]
        else:
            image_rgb = image

        # SLIC 超像素
        segments = slic(
            image_rgb,
            n_segments=n_segments,
            compactness=compactness,
            sigma=1,
            start_label=1,
        )

        # 转换为 masks
        unique_labels = np.unique(segments)
        masks = []
        scores = []

        for label in unique_labels:
            mask = segments == label
            if mask.sum() > 100:  # 过滤小区域
                masks.append(mask)
                scores.append(1.0)

        if not masks:
            return np.zeros((0, image.shape[0], image.shape[1]), dtype=bool), np.array([])

        return np.stack(masks), np.array(scores)

    def get_mask_features(
        self,
        image: np.ndarray,
        masks: np.ndarray,
    ) -> np.ndarray:
        """提取 mask 特征"""
        if len(masks) == 0:
            return np.zeros((0, 256), dtype=np.float32)

        features = []
        for mask in masks:
            # 颜色特征
            masked_pixels = image[mask]
            if len(masked_pixels) == 0:
                features.append(np.zeros(256, dtype=np.float32))
                continue

            # 均值和标准差
            mean_color = masked_pixels.mean(axis=0)
            std_color = masked_pixels.std(axis=0)

            # 几何特征
            y_coords, x_coords = np.where(mask)
            centroid = np.array([x_coords.mean(), y_coords.mean()])
            area = len(x_coords)

            feat = np.concatenate([
                mean_color, std_color,  # 6
                centroid,  # 2
                [area],  # 1
                np.zeros(247),  # padding to 256
            ]).astype(np.float32)

            features.append(feat)

        return np.stack(features)


def create_sam_wrapper(
    checkpoint_path: Optional[str] = None,
    device: str = "cuda",
    use_fallback: bool = True,
) -> Union[MobileSAMWrapper, SimplifiedMobileSAM]:
    """
    创建 SAM 分割器

    Args:
        checkpoint_path: 权重路径
        device: 计算设备
        use_fallback: 如果 MobileSAM 不可用，是否使用简化实现

    Returns:
        SAM 分割器实例
    """
    try:
        wrapper = MobileSAMWrapper(checkpoint_path, device)
        if wrapper.is_available():
            return wrapper
    except Exception as e:
        _LOG.warning("[MobileSAM] 初始化失败: %s", e)

    if use_fallback:
        _LOG.info("[SAM] 使用简化实现")
        return SimplifiedMobileSAM(device)

    raise RuntimeError("MobileSAM 不可用且禁用了简化实现")
