"""
Overlap Transformer 网络定义

基于 OverlapTransformer: An Efficient and Yaw-Angle-Invariant Transformer Network
for LiDAR-Based Place Recognition (RAL/IROS 2022)

参考: https://github.com/haomo-ai/OverlapTransformer

用于从 BEV 图像提取全局描述符，支持 LiDAR-Camera 跨模态匹配。
"""

from __future__ import annotations

import logging
import math
from typing import Optional, Tuple, Union

import numpy as np

_LOG = logging.getLogger(__name__)

# 延迟导入 torch，允许在没有 torch 的环境下导入模块
_torch_available = False
try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    _torch_available = True
except ImportError:
    _LOG.debug("[OverlapTransformer] torch 不可用，模块将仅提供占位实现")


class ConvBlock(nn.Module if _torch_available else object):
    """
    卷积块: Conv2d + BatchNorm + ReLU
    kernel_size / padding 可为 int 或 (h, w)，以兼容官方 checkpoint 的 (5,1) 卷积。
    """

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel_size: Union[int, Tuple[int, int]] = 3,
        stride: Union[int, Tuple[int, int]] = 1,
        padding: Union[int, Tuple[int, int]] = 1,
    ):
        if not _torch_available:
            raise ImportError("torch 不可用")
        super().__init__()
        self.conv = nn.Conv2d(
            in_channels, out_channels,
            kernel_size=kernel_size,
            stride=stride,
            padding=padding,
            bias=False,
        )
        self.bn = nn.BatchNorm2d(out_channels)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.relu(self.bn(self.conv(x)))


class FeatureExtractor(nn.Module if _torch_available else object):
    """
    基于 ResNet 风格的特征提取器

    从 BEV 图像提取多尺度特征。

    use_legacy_conv1=True 时与官方 pretrained_overlap_transformer.pth.tar 一致：
    第一层为单 Conv2d(1, 16, kernel_size=(5,1))，后续 layer 通道为 16/32/64。

    use_mias_official=True 时与 MIAS-LCEC 官方 checkpoint 完全一致：
    conv1 1→16 (5,1)；layer1 16→32 (3,1)、32→64 (3,1)；layer2 64→64 (3,1)、64→128 (2,1)；
    layer3 128→128 (1,1)×2；out_channels=128。
    """

    def __init__(
        self,
        in_channels: int = 1,
        base_channels: int = 32,
        use_legacy_conv1: bool = False,
        use_mias_official: bool = False,
    ):
        if not _torch_available:
            raise ImportError("torch 不可用")
        super().__init__()

        if use_mias_official:
            # MIAS-LCEC 官方 checkpoint 精确结构（conv2~bn7 对应 layer1.0~layer3.1）
            self.conv1 = nn.Sequential(
                ConvBlock(in_channels, 16, kernel_size=(5, 1), stride=1, padding=(2, 0)),
            )
            self.pool1 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
            self.layer1 = nn.Sequential(
                ConvBlock(16, 32, kernel_size=(3, 1), stride=1, padding=(1, 0)),
                ConvBlock(32, 64, kernel_size=(3, 1), stride=1, padding=(1, 0)),
            )
            self.layer2 = nn.Sequential(
                ConvBlock(64, 64, kernel_size=(3, 1), stride=2, padding=(1, 0)),
                ConvBlock(64, 128, kernel_size=(2, 1), stride=1, padding=(0, 0)),
            )
            self.layer3 = nn.Sequential(
                ConvBlock(128, 128, kernel_size=1, stride=1, padding=0),
                ConvBlock(128, 128, kernel_size=1, stride=1, padding=0),
            )
            self.out_channels = 128
        elif use_legacy_conv1:
            # 官方 checkpoint: conv1 为单层 [16, 1, 5, 1]，即 Conv2d(1, 16, (5,1))
            base_channels = 16
            self.conv1 = nn.Sequential(
                ConvBlock(
                    in_channels, 16,
                    kernel_size=(5, 1), stride=1, padding=(2, 0),
                ),
            )
            self.pool1 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
            self.layer1 = self._make_layer(16, 16, num_blocks=2, stride=1)
            self.layer2 = self._make_layer(16, 32, num_blocks=2, stride=2)
            self.layer3 = self._make_layer(32, 64, num_blocks=2, stride=2)
            self.out_channels = 64
        else:
            # 默认: 7x7 + 3x3 双块，base_channels 倍率
            self.conv1 = nn.Sequential(
                ConvBlock(in_channels, base_channels, kernel_size=7, stride=2, padding=3),
                ConvBlock(base_channels, base_channels, kernel_size=3, stride=1, padding=1),
            )
            self.pool1 = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
            self.layer1 = self._make_layer(base_channels, base_channels, num_blocks=2, stride=1)
            self.layer2 = self._make_layer(base_channels, base_channels * 2, num_blocks=2, stride=2)
            self.layer3 = self._make_layer(base_channels * 2, base_channels * 4, num_blocks=2, stride=2)
            self.out_channels = base_channels * 4

    def _make_layer(
        self,
        in_channels: int,
        out_channels: int,
        num_blocks: int,
        stride: int = 1,
    ) -> nn.Sequential:
        layers = []

        # 第一个块可能需要下采样
        layers.append(ConvBlock(in_channels, out_channels, stride=stride))
        for _ in range(1, num_blocks):
            layers.append(ConvBlock(out_channels, out_channels))

        return nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, C, H, W) BEV 图像

        Returns:
            feat: (B, C', H', W') 特征图
        """
        x = self.conv1(x)
        x = self.pool1(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        return x


class TransformerBlock(nn.Module if _torch_available else object):
    """
    Transformer 编码器块

    使用标准 Transformer 结构处理空间特征
    """

    def __init__(
        self,
        embed_dim: int,
        num_heads: int = 4,
        ff_dim: int = 512,
        dropout: float = 0.1,
    ):
        if not _torch_available:
            raise ImportError("torch 不可用")
        super().__init__()

        self.norm1 = nn.LayerNorm(embed_dim)
        self.attn = nn.MultiheadAttention(embed_dim, num_heads, dropout=dropout, batch_first=True)
        self.norm2 = nn.LayerNorm(embed_dim)

        self.ff = nn.Sequential(
            nn.Linear(embed_dim, ff_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(ff_dim, embed_dim),
            nn.Dropout(dropout),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Self-attention with residual
        x_norm = self.norm1(x)
        attn_out, _ = self.attn(x_norm, x_norm, x_norm)
        x = x + attn_out

        # Feed-forward with residual
        x = x + self.ff(self.norm2(x))

        return x


class OverlapTransformer(nn.Module if _torch_available else object):
    """
    Overlap Transformer 网络

    从 BEV 图像提取全局描述符，用于 LiDAR place recognition 和跨模态匹配。

    结构:
    1. 特征提取器: 从 BEV 图像提取空间特征
    2. 位置编码: 添加位置信息
    3. Transformer 编码器: 建模全局依赖
    4. 池化 + 全连接: 生成全局描述符
    """

    def __init__(
        self,
        in_channels: int = 1,
        feature_dim: int = 256,
        base_channels: int = 32,
        num_heads: int = 4,
        num_layers: int = 2,
        dropout: float = 0.1,
        input_size: Tuple[int, int] = (1024, 1024),
        use_legacy_feature_extractor: bool = False,
        use_mias_official_feature_extractor: bool = False,
    ):
        """
        Args:
            in_channels: 输入通道数 (BEV 特征通道)
            feature_dim: 输出特征维度
            base_channels: 基础通道数（use_legacy/use_mias_official 时被忽略）
            num_heads: Transformer 注意力头数
            num_layers: Transformer 层数
            dropout: Dropout 比率
            input_size: 输入 BEV 图像尺寸 (H, W)
            use_legacy_feature_extractor: 若 True，conv1 单层 16 通道 (5,1)，layer 16/32/64
            use_mias_official_feature_extractor: 若 True，与 MIAS-LCEC 官方 checkpoint 完全一致（16→32→64→128）
        """
        if not _torch_available:
            raise ImportError("torch 不可用")
        super().__init__()

        self.feature_dim = feature_dim
        self.input_size = input_size

        # 1. 特征提取（legacy / mias_official 与对应 checkpoint 结构一致）
        self.feature_extractor = FeatureExtractor(
            in_channels,
            base_channels,
            use_legacy_conv1=use_legacy_feature_extractor or use_mias_official_feature_extractor,
            use_mias_official=use_mias_official_feature_extractor,
        )
        feat_channels = self.feature_extractor.out_channels

        # 计算特征图尺寸 (与 backbone 下采样一致)
        # MIAS 官方: pool1 stride=2, layer2.0 stride=2 → 共 4x 下采样 → feat = input/4
        # 其他: conv1+pool1+layer2+layer3 共 16x 下采样 → feat = input/16
        if use_mias_official_feature_extractor:
            self.feat_h = input_size[0] // 4
            self.feat_w = input_size[1] // 4
        else:
            self.feat_h = input_size[0] // 16
            self.feat_w = input_size[1] // 16

        # 2. 位置编码
        self.pos_embed = nn.Parameter(
            torch.zeros(1, self.feat_h * self.feat_w, feat_channels)
        )
        self._init_pos_embed()

        # 3. Transformer 编码器
        self.transformer = nn.Sequential(
            *[TransformerBlock(feat_channels, num_heads, feat_channels * 4, dropout)
              for _ in range(num_layers)]
        )

        # 4. 输出层
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.fc = nn.Linear(feat_channels, feature_dim)

        # 5. 可选: 用于细粒度匹配的局部特征
        self.local_fc = nn.Linear(feat_channels, feature_dim)

    def _init_pos_embed(self):
        """初始化位置编码 (正弦)"""
        h, w = self.feat_h, self.feat_w
        embed_dim = self.pos_embed.shape[-1]

        pos = torch.zeros(h * w, embed_dim)
        for i in range(h):
            for j in range(w):
                idx = i * w + j
                for k in range(embed_dim // 2):
                    div_term = math.exp(k * math.log(10000.0) / (embed_dim // 2))
                    pos[idx, 2 * k] = math.sin(i / div_term)
                    pos[idx, 2 * k + 1] = math.cos(j / div_term)

        self.pos_embed.data = pos.unsqueeze(0)

    def forward(
        self,
        bev_image: torch.Tensor,
        return_local: bool = False,
    ) -> torch.Tensor:
        """
        前向传播

        Args:
            bev_image: (B, C, H, W) BEV 图像
            return_local: 是否返回局部特征 (用于细粒度匹配)

        Returns:
            global_feat: (B, D) 全局描述符
            local_feat: (B, N, D) 局部描述符 (可选)
        """
        B = bev_image.shape[0]

        # 1. 提取特征
        feat = self.feature_extractor(bev_image)  # (B, C', H', W')
        C, H, W = feat.shape[1], feat.shape[2], feat.shape[3]

        # 2. 展平并添加位置编码
        feat = feat.flatten(2).transpose(1, 2)  # (B, H'*W', C')
        feat = feat + self.pos_embed[:, :H * W, :].to(feat.device)

        # 3. Transformer
        feat = self.transformer(feat)  # (B, H'*W', C')

        # 4. 全局特征
        feat_t = feat.transpose(1, 2)  # (B, C', H'*W')
        global_feat = self.pool(feat_t).squeeze(-1)  # (B, C')
        global_feat = self.fc(global_feat)  # (B, D)

        # L2 归一化
        global_feat = F.normalize(global_feat, p=2, dim=1)

        if return_local:
            # 局部特征用于细粒度匹配
            local_feat = self.local_fc(feat)  # (B, H'*W', D)
            local_feat = F.normalize(local_feat, p=2, dim=-1)
            return global_feat, local_feat

        return global_feat

    @torch.no_grad()
    def extract_features(
        self,
        bev_image: np.ndarray,
        device: str = "cuda",
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        从 BEV 图像提取特征 (便捷方法)

        Args:
            bev_image: (H, W, C) 或 (H, W) BEV 图像
            device: 计算设备

        Returns:
            global_feat: (D,) 全局描述符
            local_feat: (N, D) 局部描述符 (如果 return_local=True)
        """
        if not _torch_available:
            raise ImportError("torch 不可用")

        # 转换为 tensor
        if bev_image.ndim == 2:
            bev_image = bev_image[:, :, np.newaxis]
        bev_tensor = torch.from_numpy(bev_image).float()
        bev_tensor = bev_tensor.permute(2, 0, 1).unsqueeze(0)  # (1, C, H, W)
        bev_tensor = bev_tensor.to(device)

        # 推理
        self.eval()
        global_feat, local_feat = self.forward(bev_tensor, return_local=True)
        g_np = global_feat.cpu().numpy().flatten()
        l_np = local_feat.cpu().numpy().squeeze(0) if local_feat is not None else None
        _LOG.debug("[OverlapTransformer] extract_features 输出 global=%s local=%s", g_np.shape, getattr(l_np, "shape", None))
        return (g_np, l_np)


def _detect_legacy_conv1(state_dict: dict) -> bool:
    """若 checkpoint 中 conv1 的 weight 为 [16, 1, 5, 1]，则使用 legacy 结构。"""
    for k, v in state_dict.items():
        if not hasattr(v, "shape") or v.dim() != 4:
            continue
        if "conv1" in k and "weight" in k:
            if tuple(v.shape) == (16, 1, 5, 1):
                return True
            break
    return False


def load_overlap_transformer(
    checkpoint_path: str,
    device: str = "cuda",
    **kwargs,
) -> OverlapTransformer:
    """
    加载预训练的 Overlap Transformer

    若 checkpoint 中 conv1 卷积形状为 [16, 1, 5, 1]（官方 pretrained_overlap_transformer.pth.tar），
    会自动使用 use_legacy_feature_extractor 以匹配结构。

    Args:
        checkpoint_path: checkpoint 路径 (.pth.tar 或 .pth)
        device: 计算设备
        **kwargs: 传递给 OverlapTransformer 的其他参数（如 use_legacy_feature_extractor 可显式指定）

    Returns:
        model: 加载权重后的模型
    """
    if not _torch_available:
        raise ImportError("torch 不可用")

    # 加载权重并检测结构
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    if "state_dict" in checkpoint:
        state_dict = checkpoint["state_dict"]
    elif "model" in checkpoint:
        state_dict = checkpoint["model"]
    else:
        state_dict = checkpoint

    # 移除可能的前缀 (如 "module.")
    new_state_dict = {}
    for k, v in state_dict.items():
        if k.startswith("module."):
            new_state_dict[k[7:]] = v
        else:
            new_state_dict[k] = v

    # 自动检测是否为官方 checkpoint（conv1 [16, 1, 5, 1]）
    use_legacy = kwargs.pop("use_legacy_feature_extractor", None)
    if use_legacy is None:
        use_legacy = _detect_legacy_conv1(new_state_dict)
    if use_legacy:
        kwargs["use_legacy_feature_extractor"] = True

    # 先做 key 映射，再根据映射后的 state 检测是否为 MIAS 官方 backbone（layer1.0 为 16→32 通道 (3,1)）
    _CONV_BN_TO_LAYER = [
        ("conv2.", "feature_extractor.layer1.0.conv."),
        ("bn2.", "feature_extractor.layer1.0.bn."),
        ("conv3.", "feature_extractor.layer1.1.conv."),
        ("bn3.", "feature_extractor.layer1.1.bn."),
        ("conv4.", "feature_extractor.layer2.0.conv."),
        ("bn4.", "feature_extractor.layer2.0.bn."),
        ("conv5.", "feature_extractor.layer2.1.conv."),
        ("bn5.", "feature_extractor.layer2.1.bn."),
        ("conv6.", "feature_extractor.layer3.0.conv."),
        ("bn6.", "feature_extractor.layer3.0.bn."),
        ("conv7.", "feature_extractor.layer3.1.conv."),
        ("bn7.", "feature_extractor.layer3.1.bn."),
    ]
    remapped = {}
    for k, v in new_state_dict.items():
        new_key = k
        if not k.startswith("feature_extractor."):
            if k.startswith("conv1."):
                new_key = "feature_extractor.conv1.0.conv." + k[6:]
            elif k.startswith("bn1."):
                new_key = "feature_extractor.conv1.0.bn." + k[4:]
            elif k.startswith("layer1."):
                new_key = "feature_extractor.layer1." + k[7:]
            elif k.startswith("layer2."):
                new_key = "feature_extractor.layer2." + k[7:]
            elif k.startswith("layer3."):
                new_key = "feature_extractor.layer3." + k[7:]
            elif k.startswith("pool1."):
                new_key = "feature_extractor.pool1." + k[6:]
            else:
                for prefix, repl in _CONV_BN_TO_LAYER:
                    if k.startswith(prefix):
                        suffix = k[len(prefix):]
                        new_key = repl + suffix
                        break
        remapped[new_key] = v
    new_state_dict = remapped

    # 检测 MIAS 官方 backbone：layer1.0.conv.weight 形状为 (32, 16, 3, 1)
    use_mias_official = kwargs.pop("use_mias_official_feature_extractor", None)
    if use_mias_official is None:
        k = "feature_extractor.layer1.0.conv.weight"
        if k in new_state_dict:
            sh = getattr(new_state_dict[k], "shape", None)
            if sh is not None and len(sh) == 4 and tuple(sh) == (32, 16, 3, 1):
                use_mias_official = True
        if use_mias_official is None:
            use_mias_official = False
    if use_mias_official:
        kwargs["use_mias_official_feature_extractor"] = True

    # 创建模型（结构需与 checkpoint 一致）
    model = OverlapTransformer(**kwargs)

    # MIAS 官方 checkpoint 的 pos_embed/transformer 与训练时 BEV 分辨率绑定（如 65280 等），
    # 与当前 BEV 尺寸不一致会导致 "tensor a (65280) must match tensor b (4096)"。仅加载 backbone。
    if use_mias_official:
        load_dict = {k: v for k, v in new_state_dict.items() if k.startswith("feature_extractor.")}
        _LOG.info("[OverlapTransformer] MIAS 官方模式: 仅加载 feature_extractor (%d 个键)，pos_embed/transformer 使用当前尺寸初始化", len(load_dict))
        missing, unexpected = model.load_state_dict(load_dict, strict=False)
        # 局部特征来自未训练的 transformer，C3M 跨模态匹配不可靠
        model._local_feature_unreliable = True
    else:
        missing, unexpected = model.load_state_dict(new_state_dict, strict=False)
        model._local_feature_unreliable = False

    # MIAS 官方模式下仅加载 backbone，pos_embed/transformer 缺失为预期行为，不按错误告警
    _missing_expected = all(
        m == "pos_embed" or m.startswith("transformer.") for m in missing
    )
    if missing:
        if use_mias_official and _missing_expected:
            _LOG.info(
                "[OverlapTransformer] pos_embed 与 transformer 未从 checkpoint 加载（符合 MIAS 官方设计），已使用当前尺寸初始化"
            )
        else:
            _LOG.warning("[OverlapTransformer] 缺失的键 (%d 个): %s ...", len(missing), missing[:5])
    if unexpected:
        _LOG.warning("[OverlapTransformer] 意外的键 (%d 个): %s ...", len(unexpected), unexpected[:5])

    model.to(device)
    model.eval()

    _LOG.info(
        "[OverlapTransformer] 成功加载 checkpoint: %s (legacy=%s mias_official=%s 缺失=%d 意外=%d)",
        checkpoint_path, use_legacy, use_mias_official, len(missing), len(unexpected),
    )
    return model
