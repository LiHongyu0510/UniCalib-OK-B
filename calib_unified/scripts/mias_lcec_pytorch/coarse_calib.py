"""
MIAS-LCEC 纯 PyTorch 粗标定主入口

支持两种模式:
1) 全量模式: BEV + OverlapTransformer + MobileSAM + C3M + PnP（需 pretrained_overlap_transformer.pth.tar + mobile_sam.pt）
2) 仅 SAM 模式: 仅加载 mobile_sam.pt，用 SAM 分割 + 网格搜索平移 + 投影与 mask 对应 + PnP，不依赖 .pth.tar。

彻底摆脱 .so 库和 Python 3.10 依赖。
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import replace
from typing import Optional, Tuple

import numpy as np

_LOG = logging.getLogger(__name__)

# 延迟导入
_torch_available = False
try:
    import torch
    _torch_available = True
except ImportError:
    pass

from .bev_projection import BEVProjector, BEVConfig, point_cloud_to_bev, load_pcd_as_bev
from .overlap_transformer import OverlapTransformer, load_overlap_transformer
from .mobile_sam_wrapper import MobileSAMWrapper, SimplifiedMobileSAM, create_sam_wrapper
from .c3m_matching import C3MMatcher, C3MConfig


def _euler_deg_to_R(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """从欧拉角 (度) 构建旋转矩阵 R: 绕 ZYX 外旋，R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
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


class MIASLCECCoarseCalib:
    """
    MIAS-LCEC 粗标定器

    使用纯 PyTorch 实现，无需 .so 库，Python 3.10 依赖。
    """

    def __init__(
        self,
        overlap_ckpt: Optional[str] = None,
        mobilesam_ckpt: Optional[str] = None,
        device: str = "cuda",
        bev_config: Optional[BEVConfig] = None,
        c3m_config: Optional[C3MConfig] = None,
        use_sam_only: bool = False,
    ):
        """
        初始化粗标定器

        Args:
            overlap_ckpt: Overlap Transformer 权重路径；为空或 None 且 use_sam_only 时仅用 SAM 粗标定
            mobilesam_ckpt: MobileSAM 权重路径 (可选，自动查找)
            device: 计算设备
            bev_config: BEV 投影配置
            c3m_config: C3M 匹配配置
            use_sam_only: 若 True，不加载 OverlapTransformer，仅用 mobile_sam.pt 做 SAM+网格 PnP
        """
        self.device = device
        self.overlap_ckpt = (overlap_ckpt or "").strip() or None
        self.mobilesam_ckpt = mobilesam_ckpt
        self.use_sam_only = use_sam_only

        # 配置
        self.bev_config = bev_config or BEVConfig()
        self.c3m_config = c3m_config or C3MConfig()
        # 环境变量可覆盖 C3M 相似度阈值（便于不同数据集调节，如 0.5–0.7）
        env_sim = os.environ.get("UNICALIB_C3M_SIMILARITY_THRESHOLD", "").strip()
        if env_sim:
            try:
                self.c3m_config.similarity_threshold = float(env_sim)
                _LOG.debug("[MIAS-LCEC] C3M similarity_threshold 从环境变量设为 %.2f", self.c3m_config.similarity_threshold)
            except ValueError:
                pass

        # 环境变量可覆盖迭代精化配置（用于在不改 C++ 的情况下快速验证鲁棒性提升）
        env_iter = os.environ.get("UNICALIB_C3M_USE_ITERATIVE_REFINE", "").strip().lower()
        if env_iter in ("1", "true", "yes", "y", "on"):
            self.c3m_config.use_iterative_refine = True
        elif env_iter in ("0", "false", "no", "n", "off"):
            self.c3m_config.use_iterative_refine = False
        env_iter_max = os.environ.get("UNICALIB_C3M_ITER_MAX", "").strip()
        if env_iter_max:
            try:
                self.c3m_config.iterative_refine_max_iter = int(env_iter_max)
            except ValueError:
                pass
        env_iter_thresh = os.environ.get("UNICALIB_C3M_ITER_THRESH", "").strip()
        if env_iter_thresh:
            try:
                self.c3m_config.iterative_refine_convergence_threshold = float(env_iter_thresh)
            except ValueError:
                pass

        # 模型 (延迟加载)
        self._overlap_net = None
        self._sam_wrapper = None
        self._c3m_matcher = None
        self._initialized = False
        # 当 OverlapTransformer 仅加载 backbone 时，局部特征不可靠，优先用 SAM-only
        self._ot_local_unreliable = False

    def _lazy_init(self):
        """延迟初始化模型"""
        if self._initialized:
            return

        _LOG.info("[MIAS-LCEC] 初始化模型...")

        # CPU 下使用较小 BEV/模型尺寸以降低内存，避免 OOM (exit 137)
        if self.device == "cpu":
            self.bev_config = replace(
                self.bev_config,
                width=512,
                height=512,
            )
            _LOG.info("[MIAS-LCEC] 设备为 CPU，BEV 尺寸已设为 512x512 以降低内存占用")

        # 1. 加载 Overlap Transformer（仅非 SAM-only 且路径存在时）
        if not self.use_sam_only and self.overlap_ckpt and _torch_available and os.path.isfile(self.overlap_ckpt):
            try:
                # input_size 为 (H, W) 与 BEV shape 一致
                self._overlap_net = load_overlap_transformer(
                    self.overlap_ckpt,
                    device=self.device,
                    input_size=(self.bev_config.height, self.bev_config.width),
                )
                self._ot_local_unreliable = getattr(
                    self._overlap_net, "_local_feature_unreliable", False
                )
                if self._ot_local_unreliable:
                    _LOG.warning(
                        "[MIAS-LCEC] OverlapTransformer 仅加载 backbone，局部特征未训练，C3M 可能不可靠；将优先尝试 SAM-only 路径"
                    )
                _LOG.info("[MIAS-LCEC] OverlapTransformer 加载成功")
            except Exception as e:
                _LOG.warning("[MIAS-LCEC] OverlapTransformer 加载失败: %s", e)
        elif self.use_sam_only or not self.overlap_ckpt:
            _LOG.info("[MIAS-LCEC] 仅 SAM 模式：不加载 OverlapTransformer (use_sam_only=%s)", self.use_sam_only)
        else:
            _LOG.warning("[MIAS-LCEC] OverlapTransformer 权重不存在或不可读: %s", self.overlap_ckpt)

        # 2. 加载 SAM
        try:
            self._sam_wrapper = create_sam_wrapper(
                self.mobilesam_ckpt,
                device=self.device,
                use_fallback=True,
            )
            if self._sam_wrapper.is_available():
                _LOG.info("[MIAS-LCEC] SAM 分割器加载成功")
            else:
                _LOG.warning("[MIAS-LCEC] SAM 分割器不可用，将使用简化实现")
        except Exception as e:
            _LOG.warning("[MIAS-LCEC] SAM 加载失败: %s", e)
            self._sam_wrapper = None

        # 3. C3M Matcher
        self._c3m_matcher = C3MMatcher(self.c3m_config)

        self._initialized = True
        _LOG.info("[MIAS-LCEC] 初始化完成 — OverlapTransformer=%s SAM=%s",
                  "OK" if self._overlap_net else "未加载",
                  "OK" if (self._sam_wrapper and self._sam_wrapper.is_available()) else "简化/未加载")

    def estimate_extrinsic(
        self,
        pcd_path: str,
        image_path: str,
        camera_k: np.ndarray,
        camera_dist: Optional[np.ndarray] = None,
        initial_rt: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        估计 LiDAR -> Camera 外参 (R, t)

        Args:
            pcd_path: PCD 文件路径
            image_path: 图像文件路径
            camera_k: (3, 3) 相机内参矩阵
            camera_dist: (5,) 畸变系数
            initial_rt: 初始 (R, t)，用于几何约束

        Returns:
            (R, t): 旋转矩阵 (3, 3) 和平移向量 (3,)
            None: 估计失败
        """
        t0_total = time.perf_counter()
        self._lazy_init()

        _LOG.info("[MIAS-LCEC] ---------- estimate_extrinsic 开始 ----------")
        _LOG.info("[MIAS-LCEC] 输入: pcd=%s image=%s K.diag=[%.1f,%.1f,1]",
                  pcd_path, image_path, camera_k[0, 0], camera_k[1, 1])

        # 1. 加载数据
        t1 = time.perf_counter()
        points, intensities = self._load_pcd(pcd_path)
        image = self._load_image(image_path)
        _LOG.info("[MIAS-LCEC] [1/7] 加载数据 耗时=%.3fs 点云数=%d 强度=%s 图像=%s",
                  time.perf_counter() - t1, len(points),
                  "有" if intensities is not None else "无",
                  "OK %dx%d" % (image.shape[1], image.shape[0]) if image is not None else "None")

        if len(points) == 0:
            _LOG.error("[MIAS-LCEC] [1/7] 失败 — 点云为空，请检查 PCD 路径与格式")
            return None

        if image is None:
            _LOG.error("[MIAS-LCEC] [1/7] 失败 — 图像加载失败，请检查路径与 Pillow/cv2")
            return None

        # 2. BEV 投影（传入强度以提升特征质量）
        t2 = time.perf_counter()
        bev, bev_meta = point_cloud_to_bev(
            points,
            resolution=self.bev_config.resolution,
            width=self.bev_config.width,
            height=self.bev_config.height,
            height_range=(self.bev_config.height_min, self.bev_config.height_max),
            intensities=intensities,
        )
        _LOG.info("[MIAS-LCEC] [2/7] BEV 投影 耗时=%.3fs shape=%s x_range=%s y_range=%s n_valid=%s",
                  time.perf_counter() - t2, bev.shape,
                  bev_meta.get("x_range"), bev_meta.get("y_range"), bev_meta.get("n_valid"))

        # BEV 与 OverlapTransformer 尺寸运行时检查（避免 512 vs 1024 混用导致特征错位）
        if self._overlap_net is not None:
            expected_h, expected_w = getattr(self._overlap_net, "input_size", (None, None))
            if expected_h is not None and expected_w is not None:
                if bev.shape[0] != expected_h or bev.shape[1] != expected_w:
                    _LOG.warning(
                        "[MIAS-LCEC] BEV 与 OverlapTransformer 尺寸不一致: BEV=%s 模型 input_size=(%s,%s)，特征空间可能错位",
                        bev.shape[:2], expected_h, expected_w,
                    )

        # 3. Overlap Transformer 特征提取 (预训练模型为 1 通道 BEV，多通道时取首通道)
        t3 = time.perf_counter()
        if self._overlap_net is not None:
            try:
                bev_input = bev[..., 0:1] if bev.shape[-1] > 1 else bev
                lidar_global_feat, lidar_local_feat = self._overlap_net.extract_features(
                    bev_input, device=self.device
                )
                _LOG.info("[MIAS-LCEC] [3/7] OverlapTransformer 耗时=%.3fs global=%s local=%s",
                          time.perf_counter() - t3,
                          getattr(lidar_global_feat, "shape", None),
                          getattr(lidar_local_feat, "shape", None) if lidar_local_feat is not None else "None")
                if self.device == "cpu":
                    import gc
                    gc.collect()
            except Exception as e:
                import traceback
                _LOG.warning("[MIAS-LCEC] [3/7] OverlapTransformer 推理异常: %s", e)
                _LOG.debug("[MIAS-LCEC] traceback:\n%s", traceback.format_exc())
                lidar_global_feat = np.zeros(256, dtype=np.float32)
                lidar_local_feat = None
        else:
            _LOG.warning("[MIAS-LCEC] [3/7] OverlapTransformer 未加载，使用零特征")
            lidar_global_feat = np.zeros(256, dtype=np.float32)
            lidar_local_feat = None

        # 4. 图像分割 + 特征提取
        t4 = time.perf_counter()
        if self._sam_wrapper is not None and self._sam_wrapper.is_available():
            try:
                masks, scores = self._sam_wrapper.segment(image)
                image_features = self._sam_wrapper.get_mask_features(image, masks)
                _LOG.info("[MIAS-LCEC] [4/7] SAM 分割 耗时=%.3fs masks=%d image_feat=%s",
                          time.perf_counter() - t4, len(masks), getattr(image_features, "shape", None))
            except Exception as e:
                import traceback
                _LOG.warning("[MIAS-LCEC] [4/7] SAM 分割异常: %s", e)
                _LOG.debug("[MIAS-LCEC] traceback:\n%s", traceback.format_exc())
                masks = np.zeros((0, image.shape[0], image.shape[1]), dtype=bool)
                image_features = np.zeros((0, 256), dtype=np.float32)
        else:
            _LOG.info("[MIAS-LCEC] [4/7] SAM 使用简化实现")
            masks = np.zeros((0, image.shape[0], image.shape[1]), dtype=bool)
            image_features = np.zeros((0, 256), dtype=np.float32)

        # 仅 SAM 模式或 OT 局部特征不可靠：优先用 SAM 分割 + 网格搜索 PnP（含旋转搜索）
        use_sam_first = (
            (self._overlap_net is None or self._ot_local_unreliable)
            and len(masks) > 0
            and self._sam_wrapper is not None
            and self._sam_wrapper.is_available()
        )
        if use_sam_first:
            _LOG.info(
                "[MIAS-LCEC] %s — 使用 mask 对应 + 网格搜索 PnP（含旋转搜索）",
                "仅 SAM 模式" if self._overlap_net is None else "OT 局部特征不可靠，优先 SAM-only",
            )
            result = self._estimate_sam_only_pnp(points, image, masks, camera_k, camera_dist)
            if result is not None:
                R, t = result
                _LOG.info("[MIAS-LCEC] ---------- estimate_extrinsic 成功 (SAM-only) 总耗时=%.3fs t=[%.4f, %.4f, %.4f] ----------",
                          time.perf_counter() - t0_total, t[0], t[1], t[2])
                return R, t
            _LOG.warning("[MIAS-LCEC] SAM-only PnP 未得到有效解，返回 None")
            if self._overlap_net is None:
                return None
        elif self._overlap_net is None:
            return None

        # 5. 为每个 BEV 特征格点构造对应 3D 点
        t5 = time.perf_counter()
        lidar_points_for_c3m = self._compute_lidar_points_per_feature(
            points, bev_meta, lidar_local_feat
        )
        if lidar_points_for_c3m is None:
            _LOG.warning("[MIAS-LCEC] [5/7] 失败 — 无法构造特征格点 3D 点 (lidar_local_feat 为空或异常)")
            return None
        _LOG.info("[MIAS-LCEC] [5/7] 特征格点 3D 耗时=%.3fs shape=%s",
                  time.perf_counter() - t5, lidar_points_for_c3m.shape)

        # 6. C3M Matching
        if len(masks) == 0 or lidar_local_feat is None:
            _LOG.warning("[MIAS-LCEC] [6/7] 跳过 C3M — 无分割(masks=%d) 或无局部特征(local=%s)",
                         len(masks), "None" if lidar_local_feat is None else "OK")
            return None

        t6 = time.perf_counter()
        try:
            obj_pts, img_pts, inlier_mask = self._c3m_matcher.match(
                lidar_features=lidar_local_feat,
                image_features=image_features,
                lidar_points=lidar_points_for_c3m,
                image_masks=masks,
                camera_k=camera_k,
                initial_rt=initial_rt,
                camera_dist=camera_dist,
            )
            n_in = int(inlier_mask.sum()) if inlier_mask is not None and inlier_mask.size else 0
            _LOG.info("[MIAS-LCEC] [6/7] C3M 匹配 耗时=%.3fs 对应点=%d 内点=%d",
                      time.perf_counter() - t6, len(obj_pts), n_in)
        except Exception as e:
            import traceback
            _LOG.warning("[MIAS-LCEC] [6/7] C3M 匹配异常: %s", e)
            _LOG.debug("[MIAS-LCEC] traceback:\n%s", traceback.format_exc())
            return None

        if len(obj_pts) < 6:
            _LOG.warning("[MIAS-LCEC] [6/7] 失败 — 匹配点不足: %d < 6 (需至少 6 对用于 PnP)", len(obj_pts))
            return None

        # 7. PnP 求解
        t7 = time.perf_counter()
        result = self._solve_pnp(obj_pts, img_pts, camera_k, camera_dist)
        _LOG.info("[MIAS-LCEC] [7/7] PnP 求解 耗时=%.3fs 结果=%s",
                  time.perf_counter() - t7, "OK" if result is not None else "失败")

        if result is not None:
            R, t = result
            _LOG.info("[MIAS-LCEC] ---------- estimate_extrinsic 成功 总耗时=%.3fs t=[%.4f, %.4f, %.4f] ----------",
                      time.perf_counter() - t0_total, t[0], t[1], t[2])
            return R, t
        else:
            _LOG.warning("[MIAS-LCEC] ---------- estimate_extrinsic 失败 (PnP 未得到有效解) 总耗时=%.3fs ----------",
                         time.perf_counter() - t0_total)
            return None

    def _sam_only_one_hypothesis(
        self,
        points: np.ndarray,
        masks: np.ndarray,
        camera_k: np.ndarray,
        camera_dist: np.ndarray,
        R_guess: np.ndarray,
        t_guess: np.ndarray,
        w: int,
        h: int,
        n_masks: int,
        min_masks_for_pnp: int,
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray], int]:
        """
        单次假设：用 (R_guess, t_guess) 投影点云，按 mask 聚合成 3D-2D 对后 PnP。
        返回 (ok, R, t, n_inliers)。
        """
        import cv2
        from collections import defaultdict

        pts_cam = (R_guess @ points.T).T + t_guess
        in_front = pts_cam[:, 2] > 1e-6
        if in_front.sum() < min_masks_for_pnp:
            return False, None, None, -1
        pts_cam_fwd = pts_cam[in_front].astype(np.float32)
        proj_2d, _ = cv2.projectPoints(
            pts_cam_fwd,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            camera_k.astype(np.float32),
            camera_dist.astype(np.float32),
        )
        proj_2d = proj_2d.reshape(-1, 2)
        valid_finite = np.isfinite(proj_2d).all(axis=1)
        in_view = (
            valid_finite
            & (proj_2d[:, 0] >= 0) & (proj_2d[:, 0] < w)
            & (proj_2d[:, 1] >= 0) & (proj_2d[:, 1] < h)
        )
        if in_view.sum() < min_masks_for_pnp:
            return False, None, None, -1

        # 仅对 valid 点取整，避免 NaN/Inf 触发 cast 警告
        ix = np.zeros(len(proj_2d), dtype=np.int32)
        iy = np.zeros(len(proj_2d), dtype=np.int32)
        ix[in_view] = np.clip(np.round(proj_2d[in_view, 0]).astype(np.int32), 0, w - 1)
        iy[in_view] = np.clip(np.round(proj_2d[in_view, 1]).astype(np.int32), 0, h - 1)
        in_front_idx = np.where(in_front)[0]
        mask_ids = []
        for i in range(len(proj_2d)):
            if not in_view[i]:
                mask_ids.append(-1)
                continue
            mid = -1
            for m in range(n_masks):
                if masks[m][iy[i], ix[i]]:
                    mid = m
                    break
            mask_ids.append(mid)

        group_3d = defaultdict(list)
        for i in range(len(proj_2d)):
            if in_view[i] and mask_ids[i] >= 0:
                group_3d[mask_ids[i]].append(points[in_front_idx[i]])
        obj_list, img_list = [], []
        for mid, pts3 in group_3d.items():
            if len(pts3) < 1:
                continue
            ys, xs = np.where(masks[mid])
            if len(xs) < 3:
                continue
            obj_list.append(np.mean(pts3, axis=0).astype(np.float64))
            img_list.append((np.mean(xs).astype(np.float64), np.mean(ys).astype(np.float64)))
        if len(obj_list) < min_masks_for_pnp:
            return False, None, None, -1

        obj_pts = np.array(obj_list, dtype=np.float32)
        img_pts = np.array(img_list, dtype=np.float32).reshape(-1, 2)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts, img_pts,
            camera_k.astype(np.float32),
            camera_dist.astype(np.float32),
            reprojectionError=self.c3m_config.ransac_reproj_threshold,
            confidence=self.c3m_config.ransac_confidence,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok or inliers is None:
            return False, None, None, -1
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.ravel().astype(np.float64)
        return True, R.astype(np.float64), t, inliers.size

    def _estimate_sam_only_pnp(
        self,
        points: np.ndarray,
        image: np.ndarray,
        masks: np.ndarray,
        camera_k: np.ndarray,
        camera_dist: Optional[np.ndarray],
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        仅 SAM 模式：先对平移网格搜索，再在最佳平移上做小范围旋转搜索，将投影点归属到 SAM mask，
        按 mask 聚合成 3D-2D 对后 PnP。不依赖 OverlapTransformer。
        """
        try:
            import cv2
        except ImportError:
            _LOG.error("[MIAS-LCEC] SAM-only PnP 需要 cv2")
            return None

        if camera_dist is None:
            camera_dist = np.zeros(5, dtype=np.float64)

        h, w = image.shape[:2]
        if masks.ndim == 2:
            masks = np.expand_dims(masks, 0)
        n_masks = masks.shape[0]
        min_masks_for_pnp = 4
        mask_centroids = []
        for m in range(n_masks):
            ys, xs = np.where(masks[m])
            if len(xs) < 3:
                continue
            mask_centroids.append((np.mean(xs).astype(np.float64), np.mean(ys).astype(np.float64)))
        if len(mask_centroids) < min_masks_for_pnp:
            _LOG.warning("[MIAS-LCEC] SAM-only: 有效 mask 数 %d < %d", len(mask_centroids), min_masks_for_pnp)
            return None

        best_R, best_t, best_inliers = None, None, -1
        n_hypotheses = 0

        # Stage 1: 平移网格搜索 (R = I)
        tz_range = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        tx_range = [-0.5, -0.3, -0.1, 0.0, 0.1, 0.3]
        ty_range = [-0.3, -0.2, -0.1, 0.0, 0.1]
        for tz in tz_range:
            for tx in tx_range:
                for ty in ty_range:
                    t_guess = np.array([tx, ty, tz], dtype=np.float64)
                    R_guess = np.eye(3, dtype=np.float64)
                    ok, R, t, nin = self._sam_only_one_hypothesis(
                        points, masks, camera_k, camera_dist,
                        R_guess, t_guess, w, h, n_masks, min_masks_for_pnp,
                    )
                    n_hypotheses += 1
                    if ok and nin >= min_masks_for_pnp and nin > best_inliers:
                        best_inliers = nin
                        best_R, best_t = R, t

        # Stage 2: 在最佳平移上做小范围旋转搜索 (yaw/pitch/roll)
        if best_R is not None and best_t is not None:
            yaw_deg_list = [-6, -4, -2, 0, 2, 4, 6]
            pitch_deg_list = [-4, 0, 4]
            roll_deg_list = [0]
            for yaw_deg in yaw_deg_list:
                for pitch_deg in pitch_deg_list:
                    for roll_deg in roll_deg_list:
                        if yaw_deg == 0 and pitch_deg == 0 and roll_deg == 0:
                            continue  # 已在 Stage 1 试过 R=I
                        R_guess = _euler_deg_to_R(yaw_deg, pitch_deg, roll_deg)
                        ok, R, t, nin = self._sam_only_one_hypothesis(
                            points, masks, camera_k, camera_dist,
                            R_guess, best_t.copy(), w, h, n_masks, min_masks_for_pnp,
                        )
                        n_hypotheses += 1
                        if ok and nin >= min_masks_for_pnp and nin > best_inliers:
                            best_inliers = nin
                            best_R, best_t = R, t

        if best_R is not None and best_inliers >= min_masks_for_pnp:
            _LOG.info("[MIAS-LCEC] SAM-only 网格搜索完成 假设数=%d 最佳 inliers=%d t=[%.4f, %.4f, %.4f]",
                      n_hypotheses, best_inliers, best_t[0], best_t[1], best_t[2])
            
            # 迭代精化 (CalibRefine风格)：在当前最佳解附近重新聚合 3D-2D 后 PnP
            if getattr(self.c3m_config, "use_iterative_refine", False):
                best_R, best_t, best_inliers = self._iterative_refine_sam_only(
                    points, masks, camera_k, camera_dist,
                    best_R, best_t, best_inliers, w, h, n_masks, min_masks_for_pnp,
                )
            return best_R, best_t
        _LOG.warning("[MIAS-LCEC] SAM-only 网格搜索无有效解 (最佳 inliers=%d)", best_inliers)
        return None

    def _iterative_refine_sam_only(
        self,
        points: np.ndarray,
        masks: np.ndarray,
        camera_k: np.ndarray,
        camera_dist: np.ndarray,
        best_R: np.ndarray,
        best_t: np.ndarray,
        best_inliers: int,
        w: int,
        h: int,
        n_masks: int,
        min_masks_for_pnp: int,
    ) -> Tuple[np.ndarray, np.ndarray, int]:
        """迭代精化：在当前最佳 R,t 下重新做投影→mask 聚合→PnP，直至收敛或达到最大迭代次数。"""
        max_iter = getattr(self.c3m_config, "iterative_refine_max_iter", 3)
        conv_thresh = getattr(self.c3m_config, "iterative_refine_convergence_threshold", 0.01)
        for it in range(max_iter):
            ok, R_new, t_new, nin = self._sam_only_one_hypothesis(
                points, masks, camera_k, camera_dist,
                best_R, best_t, w, h, n_masks, min_masks_for_pnp,
            )
            if not ok or nin < min_masks_for_pnp:
                break
            improvement = (nin - best_inliers) / max(best_inliers, 1)
            if nin > best_inliers:
                best_R, best_t, best_inliers = R_new, t_new, nin
            if improvement < conv_thresh:
                _LOG.info("[MIAS-LCEC] 迭代精化 第 %d 轮收敛 improvement=%.4f inliers=%d", it + 1, improvement, best_inliers)
                break
        return best_R, best_t, best_inliers

    def _compute_lidar_points_per_feature(
        self,
        points: np.ndarray,
        bev_meta: dict,
        lidar_local_feat: np.ndarray,
    ) -> Optional[np.ndarray]:
        """
        为每个 BEV 特征格点构造对应 3D 点（与 lidar_local_feat 一一对应）。
        特征图一般为 64x64，将点云按 (x,y) 分格取质心，空格用格心 (x,y,0)。
        """
        if lidar_local_feat is None or len(lidar_local_feat) == 0:
            _LOG.debug("[MIAS-LCEC] _compute_lidar_points_per_feature: lidar_local_feat 为空")
            return None
        n_feat = lidar_local_feat.shape[0]
        feat_h = feat_w = int(round(n_feat ** 0.5))
        if feat_h * feat_w != n_feat:
            feat_h, feat_w = 64, 64
            if feat_h * feat_w != n_feat:
                _LOG.warning("[MIAS-LCEC] 特征数 %d 非平方数，使用 64x64 网格", n_feat)
                n_feat = feat_h * feat_w
        _LOG.debug("[MIAS-LCEC] 特征网格 feat_h=%d feat_w=%d n_feat=%d 点云数=%d",
                   feat_h, feat_w, n_feat, len(points))

        x_range = bev_meta.get("x_range", (points[:, 0].min(), points[:, 0].max()))
        y_range = bev_meta.get("y_range", (points[:, 1].min(), points[:, 1].max()))
        x_min, x_max = float(x_range[0]), float(x_range[1])
        y_min, y_max = float(y_range[0]), float(y_range[1])
        if x_max <= x_min:
            x_max = x_min + 1.0
        if y_max <= y_min:
            y_max = y_min + 1.0

        xy = points[:, :2]
        z = points[:, 2]
        # 格索引 [0, feat_w), [0, feat_h)
        ix = ((xy[:, 0] - x_min) / (x_max - x_min) * feat_w).astype(np.int32)
        iy = ((xy[:, 1] - y_min) / (y_max - y_min) * feat_h).astype(np.int32)
        ix = np.clip(ix, 0, feat_w - 1)
        iy = np.clip(iy, 0, feat_h - 1)
        linear = iy * feat_w + ix

        # 每格质心
        out = np.zeros((feat_h * feat_w, 3), dtype=np.float64)
        count = np.zeros(feat_h * feat_w, dtype=np.float64)
        np.add.at(out, (linear, 0), xy[:, 0])
        np.add.at(out, (linear, 1), xy[:, 1])
        np.add.at(out, (linear, 2), z)
        np.add.at(count, linear, 1.0)

        # 有点的格：质心
        mask = count > 0
        out[mask, 0] /= count[mask]
        out[mask, 1] /= count[mask]
        out[mask, 2] /= count[mask]
        # 空格：格心 (x_center, y_center, 0)
        for k in range(feat_h * feat_w):
            if not mask[k]:
                j, i = k // feat_w, k % feat_w
                out[k, 0] = x_min + (i + 0.5) * (x_max - x_min) / feat_w
                out[k, 1] = y_min + (j + 0.5) * (y_max - y_min) / feat_h
                out[k, 2] = 0.0

        # 若 lidar_local_feat 长度与 out 不一致（如模型输出不同尺寸），截断或重复以匹配
        if out.shape[0] != lidar_local_feat.shape[0]:
            n = lidar_local_feat.shape[0]
            if out.shape[0] > n:
                out = out[:n]
            else:
                pad = np.zeros((n - out.shape[0], 3), dtype=np.float64)
                out = np.concatenate([out, pad], axis=0)
        return out

    def _load_pcd(self, path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """加载 PCD 文件，返回 (points Nx3, intensities 或 None)。"""
        try:
            pts, intensities = _read_pcd_ascii(path)
            if len(pts) == 0:
                _LOG.warning("[MIAS-LCEC] PCD 解析后点数为 0: %s (请检查路径与 ASCII PCD 格式)", path)
            if intensities is not None:
                _LOG.debug("[MIAS-LCEC] PCD 已解析强度 点数=%d", len(pts))
            return pts, intensities
        except FileNotFoundError:
            _LOG.error("[MIAS-LCEC] PCD 文件不存在: %s", path)
            return np.zeros((0, 3), dtype=np.float64), None
        except Exception as e:
            import traceback
            _LOG.error("[MIAS-LCEC] PCD 加载异常 path=%s: %s", path, e)
            _LOG.debug("[MIAS-LCEC] PCD 加载 traceback:\n%s", traceback.format_exc())
            return np.zeros((0, 3), dtype=np.float64), None

    def _load_image(self, path: str) -> Optional[np.ndarray]:
        """加载图像"""
        try:
            # 优先用 Pillow (避免 NumPy 2.x 下 cv2 崩溃)
            from PIL import Image
            img = Image.open(path)
            if img.mode != "RGB":
                img = img.convert("RGB")
            arr = np.asarray(img, dtype=np.uint8)
            # BGR 顺序
            arr = arr[:, :, ::-1].copy()
            return arr
        except ImportError:
            pass

        try:
            import cv2
            image = cv2.imread(path)
            return image
        except Exception as e:
            import traceback
            _LOG.error("[MIAS-LCEC] 图像加载失败 path=%s: %s", path, e)
            _LOG.debug("[MIAS-LCEC] 图像加载 traceback:\n%s", traceback.format_exc())
            return None

    def _solve_pnp(
        self,
        obj_pts: np.ndarray,
        img_pts: np.ndarray,
        camera_k: np.ndarray,
        camera_dist: Optional[np.ndarray],
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """PnP 求解"""
        try:
            import cv2
        except ImportError as e:
            _LOG.error("[MIAS-LCEC] PnP 失败 — cv2 不可用: %s", e)
            return None

        if camera_dist is None:
            camera_dist = np.zeros(5, dtype=np.float64)

        n_pts = len(obj_pts)
        _LOG.debug("[MIAS-LCEC] PnP 输入: obj_pts=%d img_pts=%d reproj_thresh=%.1f",
                   n_pts, len(img_pts), self.c3m_config.ransac_reproj_threshold)

        try:
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj_pts.astype(np.float32),
                img_pts.astype(np.float32),
                camera_k.astype(np.float32),
                camera_dist.astype(np.float32),
                reprojectionError=self.c3m_config.ransac_reproj_threshold,
                confidence=self.c3m_config.ransac_confidence,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if not ok or inliers is None or len(inliers) < 6:
                _LOG.info("[MIAS-LCEC] PnP RANSAC 内点不足(%s)，回退 solvePnP 无 RANSAC",
                          len(inliers) if inliers is not None else "None")
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts.astype(np.float32),
                    img_pts.astype(np.float32),
                    camera_k.astype(np.float32),
                    camera_dist.astype(np.float32),
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if not ok:
                    _LOG.warning("[MIAS-LCEC] PnP solvePnP 回退仍失败")
                    return None
                inliers = np.arange(len(obj_pts))

            R, _ = cv2.Rodrigues(rvec)
            t = tvec.ravel().astype(np.float64)
            n_in = len(inliers)
            _LOG.info("[MIAS-LCEC] PnP 成功 内点数=%d/%d t=[%.4f, %.4f, %.4f]", n_in, n_pts, t[0], t[1], t[2])
            return R, t

        except Exception as e:
            import traceback
            _LOG.error("[MIAS-LCEC] PnP 求解异常: %s", e)
            _LOG.debug("[MIAS-LCEC] PnP traceback:\n%s", traceback.format_exc())
            return None


def _read_pcd_ascii(path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """读取 ASCII PCD 文件，若有第 4 列则作为强度返回。返回 (points Nx3, intensities N 或 None)。"""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    n_points = 0
    data_start = 0
    for i, line in enumerate(lines):
        line = line.strip().lower()
        if line.startswith("points"):
            n_points = int(line.split()[-1])
        elif line.startswith("data"):
            data_start = i + 1
            break

    points = []
    intensities = []
    has_intensity = False
    for i in range(data_start, min(data_start + n_points, len(lines))):
        parts = lines[i].split()
        if len(parts) >= 3:
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            if len(parts) >= 4:
                intensities.append(float(parts[3]))
                has_intensity = True

    pts_arr = np.array(points, dtype=np.float64) if points else np.zeros((0, 3), dtype=np.float64)
    int_arr = np.array(intensities, dtype=np.float64) if has_intensity and len(intensities) == len(points) else None
    return pts_arr, int_arr


def estimate_extrinsic_from_files(
    pcd_path: str,
    image_path: str,
    sensor_config_path: str,
    model_path: str,
    mobilesam_path: Optional[str] = None,
    device: str = "cuda",
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    便捷函数: 从文件路径估计外参

    Args:
        pcd_path: PCD 文件路径
        image_path: 图像文件路径
        sensor_config_path: sensor_config.yaml 路径
        model_path: Overlap Transformer 权重路径
        mobilesam_path: MobileSAM 权重路径 (可选)
        device: 计算设备

    Returns:
        (R, t): 旋转矩阵和平移向量
        None: 估计失败
    """
    import yaml

    # 读取相机内参
    with open(sensor_config_path, "r") as f:
        cfg = yaml.safe_load(f)

    camera_cfg = cfg.get("camera", {})
    fx = camera_cfg.get("fx", 0)
    fy = camera_cfg.get("fy", 0)
    cx = camera_cfg.get("cx", 0)
    cy = camera_cfg.get("cy", 0)

    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ], dtype=np.float64)

    dist = camera_cfg.get("dist_coeffs")
    if dist is not None:
        dist = np.asarray(dist, dtype=np.float64)

    # 创建粗标定器
    calib = MIASLCECCoarseCalib(
        overlap_ckpt=model_path,
        mobilesam_ckpt=mobilesam_path,
        device=device,
    )

    # 估计外参
    return calib.estimate_extrinsic(pcd_path, image_path, K, dist)
