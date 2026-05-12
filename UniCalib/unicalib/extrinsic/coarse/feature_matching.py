"""
Camera-Camera 外参粗标定
基于特征匹配 + 本质矩阵分解 + RANSAC
支持普通相机和鱼眼相机
"""
from __future__ import annotations
import logging
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from ...core.calib_result import CalibResult, CameraIntrinsic, FisheyeIntrinsic
from ...core.sensor_config import SensorConfig, SensorType
from ...core.data_manager import DataManager

logger = logging.getLogger(__name__)


class FeatureMatchingCalibrator:
    """
    Camera-Camera 外参粗标定器。

    流程:
      1. 从同步图像对中提取特征点 (SIFT / ORB)
      2. 特征匹配 + RANSAC 过滤
      3. 基于本质矩阵分解 R, t
      4. 多帧结果鲁棒集成

    注意: 仅双目可恢复尺度，纯旋转相机对需 LiDAR 约束才能获得平移尺度。
    """

    def __init__(self, system_config: dict):
        cfg = (system_config.get("calibration", {})
               .get("stage2_coarse", {})
               .get("camera_camera", {}))
        self.feature_type = cfg.get("feature_matcher", "sift")  # sift / orb
        self.min_matches = cfg.get("min_matches", 30)
        self.max_frames = cfg.get("max_frames", 50)

    def calibrate(
        self,
        data_mgr: DataManager,
        sensor_a: SensorConfig,
        sensor_b: SensorConfig,
        intrinsics: Dict,
    ) -> CalibResult:
        """执行 Camera-Camera 外参粗标定。"""

        intr_a = intrinsics.get(sensor_a.sensor_id)
        intr_b = intrinsics.get(sensor_b.sensor_id)

        if intr_a is None or intr_b is None:
            logger.warning(f"Missing intrinsics for camera pair "
                           f"{sensor_a.sensor_id}-{sensor_b.sensor_id}")
            return self._identity_result(sensor_a, sensor_b)

        # 收集同步帧对
        frame_pairs = list(data_mgr.iter_synced_frames(
            sensor_a.topic, sensor_b.topic,
            sync_threshold_ms=100.0,
            max_pairs=self.max_frames
        ))

        if len(frame_pairs) < 3:
            logger.warning(f"  Insufficient synced frames ({len(frame_pairs)}) "
                           f"for {sensor_a.sensor_id}-{sensor_b.sensor_id}")
            return self._identity_result(sensor_a, sensor_b)

        logger.info(f"  Feature matching on {len(frame_pairs)} frame pairs...")

        # 提取匹配
        all_pts_a, all_pts_b = [], []
        for _, img_a, img_b in frame_pairs:
            pts_a, pts_b = self._match_features(
                img_a, img_b, intr_a, intr_b,
                sensor_a.is_fisheye(), sensor_b.is_fisheye()
            )
            if pts_a is not None and len(pts_a) >= self.min_matches:
                all_pts_a.append(pts_a)
                all_pts_b.append(pts_b)

        if not all_pts_a:
            logger.warning("  No valid matches found.")
            return self._identity_result(sensor_a, sensor_b)

        pts_a_all = np.vstack(all_pts_a)
        pts_b_all = np.vstack(all_pts_b)
        logger.info(f"  Total matched points: {len(pts_a_all)}")

        # 估计本质矩阵并分解
        K_a = self._get_pinhole_K(intr_a)
        K_b = self._get_pinhole_K(intr_b)

        E, mask_E = cv2.findEssentialMat(
            pts_a_all, pts_b_all, K_a,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        if E is None:
            return self._identity_result(sensor_a, sensor_b)

        if len(pts_a_all) == 0:
            return self._identity_result(sensor_a, sensor_b)

        _, R, t, mask_pose = cv2.recoverPose(
            E, pts_a_all, pts_b_all, K_a, mask=mask_E)

        inlier_ratio = float(np.sum(mask_pose > 0)) / len(pts_a_all)
        logger.info(f"  [{sensor_a.sensor_id}-{sensor_b.sensor_id}] "
                    f"inliers={inlier_ratio:.1%}  "
                    f"R_euler={Rotation.from_matrix(R).as_euler('zyx', degrees=True)}")

        return CalibResult(
            pair=(sensor_a.sensor_id, sensor_b.sensor_id),
            rotation=R,
            translation=t.flatten(),
            confidence=inlier_ratio,
            method_used=f"feature_matching_{self.feature_type}",
        )

    # ------------------------------------------------------------------
    # 特征提取与匹配
    # ------------------------------------------------------------------

    def _match_features(
        self,
        img_a: np.ndarray, img_b: np.ndarray,
        intr_a, intr_b,
        fisheye_a: bool, fisheye_b: bool,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """提取特征点并匹配，返回对应点对。"""

        # 先去畸变
        gray_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
        gray_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY)

        gray_a = self._undistort(gray_a, intr_a, fisheye_a)
        gray_b = self._undistort(gray_b, intr_b, fisheye_b)

        # 提取关键点
        detector = self._get_detector()
        kp_a, desc_a = detector.detectAndCompute(gray_a, None)
        kp_b, desc_b = detector.detectAndCompute(gray_b, None)

        if desc_a is None or desc_b is None or len(kp_a) < 10 or len(kp_b) < 10:
            return None, None

        # 匹配
        matches = self._match(desc_a, desc_b)
        if len(matches) < self.min_matches:
            return None, None

        pts_a = np.float32([kp_a[m.queryIdx].pt for m in matches])
        pts_b = np.float32([kp_b[m.trainIdx].pt for m in matches])

        return pts_a, pts_b

    def _get_detector(self):
        """创建特征检测器。"""
        if self.feature_type == "sift":
            try:
                return cv2.SIFT_create(nfeatures=500)
            except AttributeError:
                return cv2.xfeatures2d.SIFT_create(500)
        elif self.feature_type == "orb":
            return cv2.ORB_create(nfeatures=500)
        else:
            return cv2.SIFT_create(nfeatures=500)

    def _match(self, desc_a, desc_b) -> List:
        """BFMatcher + Lowe 比率测试。"""
        norm = cv2.NORM_L2 if self.feature_type == "sift" else cv2.NORM_HAMMING
        matcher = cv2.BFMatcher(norm, crossCheck=False)
        raw = matcher.knnMatch(desc_a, desc_b, k=2)
        good = []
        for pair in raw:
            if len(pair) == 2:
                m, n = pair
                if m.distance < 0.75 * n.distance:
                    good.append(m)
        return good

    def _undistort(self, gray: np.ndarray, intrinsic, is_fisheye: bool) -> np.ndarray:
        """对图像进行去畸变。"""
        if isinstance(intrinsic, CameraIntrinsic):
            K = intrinsic.K
            D = intrinsic.dist_coeffs
            if is_fisheye:
                h, w = gray.shape[:2]
                K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, D[:4].reshape(4, 1), (w, h), np.eye(3))
                gray = cv2.fisheye.undistortImage(gray, K, D[:4].reshape(4, 1), K_new)
            else:
                gray = cv2.undistort(gray, K, D)
        return gray

    def _get_pinhole_K(self, intrinsic) -> np.ndarray:
        """从任意内参类型获取 3x3 K 矩阵。"""
        if isinstance(intrinsic, CameraIntrinsic):
            return intrinsic.K
        elif isinstance(intrinsic, FisheyeIntrinsic):
            p = intrinsic.params
            return np.array([
                [p.get("fx", 500), 0, p.get("cx", 320)],
                [0, p.get("fy", 500), p.get("cy", 240)],
                [0, 0, 1],
            ], dtype=np.float64)
        else:
            return np.eye(3) * 500.0

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    def _identity_result(self, sa: SensorConfig, sb: SensorConfig) -> CalibResult:
        return CalibResult(
            pair=(sa.sensor_id, sb.sensor_id),
            rotation=np.eye(3),
            translation=np.zeros(3),
            confidence=0.0,
            method_used="identity_fallback",
        )
