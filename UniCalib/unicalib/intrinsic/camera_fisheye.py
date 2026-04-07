"""
鱼眼相机内参标定模块
支持多种鱼眼模型: EUCM / Double Sphere / Kannala-Brandt / Equidistant
基于 WoodScape / click_calib 的鱼眼标定方法
"""
from __future__ import annotations
import logging
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import minimize

from ..core.calib_result import FisheyeIntrinsic
from ..core.sensor_config import SensorConfig
from ..core.data_manager import DataManager

logger = logging.getLogger(__name__)


class CameraFisheyeCalibrator:
    """
    鱼眼相机内参标定器。

    策略: 依次尝试多种鱼眼模型，选择重投影误差最小的模型。
    支持的模型:
      - eucm          : Enhanced Unified Camera Model (α, β, fx, fy, cx, cy)
      - double_sphere : Double Sphere Model (ξ, α, fx, fy, cx, cy)
      - kannala_brandt: Kannala-Brandt (k1~k4, fx, fy, cx, cy)
      - equidistant   : OpenCV fisheye (k1~k4, fx, fy, cx, cy)
    """

    def __init__(self, system_config: dict):
        cfg = (system_config.get("calibration", {})
               .get("stage1_intrinsic", {})
               .get("camera_fisheye", {}))
        self.models_to_try: List[str] = cfg.get(
            "models_to_try", ["eucm", "double_sphere", "kannala_brandt", "equidistant"]
        )
        checker_cfg = (system_config.get("data_collection", {})
                       .get("camera_intrinsic", {})
                       .get("checkerboard", {}))
        self.checker_rows = checker_cfg.get("rows", 8)
        self.checker_cols = checker_cfg.get("cols", 11)
        self.checker_square = checker_cfg.get("square_size", 0.03)  # metres

    def calibrate(self, data_mgr: DataManager, sensor: SensorConfig) -> FisheyeIntrinsic:
        """尝试多种鱼眼模型，返回最优 FisheyeIntrinsic。"""

        images = [img for _, img in data_mgr.iter_images(sensor.topic, max_frames=100, skip=3)]
        if not images:
            logger.warning(f"  No images for fisheye sensor {sensor.sensor_id}.")
            return self._default(sensor)

        image_size = (images[0].shape[1], images[0].shape[0])

        # 先用 OpenCV 鱼眼模型提取棋盘格角点
        objpoints, imgpoints = self._detect_checkerboard(images)
        if len(objpoints) < 5:
            logger.warning(f"  [{sensor.sensor_id}] Not enough checkerboard detections "
                           f"({len(objpoints)}), using equidistant fallback.")
            return self._equidistant_calib(images, image_size) or self._default(sensor)

        best_result: Optional[FisheyeIntrinsic] = None
        best_error = float('inf')

        for model in self.models_to_try:
            try:
                result = self._fit_model(model, objpoints, imgpoints, image_size)
                if result and result.reprojection_error < best_error:
                    best_error = result.reprojection_error
                    best_result = result
                    logger.info(f"  [{sensor.sensor_id}] model={model} "
                                f"reproj={result.reprojection_error:.4f}px")
            except Exception as e:
                logger.warning(f"  [{sensor.sensor_id}] model={model} failed: {e}")

        if best_result is None:
            logger.warning(f"  [{sensor.sensor_id}] All models failed, using default.")
            return self._default(sensor, image_size)

        logger.info(f"  [{sensor.sensor_id}] Best model: {best_result.model_type} "
                    f"reproj={best_result.reprojection_error:.4f}px")
        return best_result

    # ------------------------------------------------------------------
    # 棋盘格角点检测
    # ------------------------------------------------------------------

    def _detect_checkerboard(
        self, images: List[np.ndarray]
    ) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """提取所有图像中的棋盘格角点。"""
        pattern = (self.checker_cols - 1, self.checker_rows - 1)
        objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
        objp *= self.checker_square

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-6)
        objpoints, imgpoints = [], []

        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern, None)
            if ret:
                refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(refined.reshape(-1, 2))

        return objpoints, imgpoints

    # ------------------------------------------------------------------
    # 模型拟合分发
    # ------------------------------------------------------------------

    def _fit_model(
        self, model: str,
        objpoints: List[np.ndarray],
        imgpoints: List[np.ndarray],
        image_size: Tuple[int, int],
    ) -> Optional[FisheyeIntrinsic]:
        if model == "equidistant":
            return self._fit_equidistant(objpoints, imgpoints, image_size)
        elif model == "kannala_brandt":
            return self._fit_kannala_brandt(objpoints, imgpoints, image_size)
        elif model == "eucm":
            return self._fit_eucm(objpoints, imgpoints, image_size)
        elif model == "double_sphere":
            return self._fit_double_sphere(objpoints, imgpoints, image_size)
        return None

    # ------------------------------------------------------------------
    # Equidistant (OpenCV fisheye)
    # ------------------------------------------------------------------

    def _fit_equidistant(
        self, objpoints, imgpoints, image_size
    ) -> Optional[FisheyeIntrinsic]:
        W, H = image_size
        K = np.eye(3, dtype=np.float64)
        K[0, 0] = K[1, 1] = max(W, H)
        K[0, 2] = W / 2.0
        K[1, 2] = H / 2.0
        D = np.zeros((4, 1), dtype=np.float64)

        obj3d = [o.reshape(-1, 1, 3) for o in objpoints]
        img2d = [i.reshape(-1, 1, 2) for i in imgpoints]

        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                 | cv2.fisheye.CALIB_FIX_SKEW)
        try:
            rms, K_out, D_out, rvecs, tvecs = cv2.fisheye.calibrate(
                obj3d, img2d, image_size, K, D, flags=flags)
        except cv2.error as e:
            logger.warning(f"equidistant calib error: {e}")
            return None

        params = {
            "fx": float(K_out[0, 0]), "fy": float(K_out[1, 1]),
            "cx": float(K_out[0, 2]), "cy": float(K_out[1, 2]),
            "k1": float(D_out[0]), "k2": float(D_out[1]),
            "k3": float(D_out[2]), "k4": float(D_out[3]),
        }
        return FisheyeIntrinsic(
            model_type="equidistant",
            params=params,
            image_size=image_size,
            reprojection_error=float(rms),
            method="OpenCV_fisheye",
        )

    # ------------------------------------------------------------------
    # Kannala-Brandt (等效 equidistant 扩展)
    # ------------------------------------------------------------------

    def _fit_kannala_brandt(
        self, objpoints, imgpoints, image_size
    ) -> Optional[FisheyeIntrinsic]:
        # Kannala-Brandt 与 equidistant 模型结构相同，使用 OpenCV fisheye
        result = self._fit_equidistant(objpoints, imgpoints, image_size)
        if result:
            result.model_type = "kannala_brandt"
        return result

    # ------------------------------------------------------------------
    # EUCM (Enhanced Unified Camera Model)
    # ------------------------------------------------------------------

    def _fit_eucm(
        self, objpoints, imgpoints, image_size
    ) -> Optional[FisheyeIntrinsic]:
        """
        EUCM 投影: u = fx * x / (α*d + (1-α)*z) + cx
                   d = √(β*(x²+y²) + z²)
        参数: α ∈ (0,1), β > 0, fx, fy, cx, cy
        """
        W, H = image_size
        # 用 equidistant 结果作为初值
        eq = self._fit_equidistant(objpoints, imgpoints, image_size)
        if eq:
            fx0 = eq.params["fx"]
            fy0 = eq.params["fy"]
            cx0 = eq.params["cx"]
            cy0 = eq.params["cy"]
        else:
            fx0 = fy0 = max(W, H) * 0.5
            cx0, cy0 = W / 2.0, H / 2.0

        # 初始参数: [α, β, fx, fy, cx, cy]
        x0 = np.array([0.5, 1.0, fx0, fy0, cx0, cy0])

        def residuals(x):
            alpha, beta, fx, fy, cx, cy = x
            if alpha <= 0 or alpha >= 1 or beta <= 0:
                return np.full(100, 1e6)
            errs = []
            for objp, imgp in zip(objpoints, imgpoints):
                proj = _project_eucm(objp, alpha, beta, fx, fy, cx, cy)
                if proj is not None:
                    errs.append((proj - imgp).ravel())
            if not errs:
                return np.full(100, 1e6)
            return np.concatenate(errs)

        result = minimize(
            lambda x: np.sum(residuals(x) ** 2),
            x0,
            method="Nelder-Mead",
            options={"maxiter": 5000, "xatol": 1e-4, "fatol": 1e-6},
        )
        alpha, beta, fx, fy, cx, cy = result.x
        rms = _compute_rms(objpoints, imgpoints,
                           lambda p: _project_eucm(p, alpha, beta, fx, fy, cx, cy))

        params = {"alpha": float(alpha), "beta": float(beta),
                  "fx": float(fx), "fy": float(fy),
                  "cx": float(cx), "cy": float(cy)}
        return FisheyeIntrinsic(
            model_type="eucm",
            params=params,
            image_size=image_size,
            reprojection_error=float(rms),
            method="EUCM_NelderMead",
        )

    # ------------------------------------------------------------------
    # Double Sphere Model
    # ------------------------------------------------------------------

    def _fit_double_sphere(
        self, objpoints, imgpoints, image_size
    ) -> Optional[FisheyeIntrinsic]:
        """
        DS 投影: w = α*d2 + (1-α)*(ξ*d1 + z)
                 u = fx * x/w + cx
                 d1 = ‖(x,y,z)‖, d2 = ‖(x,y,ξ*d1+z)‖
        参数: ξ, α ∈ (0,1), fx, fy, cx, cy
        """
        W, H = image_size
        eq = self._fit_equidistant(objpoints, imgpoints, image_size)
        if eq:
            fx0, fy0 = eq.params["fx"], eq.params["fy"]
            cx0, cy0 = eq.params["cx"], eq.params["cy"]
        else:
            fx0 = fy0 = max(W, H) * 0.5
            cx0, cy0 = W / 2.0, H / 2.0

        x0 = np.array([0.0, 0.5, fx0, fy0, cx0, cy0])

        def residuals(x):
            xi, alpha, fx, fy, cx, cy = x
            if alpha <= 0 or alpha >= 1:
                return np.full(100, 1e6)
            errs = []
            for objp, imgp in zip(objpoints, imgpoints):
                proj = _project_double_sphere(objp, xi, alpha, fx, fy, cx, cy)
                if proj is not None:
                    errs.append((proj - imgp).ravel())
            if not errs:
                return np.full(100, 1e6)
            return np.concatenate(errs)

        result = minimize(
            lambda x: np.sum(residuals(x) ** 2),
            x0,
            method="Nelder-Mead",
            options={"maxiter": 5000, "xatol": 1e-4, "fatol": 1e-6},
        )
        xi, alpha, fx, fy, cx, cy = result.x
        rms = _compute_rms(objpoints, imgpoints,
                           lambda p: _project_double_sphere(p, xi, alpha, fx, fy, cx, cy))

        params = {"xi": float(xi), "alpha": float(alpha),
                  "fx": float(fx), "fy": float(fy),
                  "cx": float(cx), "cy": float(cy)}
        return FisheyeIntrinsic(
            model_type="double_sphere",
            params=params,
            image_size=image_size,
            reprojection_error=float(rms),
            method="DS_NelderMead",
        )

    def _equidistant_calib(self, images, image_size):
        """仅有少量图像时的快速 equidistant 标定。"""
        objpoints, imgpoints = self._detect_checkerboard(images)
        if len(objpoints) < 3:
            return None
        return self._fit_equidistant(objpoints, imgpoints, image_size)

    def _default(self, sensor: SensorConfig,
                 image_size: Optional[Tuple[int, int]] = None) -> FisheyeIntrinsic:
        W, H = image_size or (sensor.resolution or (1280, 960))
        return FisheyeIntrinsic(
            model_type="equidistant",
            params={"fx": W * 0.4, "fy": W * 0.4,
                    "cx": W / 2.0, "cy": H / 2.0,
                    "k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0},
            image_size=(W, H),
            reprojection_error=float('inf'),
            method="default",
        )


# ------------------------------------------------------------------
# 投影辅助函数
# ------------------------------------------------------------------

def _project_eucm(
    points_3d: np.ndarray,
    alpha: float, beta: float,
    fx: float, fy: float, cx: float, cy: float,
) -> Optional[np.ndarray]:
    x, y, z = points_3d[:, 0], points_3d[:, 1], points_3d[:, 2]
    d = np.sqrt(np.maximum(beta * (x**2 + y**2) + z**2, 1e-10))
    denom = alpha * d + (1.0 - alpha) * z
    mask = denom > 1e-6
    if not np.all(mask):
        return None
    u = fx * x[mask] / denom[mask] + cx
    v = fy * y[mask] / denom[mask] + cy
    return np.stack([u, v], axis=-1)


def _project_double_sphere(
    points_3d: np.ndarray,
    xi: float, alpha: float,
    fx: float, fy: float, cx: float, cy: float,
) -> Optional[np.ndarray]:
    x, y, z = points_3d[:, 0], points_3d[:, 1], points_3d[:, 2]
    d1 = np.sqrt(x**2 + y**2 + z**2)
    inner = xi * d1 + z
    d2 = np.sqrt(x**2 + y**2 + inner**2)
    denom = alpha * d2 + (1.0 - alpha) * inner
    mask = denom > 1e-6
    if not np.all(mask):
        return None
    u = fx * x[mask] / denom[mask] + cx
    v = fy * y[mask] / denom[mask] + cy
    return np.stack([u, v], axis=-1)


def _compute_rms(objpoints, imgpoints, project_fn) -> float:
    errors = []
    for objp, imgp in zip(objpoints, imgpoints):
        proj = project_fn(objp)
        if proj is not None and len(proj) == len(imgp):
            errors.append(np.sqrt(np.mean((proj - imgp) ** 2)))
    return float(np.mean(errors)) if errors else float('inf')
