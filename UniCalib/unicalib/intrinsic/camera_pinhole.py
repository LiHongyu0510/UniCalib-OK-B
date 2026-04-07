"""
普通相机内参标定模块
策略: DM-Calib 自动初估计 → OpenCV 棋盘格精标定 → 交叉验证
"""
from __future__ import annotations
import logging
import os
import sys
import json
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from ..core.calib_result import CameraIntrinsic
from ..core.sensor_config import SensorConfig
from ..core.data_manager import DataManager

logger = logging.getLogger(__name__)

WORKSPACE = os.environ.get("WORKSPACE", "/root/calib_ws")
DM_CALIB_PATH = os.path.join(WORKSPACE, "src", "DM-Calib")


class CameraPinholeCalibrator:
    """
    普通相机 (针孔模型) 内参标定器。

    两阶段策略:
      1. DM-Calib: 无需标定板，基于扩散模型自动初估计
      2. OpenCV:   使用棋盘格精化（如有标定板图像）
    """

    def __init__(self, system_config: dict):
        cfg = system_config.get("calibration", {}).get("stage1_intrinsic", {})
        cam_cfg = cfg.get("camera_pinhole", {})
        self.primary_method = cam_cfg.get("primary_method", "dm_calib")
        self.refinement_method = cam_cfg.get("refinement_method", "opencv")
        self.min_reproj_error = cam_cfg.get("min_reproj_error", 0.5)

        checker_cfg = system_config.get("data_collection", {}).get("camera_intrinsic", {}).get("checkerboard", {})
        self.checker_rows = checker_cfg.get("rows", 8)
        self.checker_cols = checker_cfg.get("cols", 11)
        self.checker_square_mm = checker_cfg.get("square_size", 0.03) * 1000  # → mm

    def calibrate(self, data_mgr: DataManager, sensor: SensorConfig) -> CameraIntrinsic:
        """执行相机内参标定，返回 CameraIntrinsic。"""

        # Step 1: 加载图像
        images = list(data_mgr.iter_images(sensor.topic, max_frames=200, skip=3))
        if not images:
            logger.warning(f"  No images found for sensor {sensor.sensor_id}, using prior.")
            return self._prior_or_default(sensor)

        imgs_np = [img for _, img in images]
        image_size = (imgs_np[0].shape[1], imgs_np[0].shape[0])  # (W, H)

        # Step 2: DM-Calib 自动初估计
        dm_result = None
        if self.primary_method == "dm_calib":
            dm_result = self._run_dm_calib(imgs_np, sensor.sensor_id, image_size)

        # Step 3: OpenCV 棋盘格精标定
        opencv_result = None
        if self.refinement_method == "opencv":
            opencv_result = self._opencv_checkerboard_calib(imgs_np, image_size, dm_result)

        # 选择最优结果
        if opencv_result and opencv_result.reprojection_error < self.min_reproj_error:
            final = opencv_result
            logger.info(f"  [{sensor.sensor_id}] OpenCV refined: "
                        f"reproj={final.reprojection_error:.3f}px  "
                        f"fx={final.K[0,0]:.1f} fy={final.K[1,1]:.1f}")
        elif dm_result:
            final = dm_result
            logger.info(f"  [{sensor.sensor_id}] DM-Calib only: "
                        f"fx={final.K[0,0]:.1f} fy={final.K[1,1]:.1f}")
        else:
            final = self._prior_or_default(sensor, image_size)
            logger.warning(f"  [{sensor.sensor_id}] Using default intrinsic.")

        return final

    # ------------------------------------------------------------------
    # DM-Calib 推理
    # ------------------------------------------------------------------

    def _run_dm_calib(self, images: List[np.ndarray],
                      sensor_id: str,
                      image_size: Tuple[int, int]) -> Optional[CameraIntrinsic]:
        """
        调用 DM-Calib 的推理接口。
        DM-Calib 项目位于: /root/calib_ws/src/DM-Calib/
        使用 DMCalib/tools/infer.py
        """
        if not Path(DM_CALIB_PATH).exists():
            logger.warning(f"DM-Calib not found at {DM_CALIB_PATH}, skipping.")
            return None

        try:
            # 将图像写入临时目录
            with tempfile.TemporaryDirectory() as tmpdir:
                img_dir = os.path.join(tmpdir, "images")
                os.makedirs(img_dir)
                out_json = os.path.join(tmpdir, "intrinsic.json")

                # 采样 10 张有代表性的图像
                step = max(1, len(images) // 10)
                for i, img in enumerate(images[::step]):
                    cv2.imwrite(os.path.join(img_dir, f"{i:06d}.jpg"), img)

                # 调用 DM-Calib 推理
                env = os.environ.copy()
                env["PYTHONPATH"] = f"{DM_CALIB_PATH}:{env.get('PYTHONPATH', '')}"

                proc = subprocess.run(
                    [
                        sys.executable,
                        os.path.join(DM_CALIB_PATH, "DMCalib", "tools", "infer.py"),
                        "--image_dir", img_dir,
                        "--model_dir", os.path.join(DM_CALIB_PATH, "model"),
                        "--output", out_json,
                    ],
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=300,
                )

                if proc.returncode != 0:
                    logger.warning(f"DM-Calib failed: {proc.stderr[-500:]}")
                    return None

                if not os.path.exists(out_json):
                    logger.warning("DM-Calib did not produce output JSON.")
                    return None

                with open(out_json) as f:
                    result = json.load(f)

                W, H = image_size
                fx = result.get("fx", W * 0.8)
                fy = result.get("fy", W * 0.8)
                cx = result.get("cx", W / 2.0)
                cy = result.get("cy", H / 2.0)
                k1 = result.get("k1", 0.0)
                k2 = result.get("k2", 0.0)
                p1 = result.get("p1", 0.0)
                p2 = result.get("p2", 0.0)
                k3 = result.get("k3", 0.0)

                K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
                dist = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

                return CameraIntrinsic(
                    K=K,
                    dist_coeffs=dist,
                    image_size=image_size,
                    reprojection_error=result.get("confidence", 1.0),
                    method="DM-Calib",
                )

        except subprocess.TimeoutExpired:
            logger.warning("DM-Calib timed out.")
        except Exception as e:
            logger.warning(f"DM-Calib error: {e}")

        return None

    # ------------------------------------------------------------------
    # OpenCV 棋盘格标定
    # ------------------------------------------------------------------

    def _opencv_checkerboard_calib(
        self, images: List[np.ndarray],
        image_size: Tuple[int, int],
        initial: Optional[CameraIntrinsic] = None,
    ) -> Optional[CameraIntrinsic]:
        """使用 OpenCV 棋盘格内参标定。"""

        pattern_size = (self.checker_cols - 1, self.checker_rows - 1)
        square_size = self.checker_square_mm  # mm

        # 构建世界坐标系中的棋盘角点
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        objp *= square_size

        objpoints, imgpoints = [], []

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if ret:
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners_refined)

        if len(objpoints) < 5:
            logger.warning(f"  Only {len(objpoints)} checkerboard images detected (need ≥5).")
            return None

        logger.info(f"  Checkerboard detected in {len(objpoints)} images.")

        # 使用 DM-Calib 结果作为初值
        flags = 0
        K_init = None
        dist_init = None
        if initial is not None:
            K_init = initial.K.copy()
            dist_init = initial.dist_coeffs.copy()
            flags = cv2.CALIB_USE_INTRINSIC_GUESS

        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size,
            K_init, dist_init,
            flags=flags,
        )

        if not ret:
            logger.warning("OpenCV calibrateCamera failed.")
            return None

        # 计算重投影误差
        errors = []
        for objp_, imgp_, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
            projected, _ = cv2.projectPoints(objp_, rvec, tvec, K, dist)
            err = cv2.norm(imgp_, projected, cv2.NORM_L2) / len(projected)
            errors.append(err)
        reproj_err = float(np.mean(errors))

        return CameraIntrinsic(
            K=K,
            dist_coeffs=dist.flatten(),
            image_size=image_size,
            reprojection_error=reproj_err,
            method="OpenCV_checkerboard",
        )

    # ------------------------------------------------------------------
    # 默认内参 (fallback)
    # ------------------------------------------------------------------

    def _prior_or_default(
        self, sensor: SensorConfig,
        image_size: Optional[Tuple[int, int]] = None
    ) -> CameraIntrinsic:
        """先验或默认内参。"""
        if sensor.intrinsic_prior:
            return CameraIntrinsic.from_dict(sensor.intrinsic_prior)

        W, H = image_size or (sensor.resolution or (1920, 1080))
        f = W * 0.8
        K = np.array([[f, 0, W/2], [0, f, H/2], [0, 0, 1]], dtype=np.float64)
        dist = np.zeros(5, dtype=np.float64)
        return CameraIntrinsic(K=K, dist_coeffs=dist, image_size=(W, H),
                               reprojection_error=float('inf'), method="default")
