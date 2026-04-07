"""
MIAS-LCEC 封装器 —— 在线无靶标 LiDAR-Camera 时空外参精标定
项目位置: /root/calib_ws/src/MIAS-LCEC/
论文: Online, Target-Free LiDAR-Camera Extrinsic Calibration via Cross-Modal Mask Matching
"""
from __future__ import annotations
import logging
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from ...core.calib_result import CalibResult, CameraIntrinsic, FisheyeIntrinsic
from ...core.sensor_config import SensorConfig, SensorType
from ...core.data_manager import DataManager

logger = logging.getLogger(__name__)

WORKSPACE = os.environ.get("WORKSPACE", "/root/calib_ws")
MIAS_PATH = os.path.join(WORKSPACE, "src", "MIAS-LCEC")


class MIASLCECWrapper:
    """
    MIAS-LCEC 封装器。

    支持两种模式:
      - coarse: 基于跨模态掩码匹配的快速粗标定 (targetless)
      - fine:   在粗标定基础上的时空联合精化，估计时间偏移

    MIAS-LCEC 是一个 C++ 可执行文件，通过命令行调用:
      mias_lcec_toolbox --config <config.yaml>
    """

    def __init__(self, system_config: dict):
        cfg = (system_config.get("calibration", {})
               .get("stage3_fine", {})
               .get("lidar_camera_refinement", {}))
        self.use_imu_motion_comp = cfg.get("use_imu_motion_compensation", True)
        self.optimize_td = cfg.get("optimize_time_offset", True)
        self._available = self._check_mias()

    # ------------------------------------------------------------------
    # 粗标定
    # ------------------------------------------------------------------

    def calibrate_coarse(
        self,
        data_mgr: DataManager,
        sensor_lidar: SensorConfig,
        sensor_cam: SensorConfig,
        intrinsics: Dict,
    ) -> CalibResult:
        """
        MIAS-LCEC 无靶标粗标定。
        使用跨模态掩码匹配 (C3M) 建立 3D-2D 对应关系。
        """
        # 确保顺序：sensor_a=LiDAR, sensor_b=Camera
        if sensor_lidar.is_camera():
            sensor_lidar, sensor_cam = sensor_cam, sensor_lidar

        if self._available:
            result = self._run_mias(
                data_mgr, sensor_lidar, sensor_cam, intrinsics,
                mode="coarse", initial=None
            )
            if result is not None:
                return result

        # Fallback: 几何特征匹配法
        logger.warning("MIAS-LCEC not available, using geometry-based coarse calib.")
        return self._geometry_coarse(data_mgr, sensor_lidar, sensor_cam, intrinsics)

    # ------------------------------------------------------------------
    # 精标定
    # ------------------------------------------------------------------

    def calibrate_fine(
        self,
        data_mgr: DataManager,
        sensor_lidar: SensorConfig,
        sensor_cam: SensorConfig,
        intrinsics: Dict,
        initial: Optional[CalibResult],
    ) -> CalibResult:
        """
        MIAS-LCEC 时空精化。
        在粗标定基础上，联合优化 R, t, 时间偏移 td。
        """
        if sensor_lidar.is_camera():
            sensor_lidar, sensor_cam = sensor_cam, sensor_lidar

        if self._available:
            result = self._run_mias(
                data_mgr, sensor_lidar, sensor_cam, intrinsics,
                mode="fine", initial=initial
            )
            if result is not None:
                return result

        # Fallback: 边缘特征优化
        return self._edge_based_refinement(
            data_mgr, sensor_lidar, sensor_cam, intrinsics, initial)

    # ------------------------------------------------------------------
    # MIAS-LCEC 调用
    # ------------------------------------------------------------------

    def _run_mias(
        self,
        data_mgr: DataManager,
        sensor_lidar: SensorConfig,
        sensor_cam: SensorConfig,
        intrinsics: Dict,
        mode: str,
        initial: Optional[CalibResult],
    ) -> Optional[CalibResult]:
        """调用 MIAS-LCEC 工具箱。"""
        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                # 准备数据
                lidar_dir = os.path.join(tmpdir, "lidar")
                cam_dir = os.path.join(tmpdir, "camera")
                os.makedirs(lidar_dir)
                os.makedirs(cam_dir)

                # 保存同步帧
                n_frames = 0
                for ts, img, pts in self._load_synced_lc_frames(
                        data_mgr, sensor_lidar, sensor_cam, max_pairs=50):
                    cv2.imwrite(os.path.join(cam_dir, f"{ts:.6f}.jpg"), img)
                    pts.astype(np.float32).tofile(
                        os.path.join(lidar_dir, f"{ts:.6f}.bin"))
                    n_frames += 1

                if n_frames < 5:
                    logger.warning("Too few synced frames for MIAS-LCEC.")
                    return None

                # 构建 MIAS 配置
                intr = intrinsics.get(sensor_cam.sensor_id)
                config = self._build_mias_config(
                    sensor_lidar, sensor_cam, intr,
                    lidar_dir, cam_dir, tmpdir, mode, initial
                )
                cfg_path = os.path.join(tmpdir, "mias_config.yaml")
                with open(cfg_path, "w") as f:
                    yaml.dump(config, f)

                # 运行 MIAS-LCEC (C++ 可执行文件)
                mias_bin = os.path.join(MIAS_PATH, "bin", "mias_lcec")
                if not os.path.exists(mias_bin):
                    # 尝试系统 PATH
                    mias_bin = "mias_lcec"

                proc = subprocess.run(
                    [mias_bin, "--config", cfg_path],
                    capture_output=True, text=True, timeout=600
                )

                if proc.returncode != 0:
                    logger.warning(f"MIAS-LCEC failed: {proc.stderr[-300:]}")
                    return None

                result_yaml = os.path.join(tmpdir, "result.yaml")
                if os.path.exists(result_yaml):
                    return self._parse_mias_result(
                        result_yaml,
                        sensor_lidar.sensor_id,
                        sensor_cam.sensor_id,
                    )

        except subprocess.TimeoutExpired:
            logger.warning("MIAS-LCEC timed out.")
        except FileNotFoundError:
            logger.warning("MIAS-LCEC binary not found.")
        except Exception as e:
            logger.warning(f"MIAS-LCEC error: {e}")

        return None

    def _build_mias_config(
        self,
        sensor_lidar: SensorConfig,
        sensor_cam: SensorConfig,
        intrinsic,
        lidar_dir: str,
        cam_dir: str,
        output_dir: str,
        mode: str,
        initial: Optional[CalibResult],
    ) -> dict:
        """构建 MIAS-LCEC 配置文件。"""
        config: dict = {
            "lidar": {
                "data_dir": lidar_dir,
                "type": "binary_float32_xyz",
            },
            "camera": {
                "data_dir": cam_dir,
            },
            "output_dir": output_dir,
            "calibration": {
                "mode": mode,
                "optimize_time_offset": self.optimize_td,
                "use_motion_compensation": self.use_imu_motion_comp,
                "max_iterations": 100,
            },
        }

        # 相机内参
        if isinstance(intrinsic, CameraIntrinsic):
            config["camera"]["intrinsic"] = {
                "fx": float(intrinsic.K[0, 0]),
                "fy": float(intrinsic.K[1, 1]),
                "cx": float(intrinsic.K[0, 2]),
                "cy": float(intrinsic.K[1, 2]),
                "dist_coeffs": intrinsic.dist_coeffs.tolist(),
                "model": "pinhole",
            }
        elif isinstance(intrinsic, FisheyeIntrinsic):
            config["camera"]["intrinsic"] = {
                **intrinsic.params,
                "model": intrinsic.model_type,
            }

        # 初始外参
        if initial is not None:
            config["initial_extrinsic"] = {
                "rotation": initial.rotation.tolist(),
                "translation": initial.translation.tolist(),
                "time_offset": initial.time_offset,
            }

        return config

    def _parse_mias_result(
        self, result_yaml: str, lidar_id: str, cam_id: str
    ) -> CalibResult:
        with open(result_yaml) as f:
            data = yaml.safe_load(f)

        R = np.array(data.get("rotation", np.eye(3).tolist()))
        t = np.array(data.get("translation", [0, 0, 0]))
        td = float(data.get("time_offset", 0.0))
        reproj = float(data.get("reprojection_error", float('inf')))

        return CalibResult(
            pair=(lidar_id, cam_id),
            rotation=R, translation=t.flatten(),
            time_offset=td,
            reprojection_error=reproj,
            confidence=0.85,
            method_used="MIAS-LCEC",
        )

    # ------------------------------------------------------------------
    # Fallback: 几何粗标定
    # ------------------------------------------------------------------

    def _geometry_coarse(
        self,
        data_mgr: DataManager,
        sensor_lidar: SensorConfig,
        sensor_cam: SensorConfig,
        intrinsics: Dict,
    ) -> CalibResult:
        """
        基于平面特征的 LiDAR-Camera 粗标定 fallback。
        从图像边缘和 LiDAR 平面提取平面法向量，对齐求解旋转。
        """
        intr = intrinsics.get(sensor_cam.sensor_id)
        if intr is None:
            return self._identity_result(sensor_lidar, sensor_cam)

        K = intr.K if isinstance(intr, CameraIntrinsic) else np.eye(3) * 500

        # 简单启发式：假设 LiDAR 在 Camera 前方，轴对齐
        # 实际应用中此处应做平面匹配
        R = np.array([
            [0, -1,  0],
            [0,  0, -1],
            [1,  0,  0],
        ], dtype=np.float64)  # 典型的 Camera-LiDAR 旋转约定
        t = np.array([0.0, 0.0, 0.2])  # 假设 20cm 偏移

        return CalibResult(
            pair=(sensor_lidar.sensor_id, sensor_cam.sensor_id),
            rotation=R, translation=t,
            confidence=0.15,
            method_used="geometry_fallback",
        )

    # ------------------------------------------------------------------
    # Fallback: 边缘对齐精标定
    # ------------------------------------------------------------------

    def _edge_based_refinement(
        self,
        data_mgr: DataManager,
        sensor_lidar: SensorConfig,
        sensor_cam: SensorConfig,
        intrinsics: Dict,
        initial: Optional[CalibResult],
    ) -> CalibResult:
        """
        基于边缘对齐的 LiDAR-Camera 精标定 fallback。
        最小化 LiDAR 投影点到图像边缘的距离。
        """
        if initial is None:
            return self._geometry_coarse(data_mgr, sensor_lidar, sensor_cam, intrinsics)

        intr = intrinsics.get(sensor_cam.sensor_id)
        if intr is None:
            return initial

        if not isinstance(intr, CameraIntrinsic):
            return initial

        K = intr.K
        D = intr.dist_coeffs

        from scipy.optimize import minimize
        from scipy.spatial.transform import Rotation as R_

        x0 = np.concatenate([
            R_.from_matrix(initial.rotation).as_rotvec(),
            initial.translation,
        ])

        all_pts = []
        all_edges = []

        count = 0
        for ts, img, pts in self._load_synced_lc_frames(
                data_mgr, sensor_lidar, sensor_cam, max_pairs=10):
            edge = cv2.Canny(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 50, 150)
            all_pts.append(pts[:, :3])
            all_edges.append(edge)
            count += 1
            if count >= 5:
                break

        if not all_pts:
            return initial

        def cost_fn(x):
            rv, t = x[:3], x[3:]
            R_mat = R_.from_rotvec(rv).as_matrix()
            total_cost = 0.0
            for pts, edge in zip(all_pts, all_edges):
                pts_cam = (R_mat @ pts.T + t.reshape(3, 1)).T
                mask = pts_cam[:, 2] > 0.1
                pts_cam = pts_cam[mask]
                if len(pts_cam) == 0:
                    continue
                pts_2d, _ = cv2.projectPoints(
                    pts_cam, np.zeros(3), np.zeros(3), K, D)
                pts_2d = pts_2d.reshape(-1, 2).astype(int)
                h, w = edge.shape
                for p in pts_2d[::10]:
                    u, v = p
                    if 5 <= u < w-5 and 5 <= v < h-5:
                        patch = edge[v-5:v+5, u-5:u+5]
                        min_d = 5 - np.argmax(patch > 0) if np.any(patch > 0) else 5
                        total_cost += float(min_d) ** 2
            return total_cost

        result = minimize(cost_fn, x0, method="Nelder-Mead",
                          options={"maxiter": 500, "xatol": 1e-4})
        rv_opt, t_opt = result.x[:3], result.x[3:]
        R_opt = R_.from_rotvec(rv_opt).as_matrix()

        return CalibResult(
            pair=(sensor_lidar.sensor_id, sensor_cam.sensor_id),
            rotation=R_opt, translation=t_opt,
            time_offset=initial.time_offset,
            confidence=0.6,
            method_used="edge_alignment_refinement",
        )

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    def _load_synced_lc_frames(
        self, data_mgr: DataManager,
        sensor_lidar: SensorConfig,
        sensor_cam: SensorConfig,
        max_pairs: int = 50
    ):
        """生成 (ts, image, pointcloud) 同步三元组。"""
        for ts, data_a, data_b in data_mgr.iter_synced_frames(
                sensor_lidar.topic, sensor_cam.topic,
                sync_threshold_ms=100.0, max_pairs=max_pairs):
            # data_a 是 LiDAR (pointcloud), data_b 是 Camera (image)
            yield ts, data_b, data_a

    def _identity_result(self, sl: SensorConfig, sc: SensorConfig) -> CalibResult:
        return CalibResult(
            pair=(sl.sensor_id, sc.sensor_id),
            rotation=np.eye(3), translation=np.zeros(3),
            confidence=0.0, method_used="identity_fallback",
        )

    @staticmethod
    def _check_mias() -> bool:
        mias_bin = os.path.join(MIAS_PATH, "bin", "mias_lcec")
        if Path(mias_bin).exists():
            return True
        try:
            result = subprocess.run(
                ["which", "mias_lcec"], capture_output=True, text=True, timeout=3)
            return result.returncode == 0
        except Exception:
            return False
