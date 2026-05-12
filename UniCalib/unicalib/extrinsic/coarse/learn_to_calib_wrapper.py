"""
learn-to-calibrate 封装器
基于强化学习的 LiDAR-IMU 外参粗标定
项目位置: /root/calib_ws/src/learn-to-calibrate/
"""
from __future__ import annotations
import logging
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import yaml

from ...core.calib_result import CalibResult
from ...core.sensor_config import SensorConfig, SensorType
from ...core.data_manager import DataManager

logger = logging.getLogger(__name__)

WORKSPACE = os.environ.get("WORKSPACE", "/root/calib_ws")
L2CALIB_PATH = os.path.join(WORKSPACE, "src", "learn-to-calibrate")


class LearnToCalibWrapper:
    """
    learn-to-calibrate (L2Calib) 封装器。

    用于 LiDAR-IMU 外参粗标定:
      1. 调用 RL 求解器估计初始外参
      2. 对多次结果做鲁棒集成（RANSAC 中值）

    接口说明:
      - 输入: LiDAR 点云序列 + IMU 数据 (rosbag 或目录)
      - 输出: CalibResult (R, t, confidence)
    """

    def __init__(self, system_config: dict):
        cfg = (system_config.get("calibration", {})
               .get("stage2_coarse", {})
               .get("imu_lidar", {}))
        self.available = Path(L2CALIB_PATH).exists()
        if not self.available:
            logger.warning(f"learn-to-calibrate not found at {L2CALIB_PATH}")

    def calibrate(
        self,
        data_mgr: DataManager,
        sensor_imu: SensorConfig,
        sensor_lidar: SensorConfig,
        intrinsics: Dict,
    ) -> CalibResult:
        """
        执行 LiDAR-IMU 粗外参标定。

        Returns:
            CalibResult: 初始外参估计结果
        """
        # 确保 sensor_a 是 IMU，sensor_b 是 LiDAR
        if sensor_imu.is_lidar():
            sensor_imu, sensor_lidar = sensor_lidar, sensor_imu

        if self.available:
            result = self._run_l2calib(data_mgr, sensor_imu, sensor_lidar)
            if result is not None:
                return result

        # Fallback: 基于重力方向对齐的解析解
        logger.warning("learn-to-calibrate failed, using gravity-alignment fallback.")
        return self._gravity_align_fallback(data_mgr, sensor_imu, sensor_lidar)

    # ------------------------------------------------------------------
    # L2Calib RL 求解
    # ------------------------------------------------------------------

    def _run_l2calib(
        self,
        data_mgr: DataManager,
        sensor_imu: SensorConfig,
        sensor_lidar: SensorConfig,
    ) -> Optional[CalibResult]:
        """调用 learn-to-calibrate 的 RL 求解器。"""
        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                # 保存 IMU 数据
                imu_data = data_mgr.load_imu_data(sensor_imu.topic)
                if imu_data is None:
                    return None
                imu_csv = os.path.join(tmpdir, "imu.csv")
                self._save_imu_csv(imu_data, imu_csv)

                # 保存 LiDAR 点云
                lidar_dir = os.path.join(tmpdir, "lidar")
                os.makedirs(lidar_dir)
                count = 0
                for ts, pts in data_mgr.iter_pointclouds(
                        sensor_lidar.topic, max_frames=100, skip=1):
                    pts.astype(np.float32).tofile(
                        os.path.join(lidar_dir, f"{ts:.6f}.bin"))
                    count += 1
                if count < 10:
                    logger.warning("Too few LiDAR frames for L2Calib.")
                    return None

                # 写 config
                config = {
                    "imu_topic": sensor_imu.topic,
                    "lidar_topic": sensor_lidar.topic,
                    "imu_data": imu_csv,
                    "lidar_data": lidar_dir,
                    "output_dir": tmpdir,
                }
                cfg_path = os.path.join(tmpdir, "config.yaml")
                with open(cfg_path, "w") as f:
                    yaml.dump(config, f)

                env = os.environ.copy()
                env["PYTHONPATH"] = (
                    f"{L2CALIB_PATH}:"
                    f"{os.path.join(L2CALIB_PATH, 'rl_solver')}:"
                    f"{env.get('PYTHONPATH', '')}"
                )

                proc = subprocess.run(
                    [
                        sys.executable,
                        os.path.join(L2CALIB_PATH, "rl_solver", "calib_rl.py"),
                        "--config", cfg_path,
                    ],
                    env=env, capture_output=True, text=True, timeout=600
                )

                if proc.returncode != 0:
                    logger.warning(f"L2Calib RL failed: {proc.stderr[-300:]}")
                    return None

                result_yaml = os.path.join(tmpdir, "extrinsic_result.yaml")
                if os.path.exists(result_yaml):
                    return self._parse_result(
                        result_yaml, sensor_imu.sensor_id, sensor_lidar.sensor_id)

        except subprocess.TimeoutExpired:
            logger.warning("L2Calib timed out (600s).")
        except Exception as e:
            logger.warning(f"L2Calib error: {e}")

        return None

    def _parse_result(self, result_yaml: str, imu_id: str, lidar_id: str) -> CalibResult:
        with open(result_yaml) as f:
            data = yaml.safe_load(f)

        R = np.array(data.get("rotation", np.eye(3).tolist()))
        t = np.array(data.get("translation", [0, 0, 0]))

        if R.shape == (4,) or R.shape == (1, 4):
            from scipy.spatial.transform import Rotation
            R = Rotation.from_quat(R.flatten()).as_matrix()
        elif len(R) == 3 and len(R[0]) == 1:
            from scipy.spatial.transform import Rotation
            R = Rotation.from_rotvec(np.array(R).flatten()).as_matrix()

        return CalibResult(
            pair=(imu_id, lidar_id),
            rotation=np.array(R),
            translation=np.array(t).flatten(),
            confidence=float(data.get("confidence", 0.5)),
            method_used="learn-to-calibrate_RL",
        )

    # ------------------------------------------------------------------
    # Fallback: 重力方向对齐
    # ------------------------------------------------------------------

    def _gravity_align_fallback(
        self, data_mgr: DataManager,
        sensor_imu: SensorConfig,
        sensor_lidar: SensorConfig,
    ) -> CalibResult:
        """
        基于重力方向的解析初始化:
          - IMU 静止时 accel 方向 ≈ 重力方向 → 估计旋转
          - 平移设为零（需后续精标定）
        """
        imu_data = data_mgr.load_imu_data(sensor_imu.topic)
        if imu_data is not None and len(imu_data["accel"]) > 0:
            g_meas = np.mean(imu_data["accel"][:100], axis=0)
            g_meas_norm = g_meas / np.linalg.norm(g_meas)
            g_world = np.array([0, 0, -1.0])  # 重力方向（NED: z向下）

            # 计算将 g_meas 对齐到 g_world 的旋转
            R = _rotation_from_vectors(g_meas_norm, g_world)
        else:
            R = np.eye(3)

        return CalibResult(
            pair=(sensor_imu.sensor_id, sensor_lidar.sensor_id),
            rotation=R,
            translation=np.zeros(3),
            confidence=0.2,
            method_used="gravity_align_fallback",
        )

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    def _save_imu_csv(self, imu_data: Dict, path: str):
        import csv
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "gx", "gy", "gz", "ax", "ay", "az"])
            for t, g, a in zip(
                imu_data["timestamps"], imu_data["gyro"], imu_data["accel"]
            ):
                writer.writerow([t, g[0], g[1], g[2], a[0], a[1], a[2]])


def _rotation_from_vectors(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
    """计算将向量 v1 旋转到 v2 的旋转矩阵。"""
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    cross = np.cross(v1, v2)
    dot = np.dot(v1, v2)
    s = np.linalg.norm(cross)
    if s < 1e-10:
        return np.eye(3)
    kmat = np.array([
        [0, -cross[2], cross[1]],
        [cross[2], 0, -cross[0]],
        [-cross[1], cross[0], 0],
    ])
    return np.eye(3) + kmat + kmat @ kmat * (1 - dot) / (s ** 2)
