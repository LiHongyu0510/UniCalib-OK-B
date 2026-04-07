"""
IMU 内参标定模块
集成 iKalibr 连续时间 B-spline 方法 + Allan 方差分析
支持完整 12 参数模型:
  - 加速度计: Ma (非正交) + Sa (比例) + ba (零偏)
  - 陀螺仪:   Mg (非正交) + Sg (比例) + bg (零偏)
  - g-sensitivity: Tg
"""
from __future__ import annotations
import logging
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import yaml

from ..core.calib_result import IMUIntrinsic
from ..core.sensor_config import SensorConfig
from ..core.data_manager import DataManager
from .allan_variance import AllanVarianceAnalyzer

logger = logging.getLogger(__name__)

WORKSPACE = os.environ.get("WORKSPACE", "/root/calib_ws")
IKALIBR_PATH = os.path.join(WORKSPACE, "src", "iKalibr")
TRANSFORMER_IMU_PATH = os.path.join(WORKSPACE, "src", "Transformer-IMU-Calibrator")


class IMUIntrinsicCalibrator:
    """
    IMU 内参标定器。

    两阶段:
      1. Allan 方差分析: 从静态数据提取噪声参数 (ARW, VRW, bias instability)
      2. iKalibr 连续时间标定: 从动态数据估计完整 12 参数模型

    若 iKalibr 不可用，回退到 Transformer-IMU-Calibrator 或纯 Allan 方差。
    """

    def __init__(self, system_config: dict):
        cfg = (system_config.get("calibration", {})
               .get("stage1_intrinsic", {})
               .get("imu", {}))
        self.accel_model = cfg.get("accel_model", "FULL_TWELVE")
        self.gyro_model = cfg.get("gyro_model", "FULL_TWELVE")
        self.allan_analyzer = AllanVarianceAnalyzer()
        self._ikalibr_available = self._check_ikalibr()

    def calibrate(self, data_mgr: DataManager, sensor: SensorConfig) -> IMUIntrinsic:
        """执行 IMU 内参标定。"""

        # Step 1: 加载 IMU 数据
        imu_data = data_mgr.load_imu_data(sensor.topic)
        if imu_data is None or len(imu_data.get("timestamps", [])) < 100:
            logger.warning(f"  [{sensor.sensor_id}] Insufficient IMU data, using defaults.")
            return self._default_intrinsic()

        logger.info(f"  [{sensor.sensor_id}] IMU data loaded: "
                    f"{len(imu_data['timestamps'])} samples @ "
                    f"{imu_data['sample_rate']:.1f}Hz")

        # Step 2: Allan 方差分析
        allan_result = self.allan_analyzer.analyze(imu_data)

        # Step 3: 多位置标定 (6面法) - 估计加速度计比例和非正交
        accel_params = self._six_position_calibration(imu_data)

        # Step 4: iKalibr 或 Transformer-IMU 精标定
        if self._ikalibr_available:
            result = self._run_ikalibr(imu_data, sensor, allan_result, accel_params)
        else:
            result = self._run_transformer_imu(imu_data, sensor, allan_result)

        if result is None:
            result = self._build_from_allan(allan_result, accel_params)

        logger.info(f"  [{sensor.sensor_id}] IMU intrinsic calibrated:")
        logger.info(f"    Gyro ARW:       {result.gyro_noise:.6f} rad/s/√Hz")
        logger.info(f"    Accel VRW:      {result.accel_noise:.6f} m/s²/√Hz")
        logger.info(f"    Gyro bias inst: {result.gyro_bias_instability:.6f} rad/s")
        logger.info(f"    Accel bias:     {result.accel_bias}")

        return result

    # ------------------------------------------------------------------
    # 6 面法加速度计标定
    # ------------------------------------------------------------------

    def _six_position_calibration(self, imu_data: Dict) -> Optional[Dict]:
        """
        六面静态标定: 将 IMU 放置在 6 个标准位置，每次静止采集约 60 秒。
        估计加速度计的比例因子误差和轴对齐误差。

        若数据中存在明显的静止区段 (加速度 ≈ g)，自动识别并处理。
        """
        accel = imu_data["accel"]  # N×3
        timestamps = imu_data["timestamps"]
        g_nominal = 9.81  # m/s²

        # 自动识别静止区段 (‖a‖ ≈ g, 方差小)
        window = max(10, int(imu_data["sample_rate"] * 2))  # 2 秒窗口
        static_means = []

        i = 0
        while i + window < len(accel):
            segment = accel[i:i+window]
            if np.std(segment) < 0.05:  # 静止判定阈值
                norm = np.linalg.norm(np.mean(segment, axis=0))
                if abs(norm - g_nominal) < 0.5:  # ±0.5 m/s² 容差
                    static_means.append(np.mean(segment, axis=0))
                    i += window  # 跳过已处理的区段
                    continue
            i += max(1, window // 4)

        if len(static_means) < 3:
            logger.info(f"  Only {len(static_means)} static segments, skipping 6-face calib.")
            return None

        logger.info(f"  Found {len(static_means)} static segments for 6-face calibration.")

        # 用最小二乘估计比例误差矩阵
        # 模型: a_meas = Sa * a_true + ba
        # 在各静止位置 a_true 对应各轴 ±g
        meas = np.array(static_means)  # M×3

        # 估计零偏 (各轴相对测量值的中心偏差)
        Sa_diag = np.array([
            np.max(np.abs(meas[:, i])) / g_nominal for i in range(3)
        ])
        ba = np.mean(meas, axis=0) - g_nominal * np.sign(np.mean(meas, axis=0))

        return {
            "Sa_diag": Sa_diag.tolist(),  # 比例因子
            "ba": ba.tolist(),            # 零偏估计
        }

    # ------------------------------------------------------------------
    # iKalibr IMU 内参标定
    # ------------------------------------------------------------------

    def _run_ikalibr(
        self, imu_data: Dict, sensor: SensorConfig,
        allan: Dict, accel_params: Optional[Dict]
    ) -> Optional[IMUIntrinsic]:
        """
        调用 iKalibr 做连续时间 IMU 内参标定。
        """
        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                # 保存 IMU 数据到 CSV
                imu_csv = os.path.join(tmpdir, "imu.csv")
                self._save_imu_csv(imu_data, imu_csv)

                # 生成 iKalibr 配置
                config = self._build_ikalibr_imu_config(sensor, allan, accel_params, tmpdir)
                config_path = os.path.join(tmpdir, "ikalibr_imu_config.yaml")
                with open(config_path, "w") as f:
                    yaml.dump(config, f)

                # 运行 iKalibr
                env = os.environ.copy()
                env["ROS_DOMAIN_ID"] = "0"
                proc = subprocess.run(
                    [
                        "ros2", "run", "ikalibr", "ikalibr_imu_intri_calib",
                        "--ros-args",
                        "-p", f"config_path:={config_path}",
                        "-p", f"bag_path:={imu_csv}",
                    ],
                    env=env, capture_output=True, text=True, timeout=600
                )

                if proc.returncode != 0:
                    logger.warning(f"iKalibr IMU failed: {proc.stderr[-300:]}")
                    return None

                # 解析结果
                result_yaml = os.path.join(tmpdir, "ikalibr_imu_result.yaml")
                if os.path.exists(result_yaml):
                    return self._parse_ikalibr_result(result_yaml, allan)

        except subprocess.TimeoutExpired:
            logger.warning("iKalibr IMU calibration timed out.")
        except FileNotFoundError:
            logger.warning("iKalibr not found, trying Transformer-IMU.")
        except Exception as e:
            logger.warning(f"iKalibr IMU error: {e}")

        return None

    def _build_ikalibr_imu_config(
        self, sensor: SensorConfig, allan: Dict,
        accel_params: Optional[Dict], tmpdir: str
    ) -> dict:
        """构建 iKalibr IMU 内参标定配置文件。"""
        return {
            "Calibration": {
                "Type": "IMU_Intrinsic",
                "OutputPath": tmpdir,
                "IMU": {
                    sensor.sensor_id: {
                        "Type": "SENSOR_IMU",
                        "Topic": sensor.topic,
                        "Intrinsic": {
                            "AcceModel": self.accel_model,
                            "GyroModel": self.gyro_model,
                        },
                        "NoiseParams": {
                            "AcceNoise": allan.get("accel_noise_avg", 0.01),
                            "GyroNoise": allan.get("gyro_noise_avg", 0.001),
                            "AcceBias": allan.get("accel_bias_avg", 0.001),
                            "GyroBias": allan.get("gyro_bias_avg", 0.0001),
                        },
                    }
                },
                "Optimization": {
                    "SplineOrder": 4,
                    "KnotDistance": 0.02,
                    "MaxIterations": 50,
                },
            }
        }

    def _parse_ikalibr_result(self, result_yaml: str, allan: Dict) -> IMUIntrinsic:
        """解析 iKalibr 输出的 IMU 内参结果文件。"""
        with open(result_yaml) as f:
            data = yaml.safe_load(f)

        imu_data = data.get("IMU", {})
        return IMUIntrinsic(
            gyro_noise=allan.get("gyro_noise_avg", 0.001),
            gyro_bias_instability=allan.get("gyro_bias_avg", 0.0001),
            accel_noise=allan.get("accel_noise_avg", 0.01),
            accel_bias_instability=allan.get("accel_bias_avg", 0.001),
            gyro_bias=np.array(imu_data.get("GyroBias", [0, 0, 0])),
            accel_bias=np.array(imu_data.get("AcceBias", [0, 0, 0])),
            Ma=np.array(imu_data.get("Ma", np.eye(3).tolist())),
            Sa=np.array(imu_data.get("Sa", np.eye(3).tolist())),
            Mg=np.array(imu_data.get("Mg", np.eye(3).tolist())),
            Sg=np.array(imu_data.get("Sg", np.eye(3).tolist())),
            Tg=np.array(imu_data.get("Tg", np.zeros((3, 3)).tolist())),
            method="iKalibr_continuous_time",
        )

    # ------------------------------------------------------------------
    # Transformer-IMU-Calibrator
    # ------------------------------------------------------------------

    def _run_transformer_imu(
        self, imu_data: Dict, sensor: SensorConfig, allan: Dict
    ) -> Optional[IMUIntrinsic]:
        """
        调用 Transformer-IMU-Calibrator 做 IMU 内参标定。
        适用于 iKalibr 不可用时的备选方案。
        """
        if not Path(TRANSFORMER_IMU_PATH).exists():
            return None

        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                imu_csv = os.path.join(tmpdir, "imu.csv")
                self._save_imu_csv(imu_data, imu_csv)

                env = os.environ.copy()
                env["PYTHONPATH"] = f"{TRANSFORMER_IMU_PATH}:{env.get('PYTHONPATH', '')}"

                proc = subprocess.run(
                    [
                        sys.executable,
                        os.path.join(TRANSFORMER_IMU_PATH, "eval.py"),
                        "--data", imu_csv,
                        "--output", tmpdir,
                    ],
                    env=env, capture_output=True, text=True, timeout=300
                )

                if proc.returncode == 0:
                    result_file = os.path.join(tmpdir, "imu_intrinsic.yaml")
                    if os.path.exists(result_file):
                        return self._parse_transformer_result(result_file, allan)

        except Exception as e:
            logger.warning(f"Transformer-IMU-Calibrator error: {e}")

        return None

    def _parse_transformer_result(self, result_file: str, allan: Dict) -> IMUIntrinsic:
        with open(result_file) as f:
            data = yaml.safe_load(f)
        return IMUIntrinsic(
            gyro_noise=allan.get("gyro_noise_avg", 0.001),
            gyro_bias_instability=allan.get("gyro_bias_avg", 0.0001),
            accel_noise=allan.get("accel_noise_avg", 0.01),
            accel_bias_instability=allan.get("accel_bias_avg", 0.001),
            gyro_bias=np.array(data.get("gyro_bias", [0, 0, 0])),
            accel_bias=np.array(data.get("accel_bias", [0, 0, 0])),
            method="Transformer-IMU-Calibrator",
        )

    # ------------------------------------------------------------------
    # 辅助方法
    # ------------------------------------------------------------------

    def _build_from_allan(self, allan: Dict,
                          accel_params: Optional[Dict]) -> IMUIntrinsic:
        """仅使用 Allan 方差结果构建 IMUIntrinsic。"""
        Sa = np.eye(3)
        ba = np.zeros(3)
        if accel_params:
            Sa_diag = np.array(accel_params.get("Sa_diag", [1, 1, 1]))
            Sa = np.diag(Sa_diag)
            ba = np.array(accel_params.get("ba", [0, 0, 0]))

        return IMUIntrinsic(
            gyro_noise=allan.get("gyro_noise_avg", 0.001),
            gyro_bias_instability=allan.get("gyro_bias_avg", 0.0001),
            accel_noise=allan.get("accel_noise_avg", 0.01),
            accel_bias_instability=allan.get("accel_bias_avg", 0.001),
            accel_bias=ba,
            Sa=Sa,
            method="AllanVariance_only",
        )

    def _default_intrinsic(self) -> IMUIntrinsic:
        """典型 MEMS IMU 默认参数。"""
        return IMUIntrinsic(
            gyro_noise=1.6968e-4,         # BMI088 typical ARW
            gyro_bias_instability=1e-5,
            accel_noise=2.0e-3,           # BMI088 typical VRW
            accel_bias_instability=3e-4,
            method="default_mems",
        )

    def _save_imu_csv(self, imu_data: Dict, path: str):
        """将 IMU 数据保存为 CSV 文件。"""
        import csv
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "gx", "gy", "gz", "ax", "ay", "az"])
            for t, g, a in zip(
                imu_data["timestamps"],
                imu_data["gyro"],
                imu_data["accel"]
            ):
                writer.writerow([t, g[0], g[1], g[2], a[0], a[1], a[2]])

    @staticmethod
    def _check_ikalibr() -> bool:
        """检查 iKalibr 是否可用。"""
        try:
            result = subprocess.run(
                ["ros2", "pkg", "list"], capture_output=True, text=True, timeout=5)
            return "ikalibr" in result.stdout
        except Exception:
            return False
