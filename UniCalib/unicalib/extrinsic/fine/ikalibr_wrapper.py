"""
iKalibr 封装器 —— 连续时间 B-spline 多传感器联合精标定
项目位置: /root/calib_ws/src/iKalibr/
"""
from __future__ import annotations
import logging
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from ...core.calib_result import CalibResult
from ...core.sensor_config import SensorConfig, SensorType
from ...core.data_manager import DataManager

logger = logging.getLogger(__name__)

WORKSPACE = os.environ.get("WORKSPACE", "/root/calib_ws")
IKALIBR_WS = WORKSPACE  # iKalibr 编译在 colcon 工作空间中


class IKalibrWrapper:
    """
    iKalibr 连续时间 B-spline 多传感器联合精标定封装。

    核心能力:
      - IMU-LiDAR 外参 (R, t) + 时间偏移 td
      - IMU-Camera 外参 (通过 LiDAR 桥接)
      - B-spline 轨迹表示 (SO3 × R3)
      - Ceres 非线性优化

    调用 iKalibr ROS2 节点:
      ros2 run ikalibr ikalibr_solver --ros-args -p config_path:=<config>
    """

    def __init__(self, system_config: dict):
        cfg = (system_config.get("calibration", {})
               .get("stage3_fine", {})
               .get("global_optimization", {}))
        self.spline_order = cfg.get("spline_order", 4)
        self.knot_so3 = cfg.get("knot_distance_so3", 0.02)
        self.knot_pos = cfg.get("knot_distance_pos", 0.02)
        self.max_iter = cfg.get("max_iterations", 50)
        self.enable_td = cfg.get("enable_time_offset", True)
        self.robust_kernel = cfg.get("robust_kernel", "cauchy")
        self.robust_param = cfg.get("robust_kernel_param", 1.0)
        self._available = self._check_ikalibr()

    def joint_optimize(
        self,
        data_mgr: DataManager,
        sensors: Dict[str, SensorConfig],
        intrinsics: Dict,
        coarse_results: Dict[Tuple[str, str], CalibResult],
    ) -> Dict[Tuple[str, str], CalibResult]:
        """
        全局联合 B-spline 优化所有传感器外参。
        返回精化后的外参字典。
        """
        if not self._available:
            logger.warning("iKalibr not available, returning coarse results.")
            return coarse_results

        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                config = self._build_config(
                    sensors, intrinsics, coarse_results, tmpdir, data_mgr)
                config_path = os.path.join(tmpdir, "ikalibr_joint.yaml")
                with open(config_path, "w") as f:
                    yaml.dump(config, f, default_flow_style=False)

                logger.info(f"  Running iKalibr joint optimization...")
                logger.debug(f"  Config: {config_path}")

                env = os.environ.copy()
                proc = subprocess.run(
                    [
                        "ros2", "run", "ikalibr", "ikalibr_prog",
                        "--ros-args",
                        "-p", f"config_path:={config_path}",
                    ],
                    env=env,
                    capture_output=True, text=True,
                    timeout=1800,  # 30 分钟超时
                )

                if proc.returncode != 0:
                    logger.error(f"iKalibr failed:\n{proc.stderr[-500:]}")
                    return coarse_results

                # 解析结果
                result_dir = os.path.join(tmpdir, "result")
                if os.path.exists(result_dir):
                    return self._parse_results(result_dir, coarse_results)
                else:
                    logger.warning("iKalibr result directory not found.")
                    return coarse_results

        except subprocess.TimeoutExpired:
            logger.error("iKalibr timed out (1800s).")
        except Exception as e:
            logger.error(f"iKalibr error: {e}", exc_info=True)

        return coarse_results

    # ------------------------------------------------------------------
    # 配置文件构建
    # ------------------------------------------------------------------

    def _build_config(
        self,
        sensors: Dict[str, SensorConfig],
        intrinsics: Dict,
        coarse_results: Dict[Tuple[str, str], CalibResult],
        output_dir: str,
        data_mgr: DataManager,
    ) -> dict:
        """构建 iKalibr 联合优化配置文件。"""

        config = {
            "Calibration": {
                "Type": "MULTI_SENSOR",
                "OutputPath": os.path.join(output_dir, "result"),
                "DataPath": str(data_mgr.data_path),

                # B-Spline 配置
                "BSpline": {
                    "SplineOrder": self.spline_order,
                    "KnotTimeDistance": {
                        "SO3": self.knot_so3,
                        "Pos": self.knot_pos,
                    },
                },

                # 优化配置
                "Optimization": {
                    "MaxIterations": self.max_iter,
                    "EnableTimeOffset": self.enable_td,
                    "TimeOffsetPadding": 0.1,  # ±100ms 搜索范围
                    "RobustKernel": self.robust_kernel,
                    "RobustKernelParam": self.robust_param,
                },

                # 传感器配置
                "Sensors": {},

                # 初始外参
                "InitialExtrinsics": {},
            }
        }

        for sid, sensor in sensors.items():
            sensor_cfg: dict = {
                "Type": self._sensor_type_str(sensor.sensor_type),
                "Topic": sensor.topic,
                "FrameId": sensor.frame_id,
            }

            if sensor.is_camera():
                intr = intrinsics.get(sid)
                if intr and hasattr(intr, "to_dict"):
                    sensor_cfg["Intrinsic"] = intr.to_dict()
            elif sensor.is_lidar() and sensor.lidar_type:
                sensor_cfg["LidarType"] = sensor.lidar_type.value

            if sensor.rate:
                sensor_cfg["Rate"] = sensor.rate

            config["Calibration"]["Sensors"][sid] = sensor_cfg

        # 填充初始外参
        for (sa, sb), result in coarse_results.items():
            key = f"{sa}_to_{sb}"
            config["Calibration"]["InitialExtrinsics"][key] = {
                "Rotation": result.rotation.tolist(),
                "Translation": result.translation.tolist(),
                "TimeOffset": result.time_offset,
            }

        return config

    # ------------------------------------------------------------------
    # 结果解析
    # ------------------------------------------------------------------

    def _parse_results(
        self, result_dir: str,
        fallback: Dict[Tuple[str, str], CalibResult],
    ) -> Dict[Tuple[str, str], CalibResult]:
        """解析 iKalibr 输出的 YAML 结果文件。"""
        results = dict(fallback)

        result_yaml = os.path.join(result_dir, "calibration_result.yaml")
        if not os.path.exists(result_yaml):
            # 尝试其他命名
            for fname in os.listdir(result_dir):
                if fname.endswith(".yaml"):
                    result_yaml = os.path.join(result_dir, fname)
                    break

        if not os.path.exists(result_yaml):
            logger.warning(f"iKalibr result YAML not found in {result_dir}")
            return results

        with open(result_yaml) as f:
            data = yaml.safe_load(f)

        extrinsics = data.get("Extrinsics", data.get("extrinsics", {}))
        for key_str, ext_data in extrinsics.items():
            parts = key_str.replace("_to_", "|").split("|")
            if len(parts) != 2:
                continue
            sa, sb = parts[0].strip(), parts[1].strip()

            R = np.array(ext_data.get("Rotation", np.eye(3).tolist()))
            t = np.array(ext_data.get("Translation", [0, 0, 0]))
            td = float(ext_data.get("TimeOffset", 0.0))
            reproj = float(ext_data.get("ReprojectionError", float('inf')))

            results[(sa, sb)] = CalibResult(
                pair=(sa, sb),
                rotation=R, translation=t.flatten(),
                time_offset=td,
                reprojection_error=reproj,
                confidence=0.9,
                method_used="iKalibr_bspline",
            )
            logger.info(f"  iKalibr result [{sa}->{sb}]: "
                        f"R_euler={Rotation.from_matrix(R).as_euler('zyx', degrees=True)}, "
                        f"t={t}, td={td*1000:.2f}ms, reproj={reproj:.3f}px")

        return results

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    @staticmethod
    def _sensor_type_str(stype: SensorType) -> str:
        mapping = {
            SensorType.IMU: "SENSOR_IMU",
            SensorType.LIDAR: "SENSOR_LIDAR",
            SensorType.CAMERA_PINHOLE: "SENSOR_CAMERA",
            SensorType.CAMERA_FISHEYE: "SENSOR_CAMERA_FISHEYE",
        }
        return mapping.get(stype, "SENSOR_UNKNOWN")

    @staticmethod
    def _check_ikalibr() -> bool:
        try:
            result = subprocess.run(
                ["ros2", "pkg", "list"], capture_output=True, text=True, timeout=5)
            available = "ikalibr" in result.stdout.lower()
            if not available:
                logger.warning("iKalibr ROS2 package not found.")
            return available
        except Exception:
            return False
