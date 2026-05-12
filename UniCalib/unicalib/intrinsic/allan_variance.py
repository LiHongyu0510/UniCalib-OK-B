"""
Allan 方差分析工具
用于从 IMU 静态数据中提取噪声参数:
  - 角度随机游走 (ARW)
  - 速度随机游走 (VRW)
  - 零偏不稳定性 (Bias Instability)
"""
from __future__ import annotations
import logging
from typing import Dict, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class AllanVarianceAnalyzer:
    """
    使用重叠 Allan 偏差 (OADEV) 分析 IMU 噪声。

    参考: IEEE Std 952-1997 (IMU 误差分析)
    输出参数单位:
      - ARW [rad/s/√Hz] (陀螺仪) 或 [m/s²/√Hz] (加速度计)
      - 零偏不稳定性 [rad/s] 或 [m/s²]
    """

    def analyze(self, imu_data: Dict) -> Dict:
        """
        对 IMU 数据进行 Allan 方差分析。

        Args:
            imu_data: {timestamps, gyro (N×3), accel (N×3), sample_rate}

        Returns:
            {
              "gyro": {"axis_x/y/z": {random_walk, bias_instability, ...}},
              "accel": {"axis_x/y/z": {random_walk, bias_instability, ...}},
              "gyro_noise_avg": float,
              "gyro_bias_avg": float,
              "accel_noise_avg": float,
              "accel_bias_avg": float,
            }
        """
        rate = imu_data.get("sample_rate", 200.0)
        gyro = imu_data["gyro"]   # N×3
        accel = imu_data["accel"] # N×3

        results = {"gyro": {}, "accel": {}}
        axis_names = ["x", "y", "z"]

        gyro_rw_list, gyro_bi_list = [], []
        accel_rw_list, accel_bi_list = [], []

        for i, name in enumerate(axis_names):
            # 陀螺仪
            g_res = self._analyze_axis(gyro[:, i], rate, name=f"gyro_{name}")
            results["gyro"][f"axis_{name}"] = g_res
            gyro_rw_list.append(g_res["random_walk"])
            gyro_bi_list.append(g_res["bias_instability"])

            # 加速度计
            a_res = self._analyze_axis(accel[:, i], rate, name=f"accel_{name}")
            results["accel"][f"axis_{name}"] = a_res
            accel_rw_list.append(a_res["random_walk"])
            accel_bi_list.append(a_res["bias_instability"])

        results["gyro_noise_avg"] = float(np.mean(gyro_rw_list))
        results["gyro_bias_avg"] = float(np.mean(gyro_bi_list))
        results["accel_noise_avg"] = float(np.mean(accel_rw_list))
        results["accel_bias_avg"] = float(np.mean(accel_bi_list))

        logger.info(f"  Allan Variance 分析完成:")
        logger.info(f"    陀螺 ARW:  {results['gyro_noise_avg']:.6f} rad/s/√Hz")
        logger.info(f"    陀螺 Bias: {results['gyro_bias_avg']:.6f} rad/s")
        logger.info(f"    加速度 VRW: {results['accel_noise_avg']:.6f} m/s²/√Hz")
        logger.info(f"    加速度 Bias: {results['accel_bias_avg']:.6f} m/s²")

        return results

    def _analyze_axis(self, data: np.ndarray, rate: float, name: str = "") -> Dict:
        """对单轴数据进行 OADEV 分析，提取关键噪声参数。"""
        try:
            return self._oadev_analysis(data, rate)
        except ImportError:
            logger.warning("allantools not installed, using simplified estimation.")
            return self._simple_analysis(data, rate)
        except Exception as e:
            logger.warning(f"Allan variance analysis failed for {name}: {e}")
            return self._simple_analysis(data, rate)

    def _oadev_analysis(self, data: np.ndarray, rate: float) -> Dict:
        """使用 allantools 库计算 OADEV。"""
        from allantools import oadev

        taus, adevs, errors, ns = oadev(
            data, rate=rate, data_type="freq", taus="all")

        rw = self._extract_random_walk(taus, adevs)
        bi = self._extract_bias_instability(taus, adevs)
        rate_rw = self._extract_rate_random_walk(taus, adevs)

        return {
            "taus": taus.tolist(),
            "adevs": adevs.tolist(),
            "random_walk": float(rw),
            "bias_instability": float(bi),
            "rate_random_walk": float(rate_rw),
        }

    def _simple_analysis(self, data: np.ndarray, rate: float) -> Dict:
        """
        简化估计 (不依赖 allantools):
          - ARW ≈ σ(data) / √rate
          - 零偏不稳定性 ≈ 长窗口均值的标准差
        """
        dt = 1.0 / rate
        random_walk = float(np.std(data) / np.sqrt(rate))

        window = max(1, int(rate * 100))  # 100 秒窗口
        if len(data) > window * 2:
            means = [np.mean(data[i:i+window]) for i in range(0, len(data)-window, window//2)]
            bias_instability = float(np.std(means))
        else:
            bias_instability = float(np.std(data)) * 0.1

        return {
            "random_walk": random_walk,
            "bias_instability": bias_instability,
            "rate_random_walk": random_walk * 0.1,
        }

    # ------------------------------------------------------------------
    # 从 Allan 偏差曲线提取参数
    # ------------------------------------------------------------------

    def _extract_random_walk(self, taus: np.ndarray, adevs: np.ndarray) -> float:
        """
        提取随机游走 (ARW/VRW):
        在 log-log 图中斜率为 -0.5 的区域 (τ=1s 处的 OADEV 值)。
        """
        # 找 τ=1s 附近的 OADEV
        idx = np.argmin(np.abs(taus - 1.0))
        if len(adevs) > idx:
            return float(adevs[idx])
        # 若没有 τ=1s 的点则外推
        log_taus = np.log10(taus + 1e-12)
        log_adevs = np.log10(adevs + 1e-12)
        # 拟合斜率 -0.5 区域
        mask = (taus < 10.0) & (taus > 0.1)
        if np.sum(mask) > 2:
            slope, intercept = np.polyfit(log_taus[mask], log_adevs[mask], 1)
            return float(10 ** intercept)  # 在 τ=1 处的值
        return float(adevs[0])

    def _extract_bias_instability(self, taus: np.ndarray, adevs: np.ndarray) -> float:
        """
        提取零偏不稳定性:
        OADEV 曲线的最小值 (斜率为 0 的区域)。
        """
        return float(np.min(adevs))

    def _extract_rate_random_walk(self, taus: np.ndarray, adevs: np.ndarray) -> float:
        """
        提取速率随机游走:
        斜率为 +0.5 区域 (τ=3s 处的 OADEV / √3)。
        """
        idx = np.argmin(np.abs(taus - 3.0))
        if len(adevs) > idx:
            return float(adevs[idx] / np.sqrt(3.0))
        return 0.0
