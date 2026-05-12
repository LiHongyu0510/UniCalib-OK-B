"""
IMU-LiDAR 外参粗初始化
基于运动约束的旋转/平移解析估计
(iKalibr 初始化模块的 Python 实现)
"""
from __future__ import annotations
import logging
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from ...core.calib_result import CalibResult
from ...core.sensor_config import SensorConfig
from ...core.data_manager import DataManager

logger = logging.getLogger(__name__)


class IMULidarInitCalibrator:
    """
    IMU-LiDAR 旋转外参初始化。

    方法: Hand-eye calibration via rotation averaging
      - 从 IMU 角速度积分得到旋转序列 {R_I}
      - 从 LiDAR ICP/NDT 里程计得到旋转序列 {R_L}
      - 求解 R_IL 使得 R_I ≈ R_IL * R_L * R_IL^{-1}
    """

    def __init__(self, system_config: dict):
        cfg = (system_config.get("calibration", {})
               .get("stage2_coarse", {})
               .get("imu_lidar", {}))
        self.min_rotation_deg = cfg.get("min_rotation_deg", 5.0)
        self.n_segments = cfg.get("n_segments", 20)

    def calibrate(
        self,
        data_mgr: DataManager,
        sensor_imu: SensorConfig,
        sensor_lidar: SensorConfig,
        intrinsics: Dict,
    ) -> CalibResult:
        """估计 IMU-LiDAR 旋转外参初始值。"""

        # 确保顺序
        if sensor_imu.is_lidar():
            sensor_imu, sensor_lidar = sensor_lidar, sensor_imu

        imu_data = data_mgr.load_imu_data(sensor_imu.topic)
        lidar_frames = list(data_mgr.iter_pointclouds(
            sensor_lidar.topic, max_frames=200, skip=1))

        if imu_data is None or len(lidar_frames) < 10:
            logger.warning("Insufficient data for IMU-LiDAR init, using identity.")
            return CalibResult(
                pair=(sensor_imu.sensor_id, sensor_lidar.sensor_id),
                rotation=np.eye(3), translation=np.zeros(3),
                confidence=0.1, method_used="identity_fallback")

        # Step 1: IMU 旋转序列
        imu_rotations = self._integrate_imu_rotations(imu_data, self.n_segments)

        # Step 2: LiDAR ICP 旋转序列
        lidar_rotations = self._estimate_lidar_rotations(lidar_frames, self.n_segments)

        if len(imu_rotations) < 3 or len(lidar_rotations) < 3:
            return CalibResult(
                pair=(sensor_imu.sensor_id, sensor_lidar.sensor_id),
                rotation=np.eye(3), translation=np.zeros(3),
                confidence=0.1, method_used="insufficient_data")

        # Step 3: Hand-eye calibration
        R_IL = self._hand_eye_rotation(imu_rotations, lidar_rotations)

        logger.info(f"  IMU-LiDAR init: R_euler={Rotation.from_matrix(R_IL).as_euler('zyx', degrees=True)}")

        return CalibResult(
            pair=(sensor_imu.sensor_id, sensor_lidar.sensor_id),
            rotation=R_IL,
            translation=np.zeros(3),  # 平移需要精标定阶段估计
            confidence=0.4,
            method_used="hand_eye_rotation_init",
        )

    # ------------------------------------------------------------------
    # IMU 旋转积分
    # ------------------------------------------------------------------

    def _integrate_imu_rotations(
        self, imu_data: Dict, n_segments: int
    ) -> List[np.ndarray]:
        """
        将 IMU 数据分成 n 段，对每段积分得到相对旋转。
        返回各段的相对旋转矩阵列表。
        """
        n_segments = max(1, n_segments)
        timestamps = imu_data["timestamps"]
        gyro = imu_data["gyro"]
        n = len(timestamps)
        seg_len = n // n_segments

        rotations = []
        for i in range(n_segments):
            start = i * seg_len
            end = min((i + 1) * seg_len, n)
            if end - start < 5:
                continue

            t_seg = timestamps[start:end]
            g_seg = gyro[start:end]

            # 欧拉积分
            R = np.eye(3)
            for j in range(len(t_seg) - 1):
                dt = t_seg[j+1] - t_seg[j]
                omega = g_seg[j]
                angle = np.linalg.norm(omega) * dt
                if angle > 1e-8:
                    axis = omega / np.linalg.norm(omega)
                    dR = Rotation.from_rotvec(axis * angle).as_matrix()
                    R = R @ dR

            # 只保留有足够旋转量的段
            euler = Rotation.from_matrix(R).as_euler('zyx', degrees=True)
            if np.max(np.abs(euler)) > self.min_rotation_deg:
                rotations.append(R)

        return rotations

    # ------------------------------------------------------------------
    # LiDAR ICP 里程计
    # ------------------------------------------------------------------

    def _estimate_lidar_rotations(
        self, lidar_frames: List[Tuple], n_segments: int
    ) -> List[np.ndarray]:
        """
        用 Open3D ICP 估计相邻 LiDAR 帧之间的旋转。
        将帧分成 n 段，计算每段的累积旋转。
        """
        try:
            import open3d as o3d
        except ImportError:
            logger.warning("open3d not available, cannot compute LiDAR odometry.")
            return []

        seg_len = max(1, len(lidar_frames) // n_segments)
        rotations = []

        for seg_idx in range(n_segments):
            start = seg_idx * seg_len
            end = min((seg_idx + 1) * seg_len, len(lidar_frames))
            if end - start < 2:
                continue

            R_accum = np.eye(3)
            valid = True

            for i in range(start, end - 1):
                _, pts_src = lidar_frames[i]
                _, pts_tgt = lidar_frames[i + 1]

                pcd_src = o3d.geometry.PointCloud()
                pcd_src.points = o3d.utility.Vector3dVector(pts_src[:, :3])
                pcd_tgt = o3d.geometry.PointCloud()
                pcd_tgt.points = o3d.utility.Vector3dVector(pts_tgt[:, :3])

                # 下采样
                pcd_src = pcd_src.voxel_down_sample(0.2)
                pcd_tgt = pcd_tgt.voxel_down_sample(0.2)

                if len(pcd_src.points) < 100 or len(pcd_tgt.points) < 100:
                    valid = False
                    break

                # Point-to-point ICP
                reg = o3d.pipelines.registration.registration_icp(
                    pcd_src, pcd_tgt,
                    max_correspondence_distance=0.5,
                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                        max_iteration=30)
                )

                if reg.fitness < 0.3:
                    valid = False
                    break

                T = reg.transformation
                R_accum = R_accum @ T[:3, :3]

            if valid:
                euler = Rotation.from_matrix(R_accum).as_euler('zyx', degrees=True)
                if np.max(np.abs(euler)) > self.min_rotation_deg:
                    rotations.append(R_accum)

        return rotations

    # ------------------------------------------------------------------
    # Hand-Eye Calibration (旋转)
    # ------------------------------------------------------------------

    def _hand_eye_rotation(
        self, R_imu: List[np.ndarray], R_lidar: List[np.ndarray]
    ) -> np.ndarray:
        """
        求解手眼标定方程 R_A * R_X = R_X * R_B，其中:
          R_A = IMU 旋转, R_B = LiDAR 旋转, R_X = IMU-LiDAR 旋转外参

        使用 Park & Martin (1994) 方法。
        """
        n = min(len(R_imu), len(R_lidar))

        # 构建方程: R_A - I = skew(q_X_vec) * (R_B + I)
        M = np.zeros((3 * n, 3))
        b_vec = np.zeros(3 * n)

        for i in range(n):
            # 轴角表示
            rv_A = Rotation.from_matrix(R_imu[i]).as_rotvec()
            rv_B = Rotation.from_matrix(R_lidar[i]).as_rotvec()
            M[3*i:3*i+3, :] = _skew(rv_A + rv_B)
            b_vec[3*i:3*i+3] = rv_A - rv_B

        # 最小二乘求解
        q_vec, _, _, _ = np.linalg.lstsq(M, b_vec, rcond=None)
        q_norm = np.linalg.norm(q_vec)
        if q_norm < 1e-10:
            return np.eye(3)

        q_unit = q_vec / q_norm
        theta = 2.0 * np.arctan(q_norm)
        R_X = Rotation.from_rotvec(q_unit * theta).as_matrix()
        return R_X


def _skew(v: np.ndarray) -> np.ndarray:
    """反对称矩阵 (叉积矩阵)。"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])
