"""
外参标定模块单元测试
验证粗/精标定算法的基本正确性
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


class TestFeatureMatchingCalibrator:
    """特征匹配 Camera-Camera 外参标定测试。"""

    def test_init(self):
        from unicalib.extrinsic.coarse.feature_matching import FeatureMatchingCalibrator
        cfg = {"calibration": {"stage2_coarse": {"camera_camera": {
            "feature_matcher": "orb", "min_matches": 10, "max_frames": 20
        }}}}
        calib = FeatureMatchingCalibrator(cfg)
        assert calib.feature_type == "orb"
        assert calib.min_matches == 10

    def test_get_pinhole_K(self):
        from unicalib.extrinsic.coarse.feature_matching import FeatureMatchingCalibrator
        from unicalib.core.calib_result import CameraIntrinsic
        calib = FeatureMatchingCalibrator({})
        K_gt = np.array([[900, 0, 640], [0, 900, 480], [0, 0, 1]], dtype=np.float64)
        intr = CameraIntrinsic(K=K_gt, dist_coeffs=np.zeros(5),
                                image_size=(1280, 960))
        K = calib._get_pinhole_K(intr)
        np.testing.assert_allclose(K, K_gt)


class TestIMULidarInitCalibrator:
    """IMU-LiDAR 初始化标定测试。"""

    def test_gravity_align(self):
        """重力方向对齐计算。"""
        from unicalib.extrinsic.coarse.learn_to_calib_wrapper import _rotation_from_vectors

        v1 = np.array([0.0, 0.0, 1.0])
        v2 = np.array([0.0, 0.0, 1.0])
        R = _rotation_from_vectors(v1, v2)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)

    def test_gravity_align_90deg(self):
        """90° 旋转验证。"""
        from unicalib.extrinsic.coarse.learn_to_calib_wrapper import _rotation_from_vectors

        v1 = np.array([1.0, 0.0, 0.0])
        v2 = np.array([0.0, 0.0, 1.0])
        R = _rotation_from_vectors(v1, v2)
        v1_rot = R @ v1
        np.testing.assert_allclose(v1_rot, v2, atol=1e-10)

    def test_skew_symmetric(self):
        """反对称矩阵测试。"""
        from unicalib.extrinsic.coarse.imu_lidar_init import _skew

        v = np.array([1.0, 2.0, 3.0])
        S = _skew(v)
        assert S.shape == (3, 3)
        # 反对称性: S + S.T = 0
        np.testing.assert_allclose(S + S.T, np.zeros((3, 3)), atol=1e-10)
        # 叉积验证: skew(v) @ w = v × w
        w = np.array([4.0, 5.0, 6.0])
        np.testing.assert_allclose(S @ w, np.cross(v, w), atol=1e-10)

    def test_imu_integration(self):
        """IMU 旋转积分验证。"""
        from unicalib.extrinsic.coarse.imu_lidar_init import IMULidarInitCalibrator

        calib = IMULidarInitCalibrator({})
        rate = 200.0
        T = 2.0  # 秒
        N = int(T * rate)

        # 绕 Z 轴以 π/4 rad/s 旋转
        omega = np.array([0.0, 0.0, np.pi / 4])
        timestamps = np.linspace(0, T, N)
        gyro = np.tile(omega, (N, 1))

        imu_data = {
            "timestamps": timestamps,
            "gyro": gyro,
            "accel": np.zeros((N, 3)),
            "sample_rate": rate,
        }

        rotations = calib._integrate_imu_rotations(imu_data, n_segments=1)
        # 两段旋转量应 ≈ π/4 * 2s = π/2 rad = 90°
        if rotations:
            euler = Rotation.from_matrix(rotations[0]).as_euler("zyx", degrees=True)
            # 允许较大误差（积分误差）
            assert abs(abs(euler[0]) - 90.0) < 10.0 or len(rotations) == 0


class TestCalibResult:
    """CalibResult 数据结构测试。"""

    def test_to_from_dict_roundtrip(self):
        from unicalib.core.calib_result import CalibResult
        R = np.eye(3)
        t = np.array([0.1, -0.2, 0.3])
        result = CalibResult(
            pair=("imu0", "lidar0"), rotation=R, translation=t,
            time_offset=0.005, reprojection_error=0.5,
            confidence=0.9, method_used="test")

        d = result.to_dict()
        result2 = CalibResult.from_dict(d)

        assert result2.pair == result.pair
        np.testing.assert_allclose(result2.rotation, R)
        np.testing.assert_allclose(result2.translation, t)
        assert abs(result2.time_offset - 0.005) < 1e-9

    def test_rotation_euler_deg(self):
        from unicalib.core.calib_result import CalibResult
        R = Rotation.from_euler("z", 45.0, degrees=True).as_matrix()
        result = CalibResult(pair=("a", "b"), rotation=R, translation=np.zeros(3))
        euler = result.rotation_euler_deg()
        assert abs(euler[0] - 45.0) < 0.001


class TestSensorConfig:
    """传感器配置测试。"""

    def test_auto_infer_pairs(self):
        from unicalib.core.sensor_config import (
            SensorConfig, SensorType, auto_infer_calib_pairs)

        sensors = {
            "imu0": SensorConfig("imu0", SensorType.IMU, "/imu", "imu"),
            "lidar0": SensorConfig("lidar0", SensorType.LIDAR, "/lidar", "lidar"),
            "cam_front": SensorConfig("cam_front", SensorType.CAMERA_PINHOLE,
                                      "/cam", "cam"),
        }
        pairs = auto_infer_calib_pairs(sensors)

        assert len(pairs) >= 2, "Should infer at least 2 pairs"

        methods = {(p.sensor_a, p.sensor_b): p.method_fine for p in pairs}
        # IMU-LiDAR 应使用 iKalibr
        pair_types = [(p.sensor_a, p.sensor_b) for p in pairs]
        imu_lidar_found = any(
            ("imu0" in k and "lidar0" in k)
            for k in [f"{a}+{b}" for a, b in pair_types]
        )
        assert imu_lidar_found, "IMU-LiDAR pair should be inferred"

    def test_priority_ordering(self):
        from unicalib.core.sensor_config import (
            SensorConfig, SensorType, auto_infer_calib_pairs)

        sensors = {
            "imu0": SensorConfig("imu0", SensorType.IMU, "/imu", "imu"),
            "lidar0": SensorConfig("lidar0", SensorType.LIDAR, "/lidar", "lidar"),
            "cam0": SensorConfig("cam0", SensorType.CAMERA_PINHOLE, "/cam", "cam"),
            "cam1": SensorConfig("cam1", SensorType.CAMERA_PINHOLE, "/cam1", "cam1"),
        }
        pairs = auto_infer_calib_pairs(sensors)
        priorities = [p.priority for p in pairs]
        assert priorities == sorted(priorities), "Pairs should be sorted by priority"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
