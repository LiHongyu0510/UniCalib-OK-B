"""
内参标定模块单元测试
不依赖实际传感器数据，使用合成数据验证算法正确性
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pytest
import cv2


class TestAllanVariance:
    """Allan 方差分析测试。"""

    def test_random_walk_estimation(self):
        """验证白噪声 Allan 方差估计。"""
        from unicalib.intrinsic.allan_variance import AllanVarianceAnalyzer

        np.random.seed(42)
        rate = 200.0
        N = 72000  # 360 秒
        noise_sigma = 0.001  # rad/s

        data = np.random.randn(N) * noise_sigma
        imu_data = {
            "timestamps": np.arange(N) / rate,
            "gyro": np.column_stack([data, data * 0.9, data * 1.1]),
            "accel": np.column_stack([data * 10, data * 10, data * 10]),
            "sample_rate": rate,
        }

        analyzer = AllanVarianceAnalyzer()
        result = analyzer.analyze(imu_data)

        assert "gyro_noise_avg" in result
        assert "accel_noise_avg" in result
        assert result["gyro_noise_avg"] > 0
        assert result["accel_noise_avg"] > 0

    def test_simple_analysis_fallback(self):
        """验证不依赖 allantools 的简化分析。"""
        from unicalib.intrinsic.allan_variance import AllanVarianceAnalyzer

        analyzer = AllanVarianceAnalyzer()
        np.random.seed(0)
        data = np.random.randn(1000) * 0.01
        result = analyzer._simple_analysis(data, rate=200.0)

        assert "random_walk" in result
        assert "bias_instability" in result
        assert result["random_walk"] > 0


class TestCameraPinholeCalibrator:
    """针孔相机内参标定测试。"""

    def _make_checkerboard_images(self, n=20, rows=7, cols=10,
                                   square=30, K=None, D=None, size=(1280, 960)):
        """生成合成棋盘格图像用于测试。"""
        W, H = size
        if K is None:
            K = np.array([[900, 0, W/2], [0, 900, H/2], [0, 0, 1]], dtype=np.float64)
        if D is None:
            D = np.array([0.1, -0.05, 0.001, 0.001, 0.0])

        pattern = (cols - 1, rows - 1)
        objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
        objp *= square

        images = []
        rng = np.random.default_rng(42)
        for _ in range(n):
            # 随机位姿
            rvec = rng.uniform(-0.3, 0.3, 3)
            tvec = np.array([rng.uniform(-50, 50), rng.uniform(-50, 50), 400.0])
            pts_2d, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
            pts_2d = pts_2d.reshape(-1, 2)

            img = np.ones((H, W, 3), dtype=np.uint8) * 128
            # 绘制棋盘格
            for i, p in enumerate(pts_2d.astype(int)):
                row = i // pattern[0]
                col = i % pattern[0]
                if row < pattern[1] - 1 and col < pattern[0] - 1:
                    p1 = tuple(pts_2d[i].astype(int))
                    p2 = tuple(pts_2d[i + 1].astype(int))
                    p3 = tuple(pts_2d[i + pattern[0]].astype(int))
                    p4 = tuple(pts_2d[i + pattern[0] + 1].astype(int))
                    pts_rect = np.array([p1, p2, p4, p3])
                    color = 255 if (row + col) % 2 == 0 else 0
                    cv2.fillPoly(img, [pts_rect], (color, color, color))
            images.append(img)

        return images, K, D

    def test_calibrator_init(self):
        """测试标定器初始化。"""
        from unicalib.intrinsic.camera_pinhole import CameraPinholeCalibrator
        config = {
            "calibration": {"stage1_intrinsic": {"camera_pinhole": {
                "primary_method": "opencv",
                "refinement_method": "opencv",
                "min_reproj_error": 0.5,
            }}},
            "data_collection": {"camera_intrinsic": {"checkerboard": {
                "rows": 8, "cols": 11, "square_size": 0.03
            }}},
        }
        calib = CameraPinholeCalibrator(config)
        assert calib.checker_rows == 8
        assert calib.checker_cols == 11

    def test_opencv_calibration(self):
        """测试 OpenCV 棋盘格标定准确性。"""
        from unicalib.intrinsic.camera_pinhole import CameraPinholeCalibrator

        config = {
            "calibration": {"stage1_intrinsic": {"camera_pinhole": {
                "primary_method": "opencv",
                "refinement_method": "opencv",
                "min_reproj_error": 1.0,
            }}},
            "data_collection": {"camera_intrinsic": {"checkerboard": {
                "rows": 8, "cols": 11, "square_size": 0.03
            }}},
        }

        calib = CameraPinholeCalibrator(config)
        images, K_gt, D_gt = self._make_checkerboard_images()
        result = calib._opencv_checkerboard_calib(images, (1280, 960))

        assert result is not None, "OpenCV calibration should succeed with synthetic data"
        # 焦距误差应小于 10%
        assert abs(result.K[0, 0] - K_gt[0, 0]) / K_gt[0, 0] < 0.1, \
            f"fx error too large: {result.K[0,0]} vs {K_gt[0,0]}"
        assert result.reprojection_error < 2.0, \
            f"Reprojection error too large: {result.reprojection_error}"


class TestFisheyeCalibrator:
    """鱼眼相机内参标定测试。"""

    def test_eucm_projection(self):
        """验证 EUCM 投影函数。"""
        from unicalib.intrinsic.camera_fisheye import _project_eucm

        pts = np.array([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0], [0.0, 0.1, 1.0]])
        result = _project_eucm(pts, alpha=0.6, beta=1.0,
                               fx=400, fy=400, cx=320, cy=240)
        assert result is not None
        assert result.shape == (3, 2)
        # 光轴方向应投影到主点
        assert abs(result[0, 0] - 320) < 1.0
        assert abs(result[0, 1] - 240) < 1.0

    def test_double_sphere_projection(self):
        """验证 Double Sphere 投影函数。"""
        from unicalib.intrinsic.camera_fisheye import _project_double_sphere

        pts = np.array([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0]])
        result = _project_double_sphere(pts, xi=0.0, alpha=0.5,
                                        fx=400, fy=400, cx=320, cy=240)
        assert result is not None
        assert result.shape == (2, 2)


class TestTransforms:
    """变换工具测试。"""

    def test_rotation_euler_roundtrip(self):
        """测试旋转矩阵 ↔ 欧拉角转换。"""
        from unicalib.utils.transforms import (
            rotation_matrix_to_euler, euler_to_rotation_matrix)

        euler = np.array([10.0, -20.0, 30.0])  # ZYX degrees
        R = euler_to_rotation_matrix(euler)
        euler_back = rotation_matrix_to_euler(R)
        np.testing.assert_allclose(euler, euler_back, atol=1e-6)

    def test_rotation_distance(self):
        """测试旋转矩阵距离计算。"""
        from unicalib.utils.transforms import rotation_distance_deg
        from scipy.spatial.transform import Rotation

        R1 = np.eye(3)
        R2 = Rotation.from_euler("z", 15.0, degrees=True).as_matrix()
        dist = rotation_distance_deg(R1, R2)
        assert abs(dist - 15.0) < 0.001, f"Expected 15°, got {dist:.4f}°"

    def test_T_from_Rt(self):
        """测试 SE3 矩阵构建与分解。"""
        from unicalib.utils.transforms import T_from_Rt, Rt_from_T

        R = np.eye(3)
        t = np.array([1.0, 2.0, 3.0])
        T = T_from_Rt(R, t)
        R2, t2 = Rt_from_T(T)
        np.testing.assert_allclose(R, R2)
        np.testing.assert_allclose(t, t2)

    def test_quaternion_mean(self):
        """测试四元数平均。"""
        from unicalib.utils.transforms import quaternion_mean
        from scipy.spatial.transform import Rotation

        # 相近旋转的平均应接近其中任一
        rots = [Rotation.from_euler("z", a, degrees=True).as_quat()
                for a in [1.0, 2.0, 3.0]]
        mean_q = quaternion_mean(np.array(rots))
        mean_euler = Rotation.from_quat(mean_q).as_euler("z", degrees=True)
        assert abs(mean_euler[0] - 2.0) < 0.5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
