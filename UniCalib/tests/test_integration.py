"""
集成测试 —— 完整标定流水线的端到端验证
使用合成数据（无需实际传感器）
"""
import sys
import os
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pytest
import yaml
import cv2


def make_synthetic_dataset(tmpdir: str, n_images: int = 5,
                             n_lidar_frames: int = 10,
                             n_imu_samples: int = 2000):
    """
    生成用于集成测试的合成数据集。

    目录结构:
      tmpdir/
        camera_front_image_raw/   ← PNG 图像
        velodyne_points/           ← binary float32 点云
        imu.csv                    ← IMU 数据
    """
    # 相机图像（含棋盘格）
    cam_dir = os.path.join(tmpdir, "camera_front_image_raw")
    os.makedirs(cam_dir)
    W, H = 640, 480
    K = np.array([[500, 0, W/2], [0, 500, H/2], [0, 0, 1]], dtype=np.float64)

    pattern = (10, 7)
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= 30.0
    rng = np.random.default_rng(0)

    for i in range(n_images):
        rvec = rng.uniform(-0.2, 0.2, 3)
        tvec = np.array([rng.uniform(-50, 50), rng.uniform(-50, 50), 350.0])
        pts_2d, _ = cv2.projectPoints(objp, rvec, tvec, K, np.zeros(5))
        pts_2d = pts_2d.reshape(-1, 2)

        img = np.full((H, W, 3), 128, dtype=np.uint8)
        for j, p in enumerate(pts_2d.astype(int)):
            row_idx = j // pattern[0]
            col_idx = j % pattern[0]
            if row_idx < pattern[1] - 1 and col_idx < pattern[0] - 1:
                p1 = tuple(pts_2d[j].astype(int))
                p2 = tuple(pts_2d[j + 1].astype(int))
                p3 = tuple(pts_2d[j + pattern[0]].astype(int))
                p4 = tuple(pts_2d[j + pattern[0] + 1].astype(int))
                color = 255 if (row_idx + col_idx) % 2 == 0 else 0
                cv2.fillPoly(img, [np.array([p1, p2, p4, p3])], (color, color, color))
        cv2.imwrite(os.path.join(cam_dir, f"{i * 0.1:.6f}.png"), img)

    # 点云
    lidar_dir = os.path.join(tmpdir, "velodyne_points")
    os.makedirs(lidar_dir)
    for i in range(n_lidar_frames):
        pts = rng.uniform(-5, 5, (200, 3)).astype(np.float32)
        pts[:, 2] = np.abs(pts[:, 2]) + 0.5  # 保证 z > 0
        pts.tofile(os.path.join(lidar_dir, f"{i * 0.1:.6f}.bin"))

    # IMU CSV
    import csv
    imu_csv = os.path.join(tmpdir, "imu.csv")
    g = 9.81
    with open(imu_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "gx", "gy", "gz", "ax", "ay", "az"])
        for i in range(n_imu_samples):
            t = i / 200.0
            gx = np.sin(t) * 0.1 + rng.uniform(-0.001, 0.001)
            gy = np.cos(t) * 0.05 + rng.uniform(-0.001, 0.001)
            gz = rng.uniform(-0.001, 0.001)
            ax = rng.uniform(-0.1, 0.1)
            ay = rng.uniform(-0.1, 0.1)
            az = g + rng.uniform(-0.1, 0.1)
            writer.writerow([t, gx, gy, gz, ax, ay, az])

    return tmpdir


def make_test_config(tmpdir: str, output_dir: str) -> str:
    """生成最小化测试配置文件。"""
    config = {
        "system": {
            "name": "test_calib",
            "output_dir": output_dir,
            "log_level": "WARNING",
        },
        "sensors": [
            {
                "sensor_id": "imu0",
                "sensor_type": "imu",
                "topic": "/imu/data",
                "frame_id": "imu_link",
                "rate": 200,
            },
            {
                "sensor_id": "lidar0",
                "sensor_type": "lidar",
                "topic": "/velodyne_points",
                "frame_id": "velodyne",
                "lidar_type": "spinning",
            },
            {
                "sensor_id": "cam_front",
                "sensor_type": "camera_pinhole",
                "topic": "/camera/front/image_raw",
                "frame_id": "camera_front",
                "resolution": [640, 480],
            },
        ],
        "data_collection": {
            "camera_intrinsic": {
                "checkerboard": {"rows": 8, "cols": 11, "square_size": 0.03}
            }
        },
        "calibration": {
            "stage1_intrinsic": {
                "camera_pinhole": {
                    "primary_method": "opencv",
                    "refinement_method": "opencv",
                    "min_reproj_error": 2.0,
                },
                "imu": {"method": "allan_only"},
            },
            "stage2_coarse": {
                "imu_lidar": {"method": "l2calib_rl_init"},
                "lidar_camera": {"method": "mias_lcec_coarse"},
            },
            "stage3_fine": {
                "global_optimization": {
                    "method": "ikalibr",
                    "spline_order": 4,
                    "knot_distance_so3": 0.02,
                    "knot_distance_pos": 0.02,
                    "max_iterations": 5,
                    "enable_time_offset": True,
                },
                "lidar_camera_refinement": {
                    "method": "mias_lcec_fine",
                    "use_imu_motion_compensation": False,
                    "optimize_time_offset": False,
                },
            },
            "stage4_validation": {
                "thresholds": {"reproj_error_px": 5.0}
            },
        },
    }
    cfg_path = os.path.join(tmpdir, "test_config.yaml")
    with open(cfg_path, "w") as f:
        yaml.dump(config, f)
    return cfg_path


class TestDataManager:
    """DataManager 目录模式测试。"""

    def test_iter_images(self):
        from unicalib.core.data_manager import DataManager
        with tempfile.TemporaryDirectory() as tmpdir:
            make_synthetic_dataset(tmpdir, n_images=3)
            data_mgr = DataManager(tmpdir)
            data_mgr.open()
            frames = list(data_mgr.iter_images("/camera/front/image_raw",
                                                max_frames=10))
            data_mgr.close()
        assert len(frames) == 3, f"Expected 3 images, got {len(frames)}"
        ts, img = frames[0]
        assert img is not None
        assert img.ndim == 3

    def test_iter_pointclouds(self):
        from unicalib.core.data_manager import DataManager
        with tempfile.TemporaryDirectory() as tmpdir:
            make_synthetic_dataset(tmpdir, n_lidar_frames=5)
            data_mgr = DataManager(tmpdir)
            data_mgr.open()
            frames = list(data_mgr.iter_pointclouds("/velodyne_points", max_frames=10))
            data_mgr.close()
        assert len(frames) == 5
        ts, pts = frames[0]
        assert pts.ndim == 2
        assert pts.shape[1] >= 3

    def test_load_imu_data(self):
        from unicalib.core.data_manager import DataManager
        with tempfile.TemporaryDirectory() as tmpdir:
            make_synthetic_dataset(tmpdir, n_imu_samples=400)
            data_mgr = DataManager(tmpdir)
            data_mgr.open()
            imu = data_mgr.load_imu_data("/imu/data")
            data_mgr.close()
        assert imu is not None
        assert len(imu["timestamps"]) == 400
        assert imu["gyro"].shape == (400, 3)
        assert imu["accel"].shape == (400, 3)
        assert 150 < imu["sample_rate"] < 250


class TestIntrinsicPipelineSmoke:
    """内参标定流水线冒烟测试 (使用合成数据)。"""

    def test_camera_pinhole_intrinsic(self):
        """测试针孔相机内参标定流程。"""
        from unicalib.core.data_manager import DataManager
        from unicalib.core.sensor_config import SensorConfig, SensorType
        from unicalib.intrinsic.camera_pinhole import CameraPinholeCalibrator
        from unicalib.core.calib_result import CameraIntrinsic

        config = {
            "calibration": {"stage1_intrinsic": {"camera_pinhole": {
                "primary_method": "opencv",
                "refinement_method": "opencv",
                "min_reproj_error": 2.0,
            }}},
            "data_collection": {"camera_intrinsic": {"checkerboard": {
                "rows": 8, "cols": 11, "square_size": 0.03
            }}},
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            make_synthetic_dataset(tmpdir, n_images=15)
            sensor = SensorConfig(
                sensor_id="cam_front",
                sensor_type=SensorType.CAMERA_PINHOLE,
                topic="/camera/front/image_raw",
                frame_id="cam",
                resolution=[640, 480],
            )
            data_mgr = DataManager(tmpdir)
            data_mgr.open()
            calib = CameraPinholeCalibrator(config)
            result = calib.calibrate(data_mgr, sensor)
            data_mgr.close()

        # 结果应是合理的针孔内参
        assert isinstance(result, CameraIntrinsic)
        assert result.K[0, 0] > 100  # fx 合理范围
        assert result.K[1, 1] > 100  # fy 合理范围


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
