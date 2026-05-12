#!/usr/bin/env python3
"""
数据采集辅助脚本
实时检查传感器数据质量，提示用户做足够的运动激励
"""
import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

logging.basicConfig(level=logging.INFO,
                    format="[%(asctime)s] %(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="UniCalib 数据采集质量检查")
    parser.add_argument("--config", required=True, help="配置文件路径")
    parser.add_argument("--data", required=True, help="rosbag 或数据目录路径")
    parser.add_argument("--check-only", action="store_true",
                        help="仅检查数据质量，不进行标定")
    args = parser.parse_args()

    from unicalib.core.data_manager import DataManager
    import yaml, numpy as np

    with open(args.config) as f:
        config = yaml.safe_load(f)

    sensors = config.get("sensors", [])
    data_mgr = DataManager(args.data)
    data_mgr.open()

    print("\n" + "=" * 60)
    print("  UniCalib 数据质量检查报告")
    print("=" * 60)

    for sensor_cfg in sensors:
        sid = sensor_cfg["sensor_id"]
        stype = sensor_cfg["sensor_type"]
        topic = sensor_cfg["topic"]

        print(f"\n[{sid}] type={stype} topic={topic}")

        if stype == "imu":
            imu_data = data_mgr.load_imu_data(topic)
            if imu_data:
                n = len(imu_data["timestamps"])
                duration = imu_data["timestamps"][-1] - imu_data["timestamps"][0]
                rate = imu_data["sample_rate"]
                accel_std = np.std(imu_data["accel"], axis=0)
                gyro_std = np.std(imu_data["gyro"], axis=0)
                print(f"  ✓ {n} 样本, {duration:.1f}s @ {rate:.1f}Hz")
                print(f"  加速度标准差: {accel_std} m/s²")
                print(f"  陀螺仪标准差: {gyro_std} rad/s")

                max_accel = np.max(np.abs(imu_data["accel"]))
                if max_accel < 1.0:
                    print("  ⚠ 警告: 运动激励不足 (最大加速度 < 1 m/s²)")
                else:
                    print(f"  ✓ 运动激励充足 (最大加速度 {max_accel:.2f} m/s²)")

                if duration < 60:
                    print(f"  ⚠ 数据时长较短 ({duration:.1f}s)，建议 ≥ 120s")
                else:
                    print(f"  ✓ 数据时长: {duration:.1f}s")
            else:
                print("  ✗ 无法读取 IMU 数据")

        elif stype in ("camera_pinhole", "camera_fisheye"):
            count = sum(1 for _ in data_mgr.iter_images(topic, max_frames=500))
            print(f"  图像帧数: {count}")
            if count < 30:
                print(f"  ⚠ 图像帧数不足 (< 30)")
            else:
                print(f"  ✓ 图像帧数充足")

        elif stype == "lidar":
            count = sum(1 for _ in data_mgr.iter_pointclouds(topic, max_frames=200))
            print(f"  点云帧数: {count}")
            if count < 20:
                print(f"  ⚠ 点云帧数不足 (< 20)")
            else:
                print(f"  ✓ 点云帧数充足")

    data_mgr.close()
    print("\n" + "=" * 60)
    print("  数据检查完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
