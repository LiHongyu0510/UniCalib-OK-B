#!/usr/bin/env python3
"""
标定结果可视化脚本
生成: 点云着色图、投影叠加图、BEV 图、传感器概览图
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
    parser = argparse.ArgumentParser(description="UniCalib 结果可视化")
    parser.add_argument("--config", required=True, help="配置文件路径")
    parser.add_argument("--results", required=True, help="标定结果目录")
    parser.add_argument("--data", required=True, help="数据路径")
    parser.add_argument("--output", default=None, help="输出目录 (默认: results/viz)")
    args = parser.parse_args()

    import yaml, numpy as np
    from unicalib.core.data_manager import DataManager
    from unicalib.core.calib_result import CameraIntrinsic, CalibResult
    from unicalib.utils.visualization import CalibVisualizer

    results_dir = Path(args.results)
    output_dir = Path(args.output) if args.output else results_dir / "viz"
    output_dir.mkdir(parents=True, exist_ok=True)

    # 读取标定结果
    with open(args.config) as f:
        config = yaml.safe_load(f)

    intr_yaml = results_dir / "intrinsics.yaml"
    extr_yaml = results_dir / "extrinsics.yaml"

    intrinsics = {}
    if intr_yaml.exists():
        with open(intr_yaml) as f:
            intr_data = yaml.safe_load(f)
        for sid, d in intr_data.items():
            if "K" in d:
                intrinsics[sid] = CameraIntrinsic.from_dict(d)

    extrinsics = {}
    if extr_yaml.exists():
        with open(extr_yaml) as f:
            extr_data = yaml.safe_load(f)
        for key_str, d in extr_data.items():
            parts = key_str.split("_to_")
            if len(parts) == 2:
                sa, sb = parts
                extrinsics[(sa, sb)] = CalibResult.from_dict(d)

    viz = CalibVisualizer(str(output_dir))
    data_mgr = DataManager(args.data)
    data_mgr.open()

    # 为每个 LiDAR-Camera 对生成投影图
    from unicalib.core.sensor_config import SensorConfig, SensorType
    sensors = {}
    for s_cfg in config.get("sensors", []):
        sc = SensorConfig(**s_cfg)
        sensors[sc.sensor_id] = sc

    for (sa_id, sb_id), ext_result in extrinsics.items():
        sa = sensors.get(sa_id)
        sb = sensors.get(sb_id)
        if sa is None or sb is None:
            continue

        if sa.is_lidar() and sb.is_camera() and sb_id in intrinsics:
            intr = intrinsics[sb_id]
            if not isinstance(intr, CameraIntrinsic):
                continue

            logger.info(f"Generating projection image for {sa_id}-{sb_id}...")
            for ts, data_lidar, data_cam in data_mgr.iter_synced_frames(
                    sa.topic, sb.topic, max_pairs=1):
                pts_cam = (ext_result.rotation @ data_lidar[:, :3].T
                           + ext_result.translation.reshape(3, 1)).T
                vis = viz.draw_lidar_on_image(
                    data_cam, pts_cam, intr.K, intr.dist_coeffs)
                import cv2
                out_path = output_dir / f"projection_{sa_id}_{sb_id}.jpg"
                cv2.imwrite(str(out_path), vis)
                logger.info(f"  Saved: {out_path}")
                break

    # 生成传感器概览图
    viz.save_calib_overview(sensors, intrinsics, extrinsics)

    data_mgr.close()
    logger.info(f"\n可视化结果已保存至: {output_dir}")


if __name__ == "__main__":
    main()
