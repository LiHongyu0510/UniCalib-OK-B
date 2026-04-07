#!/usr/bin/env python3
"""
增强版标定结果可视化脚本
使用新的可视化功能：热力图、误差分布、交互式报告、3D可视化
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
    parser = argparse.ArgumentParser(
        description="UniCalib 增强版结果可视化",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 基础可视化（投影图、传感器概览）
  python visualize_results_v2.py --config config.yaml --results ./calib_results --data ./data
  
  # 完整可视化（包括误差分布、残差曲线）
  python visualize_results_v2.py --config config.yaml --results ./calib_results --data ./data --full
  
  # 实时可视化模式
  python visualize_results_v2.py --config config.yaml --results ./calib_results --data ./data --realtime
  
  # 生成交互式HTML报告
  python visualize_results_v2.py --config config.yaml --results ./calib_results --interactive
        """
    )
    parser.add_argument("--config", required=True, help="配置文件路径")
    parser.add_argument("--results", required=True, help="标定结果目录")
    parser.add_argument("--data", help="数据路径（用于投影可视化）")
    parser.add_argument("--output", default=None, help="输出目录（默认: results/viz_v2）")
    parser.add_argument("--full", action="store_true", help="生成完整可视化（包括误差分布、残差等）")
    parser.add_argument("--realtime", action="store_true", help="启用实时可视化（Open3D）")
    parser.add_argument("--interactive", action="store_true", help="生成交互式HTML报告")
    parser.add_argument("--n-frames", type=int, default=10, help="用于可视化的帧数")
    parser.add_argument("--error-threshold", type=float, default=1.0, help="误差合格阈值（像素）")
    
    args = parser.parse_args()

    import yaml
    import numpy as np
    from unicalib.core.data_manager import DataManager
    from unicalib.core.calib_result import CameraIntrinsic, CalibResult
    from unicalib.utils.visualization import CalibVisualizer
    from unicalib.utils.visualization_v2 import CalibVisualizerV2
    from unicalib.validation.interactive_report import InteractiveReportGenerator
    from unicalib.core.sensor_config import SensorConfig

    results_dir = Path(args.results)
    output_dir = Path(args.output) if args.output else results_dir / "viz_v2"
    output_dir.mkdir(parents=True, exist_ok=True)

    # 读取配置和标定结果
    with open(args.config) as f:
        config = yaml.safe_load(f)

    # 加载内参
    intr_yaml = results_dir / "intrinsics.yaml"
    extr_yaml = results_dir / "extrinsics.yaml"

    intrinsics = {}
    if intr_yaml.exists():
        with open(intr_yaml) as f:
            intr_data = yaml.safe_load(f)
        from unicalib.core.calib_result import CameraIntrinsic, FisheyeIntrinsic, IMUIntrinsic
        for sid, d in intr_data.items():
            if "K" in d:
                intrinsics[sid] = CameraIntrinsic.from_dict(d)
            elif "model_type" in d:
                intrinsics[sid] = FisheyeIntrinsic.from_dict(d)
            elif "gyro_noise" in d:
                intrinsics[sid] = IMUIntrinsic.from_dict(d)

    # 加载外参
    extrinsics = {}
    if extr_yaml.exists():
        with open(extr_yaml) as f:
            extr_data = yaml.safe_load(f)
        for key_str, d in extr_data.items():
            parts = key_str.split("_to_")
            if len(parts) == 2:
                sa, sb = parts
                extrinsics[(sa, sb)] = CalibResult.from_dict(d)

    # 构建传感器配置
    sensors = {}
    for s_cfg in config.get("sensors", []):
        sc = SensorConfig(**s_cfg)
        sensors[sc.sensor_id] = sc

    # 基础可视化器（V1）
    viz = CalibVisualizer(str(output_dir))
    
    # 增强版可视化器（V2）
    viz_v2 = CalibVisualizerV2(str(output_dir))
    
    # 交互式报告生成器
    if args.interactive:
        report_gen = InteractiveReportGenerator(str(output_dir))

    logger.info("=" * 70)
    logger.info("UniCalib 增强版可视化")
    logger.info("=" * 70)

    # 1. 投影对比图（使用V1基础版本）
    if args.data:
        logger.info("\n[1/6] 生成投影对比图...")
        data_mgr = DataManager(args.data)
        data_mgr.open()
        
        for (sa_id, sb_id), ext_result in extrinsics.items():
            sa = sensors.get(sa_id)
            sb = sensors.get(sb_id)
            if sa is None or sb is None:
                continue
            
            if sa.is_lidar() and sb.is_camera() and sb_id in intrinsics:
                intr = intrinsics[sb_id]
                if not isinstance(intr, CameraIntrinsic):
                    continue
                
                logger.info(f"  Processing {sa_id}-{sb_id}...")
                
                # 加载多帧数据
                images = []
                pts_list = []
                for ts, data_lidar, data_cam in data_mgr.iter_synced_frames(
                        sa.topic, sb.topic, max_pairs=args.n_frames):
                    images.append(data_cam)
                    pts_list.append(data_lidar)
                
                if len(images) > 0:
                    # 生成热力图
                    pts_cam = (ext_result.rotation @ pts_list[0][:, :3].T
                              + ext_result.translation.reshape(3, 1)).T
                    vis = viz_v2.draw_lidar_heatmap(
                        images[0], pts_cam, intr.K, intr.dist_coeffs)
                    out_path = output_dir / f"heatmap_{sa_id}_{sb_id}.jpg"
                    import cv2
                    cv2.imwrite(str(out_path), vis)
                    logger.info(f"    Saved heatmap: {out_path}")
        
        data_mgr.close()

    # 2. 误差分布图
    if args.full:
        logger.info("\n[2/6] 生成误差分布图...")
        
        # 加载验证结果
        val_yaml = results_dir / "validation.yaml"
        validation_metrics = {}
        if val_yaml.exists():
            with open(val_yaml) as f:
                val_data = yaml.safe_load(f)
            
            # 从验证数据中提取误差
            for key, metric_data in val_data.get('metrics', {}).items():
                errors = metric_data.get('all_errors', [])
                if errors:
                    errors_array = np.array(errors)
                    viz_v2.save_error_distribution_plot(
                        errors_array,
                        title=f"Reprojection Error Distribution - {key}",
                        filename=f"error_dist_{key}.png",
                        threshold=args.error_threshold,
                    )

    # 3. 传感器视锥图
    logger.info("\n[3/6] 生成传感器视锥图...")
    camera_ids = [sid for sid, s in sensors.items() 
                  if s.sensor_type.value in ['camera_pinhole', 'camera_fisheye']]
    lidar_ids = [sid for sid, s in sensors.items() 
                 if s.sensor_type.value == 'lidar']
    viz_v2.save_sensor_frustum(
        sensors, intrinsics, extrinsics,
        camera_ids=camera_ids,
        lidar_ids=lidar_ids,
        filename="sensor_frustums.png",
    )

    # 4. 多帧对比图
    if args.full and args.data:
        logger.info("\n[4/6] 生成多帧对比图...")
        data_mgr = DataManager(args.data)
        data_mgr.open()
        
        for (sa_id, sb_id), ext_result in extrinsics.items():
            sa = sensors.get(sa_id)
            sb = sensors.get(sb_id)
            if sa is None or sb is None:
                continue
            
            if sa.is_lidar() and sb.is_camera() and sb_id in intrinsics:
                intr = intrinsics[sb_id]
                if not isinstance(intr, CameraIntrinsic):
                    continue
                
                logger.info(f"  Generating multi-frame comparison for {sa_id}-{sb_id}...")
                
                images = []
                pts_list = []
                for ts, data_lidar, data_cam in data_mgr.iter_synced_frames(
                        sa.topic, sb.topic, max_pairs=4):
                    images.append(data_cam)
                    pts_list.append(data_lidar)
                
                if len(images) >= 4:
                    viz_v2.save_multi_frame_projection(
                        images[:4], pts_list[:4],
                        [ext_result for _ in range(4)],
                        intr.K, intr.dist_coeffs,
                        labels=[f"Frame {i+1}" for i in range(4)],
                        filename=f"multi_frame_{sa_id}_{sb_id}.jpg",
                    )
        
        data_mgr.close()

    # 5. 残差收敛曲线（如果存在迭代数据）
    if args.full:
        logger.info("\n[5/6] 生成残差收敛曲线...")
        residual_yaml = results_dir / "residuals.yaml"
        if residual_yaml.exists():
            with open(residual_yaml) as f:
                residual_data = yaml.safe_load(f)
            
            iteration_errors = residual_data.get('iteration_errors', [])
            if iteration_errors:
                viz_v2.save_residual_plot(
                    iteration_errors,
                    title="Calibration Optimization Residual Convergence",
                    filename="residual_convergence.png",
                )

    # 6. 交互式HTML报告
    if args.interactive:
        logger.info("\n[6/6] 生成交互式HTML报告...")
        
        # 加载验证指标
        val_yaml = results_dir / "validation.yaml"
        validation_metrics_dict = {'metrics': {}, 'overall_pass': True, 'summary': 'PASS'}
        visualization_data = {}
        
        if val_yaml.exists():
            with open(val_yaml) as f:
                val_data = yaml.safe_load(f)
            validation_metrics_dict = val_data
            
            # 构建可视化数据
            for key, metric_data in val_data.get('metrics', {}).items():
                errors = metric_data.get('all_errors', [])
                if errors:
                    visualization_data['error_distribution'] = {
                        'errors': errors
                    }
        
        # 加载残差数据
        residual_yaml = results_dir / "residuals.yaml"
        if residual_yaml.exists():
            with open(residual_yaml) as f:
                residual_data = yaml.safe_load(f)
            iteration_errors = residual_data.get('iteration_errors', [])
            if iteration_errors:
                visualization_data['residual_convergence'] = {
                    'iterations': list(range(1, len(iteration_errors) + 1)),
                    'errors': iteration_errors
                }
        
        # 生成交互式报告
        from unicalib.validation.reprojection import ReprojectionValidator
        report_gen.generate_interactive_html(
            intrinsics,
            extrinsics,
            validation_metrics_dict,
            visualization_data,
        )

    # 7. 实时可视化（如果启用）
    if args.realtime:
        logger.info("\n[实时模式] 启动实时可视化...")
        try:
            from unicalib.utils.realtime_visualizer import RealtimeVisualizer
            
            rt_viz = RealtimeVisualizer()
            if rt_viz.is_available():
                logger.info("  Open3D 实时可视化器已启动")
                logger.info("  按任意键退出...")
                
                # 添加传感器坐标系
                for (sa_id, sb_id), ext_result in extrinsics.items():
                    if sb_id == 'imu0':
                        continue
                    R = ext_result.rotation
                    t = ext_result.translation
                    rt_viz.update_coordinate_frame(
                        f"{sb_id}_frame", R, t, scale=1.0
                    )
                
                # 设置视角
                rt_viz.set_view_parameters(
                    front=(0.0, 0.0, -1.0),
                    lookat=(0.0, 0.0, 0.0),
                    up=(0.0, -1.0, 0.0),
                    zoom=0.5,
                )
                
                # 等待用户输入
                input("按 Enter 退出实时可视化...")
                rt_viz.close()
            else:
                logger.warning("  Open3D 不可用，跳过实时可视化")
        except Exception as e:
            logger.error(f"  实时可视化失败: {e}")

    # 生成传感器概览图（V1兼容）
    logger.info("\n生成传感器概览图...")
    viz.save_calib_overview(sensors, intrinsics, extrinsics)

    logger.info("\n" + "=" * 70)
    logger.info(f"可视化结果已保存至: {output_dir}")
    logger.info("生成的文件:")
    for f in sorted(output_dir.glob("*")):
        logger.info(f"  - {f.name}")
    logger.info("=" * 70)


if __name__ == "__main__":
    main()
