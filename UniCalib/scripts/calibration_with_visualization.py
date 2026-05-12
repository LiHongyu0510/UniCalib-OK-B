#!/usr/bin/env python3
"""
标定与可视化集成示例
展示如何在标定过程中使用可视化功能
"""
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

logging.basicConfig(level=logging.INFO,
                    format="[%(asctime)s] %(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def example_basic_visualization():
    """示例1：基础可视化（静态图生成）"""
    logger.info("=" * 70)
    logger.info("示例1：基础可视化")
    logger.info("=" * 70)
    
    from unicalib.utils.visualization import CalibVisualizer
    
    viz = CalibVisualizer("./viz_example_basic")
    
    # 生成传感器概览图
    # 在实际使用中，这里传入真实的传感器配置、内参、外参
    logger.info("生成传感器概览图...")
    # viz.save_calib_overview(sensors, intrinsics, extrinsics)
    
    logger.info("✓ 基础可视化完成\n")


def example_enhanced_visualization():
    """示例2：增强版可视化（热力图、误差分布等）"""
    logger.info("=" * 70)
    logger.info("示例2：增强版可视化")
    logger.info("=" * 70)
    
    from unicalib.utils.visualization_v2 import CalibVisualizerV2
    import cv2
    import numpy as np
    
    viz = CalibVisualizerV2("./viz_example_enhanced")
    
    # 生成热力图投影
    logger.info("生成热力图投影...")
    img = cv2.imread("sample_image.jpg")
    pts_cam = np.random.randn(10000, 3) * 2.0
    pts_cam[:, 2] = np.abs(pts_cam[:, 2]) + 1.0  # 确保深度为正
    K = np.eye(3) * 500
    K[2, 2] = 1.0
    D = np.zeros(4)
    
    heatmap = viz.draw_lidar_heatmap(img, pts_cam, K, D)
    cv2.imwrite("./viz_example_enhanced/heatmap_example.jpg", heatmap)
    logger.info("✓ 热力图已保存")
    
    # 生成误差分布图
    logger.info("生成误差分布图...")
    errors = np.abs(np.random.randn(1000)) * 0.5
    viz.save_error_distribution_plot(
        errors,
        title="Reprojection Error Distribution Example",
        filename="error_dist_example.png",
        threshold=1.0,
    )
    logger.info("✓ 误差分布图已保存")
    
    # 生成传感器视锥图
    logger.info("生成传感器视锥图...")
    # 在实际使用中传入真实的传感器配置
    # viz.save_sensor_frustum(sensors, intrinsics, extrinsics)
    logger.info("✓ 传感器视锥图生成完成\n")


def example_realtime_visualization():
    """示例3：实时可视化（Open3D）"""
    logger.info("=" * 70)
    logger.info("示例3：实时可视化")
    logger.info("=" * 70)
    
    from unicalib.utils.realtime_visualizer import RealtimeVisualizer
    import time
    import numpy as np
    
    rt_viz = RealtimeVisualizer("UniCalib Example")
    
    if not rt_viz.is_available():
        logger.warning("Open3D 不可用，跳过实时可视化示例\n")
        return
    
    logger.info("实时可视化器已启动")
    logger.info("正在生成点云并更新...")
    
    # 模拟点云更新
    for i in range(10):
        points = np.random.randn(1000, 3) * 2.0
        
        # 深度着色
        depths = np.linalg.norm(points, axis=1)
        max_depth = np.percentile(depths, 95) + 1e-6
        norm_depth = np.clip(depths / max_depth, 0, 1)
        colors = np.zeros_like(points)
        colors[:, 0] = norm_depth
        colors[:, 1] = 1.0 - norm_depth
        colors[:, 2] = 0.2
        
        rt_viz.update_pointcloud("pointcloud", points, colors)
        
        logger.info(f"  更新第 {i+1} 帧")
        time.sleep(0.5)
    
    # 添加坐标系
    logger.info("添加坐标系...")
    R = np.eye(3)
    t = np.array([1.0, 0.5, 0.5])
    rt_viz.update_coordinate_frame("sensor_frame", R, t, scale=1.0)
    
    logger.info("按 Enter 键关闭可视化器...")
    input()
    
    rt_viz.close()
    logger.info("✓ 实时可视化已关闭\n")


def example_process_visualization():
    """示例4：标定过程可视化（残差收敛）"""
    logger.info("=" * 70)
    logger.info("示例4：标定过程可视化")
    logger.info("=" * 70)
    
    from unicalib.utils.realtime_visualizer import ProcessVisualizer
    import time
    
    proc_viz = ProcessVisualizer(max_iterations=50)
    
    if proc_viz.plt is None:
        logger.warning("matplotlib 不可用，跳过过程可视化示例\n")
        return
    
    logger.info("过程可视化器已启动")
    logger.info("模拟优化过程...")
    
    # 模拟优化过程（指数衰减）
    for i in range(30):
        error = 1.0 * np.exp(-i / 10.0) + 0.01 * np.random.randn()
        proc_viz.update(i + 1, error)
        
        if (i + 1) % 10 == 0:
            logger.info(f"  迭代 {i+1}, 残差: {error:.6f}")
        
        time.sleep(0.1)
    
    logger.info("保存残差曲线...")
    proc_viz.save("./viz_example_process/residual_convergence.png")
    
    input("按 Enter 键关闭图表...")
    proc_viz.close()
    logger.info("✓ 过程可视化已关闭\n")


def example_interactive_report():
    """示例5：交互式HTML报告"""
    logger.info("=" * 70)
    logger.info("示例5：交互式HTML报告")
    logger.info("=" * 70)
    
    from unicalib.validation.interactive_report import InteractiveReportGenerator
    from unicalib.core.calib_result import CameraIntrinsic, CalibResult
    import numpy as np
    
    report_gen = InteractiveReportGenerator("./viz_example_report")
    
    # 创建示例数据
    intrinsics = {
        "cam_front": CameraIntrinsic(
            K=np.array([[1000, 0, 960], [0, 1000, 540], [0, 0, 1]]),
            dist_coeffs=np.zeros(4),
            reprojection_error=0.35,
            method="opencv",
        ),
    }
    
    R = np.eye(3)
    t = np.array([1.5, 0.0, 0.5])
    extrinsics = {
        ("lidar0", "cam_front"): CalibResult(
            rotation=R,
            translation=t,
            time_offset=0.001,
            reprojection_error=0.42,
            method_used="ikalibr",
        ),
    }
    
    validation_metrics = {
        'metrics': {
            'lidar0-cam_front': type('Metric', (), {
                'mean_error_px': 0.42,
                'median_error_px': 0.38,
                'pct_within_1px': 95.2,
                'extra': {'pass': True}
            })(),
        },
        'overall_pass': True,
        'summary': 'Calibration Passed',
    }
    
    visualization_data = {
        'error_distribution': {
            'errors': (np.abs(np.random.randn(500)) * 0.4).tolist()
        },
        'residual_convergence': {
            'iterations': list(range(1, 31)),
            'errors': (1.0 * np.exp(-np.arange(30) / 10.0)).tolist()
        }
    }
    
    logger.info("生成交互式HTML报告...")
    html_path = report_gen.generate_interactive_html(
        intrinsics, extrinsics, validation_metrics, visualization_data
    )
    logger.info(f"✓ 报告已保存: {html_path}")
    logger.info("  在浏览器中打开查看交互式图表\n")


def example_hybrid_visualization():
    """示例6：混合可视化（3D + 过程图）"""
    logger.info("=" * 70)
    logger.info("示例6：混合可视化（3D + 过程图）")
    logger.info("=" * 70)
    
    from unicalib.utils.realtime_visualizer import HybridVisualizer
    import time
    import numpy as np
    
    hybrid_viz = HybridVisualizer(enable_3d=True, enable_process=True)
    
    if not hybrid_viz.rt_viz.is_available():
        logger.warning("Open3D 不可用，跳过3D可视化\n")
        hybrid_viz.close()
        return
    
    if hybrid_viz.proc_viz.plt is None:
        logger.warning("matplotlib 不可用，跳过过程可视化\n")
        hybrid_viz.close()
        return
    
    logger.info("混合可视化器已启动")
    logger.info("同时更新3D视图和过程图表...")
    
    # 模拟标定过程
    for i in range(20):
        # 更新3D点云
        points = np.random.randn(500, 3) * (2.0 - i * 0.05)
        hybrid_viz.update_pointcloud("pointcloud", points)
        
        # 更新过程图
        error = 1.0 * np.exp(-i / 8.0) + 0.02 * np.random.randn()
        hybrid_viz.update_process(error)
        
        if (i + 1) % 5 == 0:
            logger.info(f"  迭代 {i+1}, 残差: {error:.6f}")
        
        time.sleep(0.2)
    
    logger.info("\n按 Enter 键关闭所有可视化...")
    input()
    
    hybrid_viz.close()
    logger.info("✓ 混合可视化已关闭\n")


def main():
    """运行所有示例"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="UniCalib 可视化集成示例",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
可用示例:
  1 - 基础可视化（静态图）
  2 - 增强版可视化（热力图、误差分布）
  3 - 实时可视化（Open3D）
  4 - 标定过程可视化（残差收敛）
  5 - 交互式HTML报告
  6 - 混合可视化（3D + 过程图）
  all - 运行所有示例
        """
    )
    parser.add_argument(
        "example",
        choices=['1', '2', '3', '4', '5', '6', 'all'],
        help="要运行的示例编号"
    )
    
    args = parser.parse_args()
    
    examples = {
        '1': example_basic_visualization,
        '2': example_enhanced_visualization,
        '3': example_realtime_visualization,
        '4': example_process_visualization,
        '5': example_interactive_report,
        '6': example_hybrid_visualization,
    }
    
    if args.example == 'all':
        for idx, func in examples.items():
            try:
                func()
            except Exception as e:
                logger.error(f"示例 {idx} 执行失败: {e}")
    else:
        func = examples.get(args.example)
        if func:
            try:
                func()
            except Exception as e:
                logger.error(f"示例执行失败: {e}")
                import traceback
                traceback.print_exc()
        else:
            logger.error(f"未知的示例编号: {args.example}")


if __name__ == "__main__":
    main()
