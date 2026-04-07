/**
 * UniCalib C++ — 实时可视化示例
 * 展示如何使用 RealtimeVisualizer 和 InteractiveReportGenerator
 */
#include "unicalib/realtime_visualizer.hpp"
#include "unicalib/interactive_report.hpp"
#include "unicalib/logger.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

using namespace unicalib;

int main() {
  // 初始化日志
  Logger::instance().init("realtime_viz_example", LogLevel::INFO, "", true);
  
  std::cout << "UniCalib C++ 实时可视化示例\n";
  std::cout << "==============================\n\n";

  // ========================================
  // 示例 1: 实时点云可视化
  // ========================================
  std::cout << "示例1: 实时点云可视化...\n";
  
  RealtimeVisualizer rt_viz("UniCalib Realtime Visualization");
  
  if (rt_viz.is_available()) {
    std::cout << "  Open3D 可视化器已启动\n";
    std::cout << "  正在生成点云并更新...\n";
    
    // 模拟实时点云更新
    for (int frame = 0; frame < 10; ++frame) {
      Eigen::MatrixXd points(1000, 3);
      Eigen::MatrixXd colors(1000, 3);
      
      for (int i = 0; i < 1000; ++i) {
        // 随机点云坐标
        points(i, 0) = (std::rand() / RAND_MAX - 0.5) * 10.0;
        points(i, 1) = (std::rand() / RAND_MAX - 0.5) * 10.0;
        points(i, 2) = (std::rand() / RAND_MAX - 0.5) * 10.0;
        
        // 深度着色
        double depth = std::sqrt(points(i, 0) * points(i, 0) +
                              points(i, 1) * points(i, 1) +
                              points(i, 2) * points(i, 2));
        double norm_depth = std::min(depth / 5.0, 1.0);
        
        colors(i, 0) = norm_depth;          // R: 近=红
        colors(i, 1) = 1.0 - norm_depth;    // G: 远=绿
        colors(i, 2) = 0.2;               // B: 固定蓝
      }
      
      // 更新点云
      rt_viz.update_pointcloud("pointcloud", points, colors, 2.0);
      
      std::cout << "    帧 " << (frame + 1) << ": 更新1000个点\n";
      
      // 暂停一下
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    std::cout << "  ✓ 完成\n\n";
    
  } else {
    std::cout << "  Open3D 不可用，跳过实时可视化\n\n";
  }

  // ========================================
  // 示例 2: 添加相机视锥
  // ========================================
  std::cout << "示例2: 添加相机视锥...\n";
  
  if (rt_viz.is_available()) {
    // 相机内参
    Eigen::Matrix3d K;
    K << 1000, 0, 960,
         0, 1000, 540,
         0, 0, 1;
    
    // 相机外参
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(1.5, 0.0, 0.5);
    
    std::pair<int, int> image_size(1920, 1080);
    Eigen::Vector3d color(0.0, 1.0, 1.0);  // 青色
    
    rt_viz.update_frustum("camera_frustum", R, t, K, image_size, 5.0, color);
    
    std::cout << "  ✓ 相机视锥已添加\n\n";
  }

  // ========================================
  // 示例 3: 添加坐标系
  // ========================================
  std::cout << "示例3: 添加坐标系...\n";
  
  if (rt_viz.is_available()) {
    // IMU坐标系（参考系）
    Eigen::Matrix3d R_imu = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_imu = Eigen::Vector3d::Zero();
    rt_viz.update_coordinate_frame("imu_frame", R_imu, t_imu, 1.0);
    
    // LiDAR坐标系
    Eigen::Matrix3d R_lidar = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_lidar(1.5, 0.5, 0.3);
    rt_viz.update_coordinate_frame("lidar_frame", R_lidar, t_lidar, 1.0);
    
    // 相机坐标系
    Eigen::Matrix3d R_cam = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_cam(-1.0, 0.3, 0.2);
    rt_viz.update_coordinate_frame("camera_frame", R_cam, t_cam, 1.0);
    
    std::cout << "  ✓ 3个坐标系已添加\n\n";
  }

  // ========================================
  // 示例 4: 添加轨迹线
  // ========================================
  std::cout << "示例4: 添加轨迹线...\n";
  
  if (rt_viz.is_available()) {
    Eigen::MatrixXd trajectory(100, 3);
    Eigen::Vector3d color(1.0, 1.0, 0.0);  // 黄色
    
    // 生成圆形轨迹
    for (int i = 0; i < 100; ++i) {
      double angle = i * 2 * M_PI / 100.0;
      trajectory(i, 0) = 3.0 * std::cos(angle);  // X
      trajectory(i, 1) = 3.0 * std::sin(angle);  // Y
      trajectory(i, 2) = 1.0;                   // Z
    }
    
    rt_viz.add_trajectory("trajectory", trajectory, color, 2.0);
    
    std::cout << "  ✓ 轨迹线已添加\n\n";
  }

  // ========================================
  // 示例 5: 设置相机视角
  // ========================================
  std::cout << "示例5: 设置相机视角...\n";
  
  if (rt_viz.is_available()) {
    Eigen::Vector3d front(0.0, 0.0, -1.0);
    Eigen::Vector3d lookat(0.0, 0.0, 0.0);
    Eigen::Vector3d up(0.0, -1.0, 0.0);
    
    rt_viz.set_view_parameters(front, lookat, up, 0.5);
    
    std::cout << "  ✓ 相机视角已设置\n\n";
  }

  // ========================================
  // 示例 6: 交互式报告生成
  // ========================================
  std::cout << "示例6: 生成交互式报告...\n";
  
  // 创建示例数据
  std::unordered_map<std::string, IntrinsicResultHolder> intrinsics;
  
  // 相机内参
  CameraIntrinsic cam_intrinsic;
  cam_intrinsic.K << 1000, 0, 960,
                      0, 1000, 540,
                      0, 0, 1;
  cam_intrinsic.dist_coeffs = {0.0, 0.0, 0.0, 0.0};
  cam_intrinsic.reprojection_error = 0.35;
  cam_intrinsic.method = "dm_calib";
  
  IntrinsicResultHolder cam_holder;
  cam_holder.camera_pinhole = cam_intrinsic;
  intrinsics["cam_front"] = cam_holder;
  
  // 外参结果
  std::unordered_map<std::string, CalibResult> extrinsics;
  
  CalibResult extr_result;
  extr_result.pair = {"lidar0", "cam_front"};
  extr_result.rotation = Eigen::Matrix3d::Identity();
  extr_result.translation = Eigen::Vector3d(1.5, 0.0, 0.5);
  extr_result.time_offset = 0.001;
  extr_result.reprojection_error = 0.42;
  extr_result.method_used = "ikalibr";
  
  extrinsics["lidar0_to_cam_front"] = extr_result;
  
  // 验证报告
  ValidationReport validation;
  validation.overall_pass = true;
  validation.summary = "Calibration Passed";
  
  ValidationMetrics metric;
  metric.mean_error_px = 0.42;
  metric.median_error_px = 0.38;
  metric.pct_within_1px = 95.2;
  metric.extra["pass"] = 1.0;
  
  validation.metrics["lidar0-cam_front"] = metric;
  
  // 生成交互式报告
  InteractiveReportGenerator report_gen("./viz_output_cpp_realtime");
  std::string html_path = report_gen.generate_interactive_html(
    intrinsics, extrinsics, validation, InteractiveReportConfig()
  );
  
  std::cout << "  ✓ 交互式报告已生成: " << html_path << "\n\n";
  
  std::cout << "==============================\n";
  std::cout << "所有示例完成！\n";
  std::cout << "\n输出目录: ./viz_output_cpp_realtime/\n";
  std::cout << "生成的文件:\n";
  std::cout << "  - interactive_calibration_report.html\n";
  
  if (rt_viz.is_available()) {
    std::cout << "\n提示: 按 Enter 键关闭实时可视化窗口...\n";
    std::cin.get();
    rt_viz.close();
  }
  
  return 0;
}
