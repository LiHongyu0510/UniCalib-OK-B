/**
 * UniCalib C++ — 可视化示例
 * 展示如何使用 VisualizationV2 进行增强版可视化
 */
#include "unicalib/visualization_v2.hpp"
#include "unicalib/calib_result.hpp"
#include "unicalib/logger.hpp"
#include <iostream>
#include <vector>

#ifdef UNICALIB_USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

using namespace unicalib;

int main() {
  // 初始化日志
  Logger::instance().init("visualization_example", LogLevel::INFO, "", true);
  
  std::cout << "UniCalib C++ 可视化示例\n";
  std::cout << "========================\n\n";

  // 创建可视化器
  VisualizationV2 viz("./viz_output_cpp");
  
  // 配置可视化参数
  VisualizationConfig config;
  config.heatmap.max_depth = 50.0;
  config.heatmap.point_size = 3;
  config.heatmap.alpha = 0.7;
  config.error_distribution.threshold = 1.0;
  config.error_distribution.bins = 50;
  viz.SetConfig(config);

  // ========================================
  // 示例 1: 误差分布图
  // ========================================
  std::cout << "示例1: 生成误差分布图...\n";
  std::vector<double> errors;
  for (int i = 0; i < 1000; ++i) {
    double error = 0.5 * std::abs(std::rand()) / RAND_MAX;
    errors.push_back(error);
  }
  
  viz.SaveErrorDistributionPlot(
    errors,
    "Reprojection Error Distribution (C++)",
    "error_distribution_cpp.png",
    1.0
  );
  std::cout << "  ✓ 完成\n\n";

  // ========================================
  // 示例 2: 残差收敛曲线
  // ========================================
  std::cout << "示例2: 生成残差收敛曲线...\n";
  std::vector<double> iteration_errors;
  for (int i = 0; i < 50; ++i) {
    double error = 1.0 * std::exp(-i / 10.0) + 0.01 * (std::rand() / RAND_MAX - 0.5);
    iteration_errors.push_back(error);
  }
  
  viz.SaveResidualPlot(
    iteration_errors,
    "Calibration Optimization Residual Convergence (C++)",
    "residual_convergence_cpp.png"
  );
  std::cout << "  ✓ 完成\n\n";

#ifdef UNICALIB_USE_OPENCV
  // ========================================
  // 示例 3: 热力图投影
  // ========================================
  std::cout << "示例3: 生成热力图投影...\n";
  
  // 创建示例图像（模拟相机图像）
  cv::Mat img(1080, 1920, CV_8UC3, cv::Scalar(100, 100, 100));
  cv::rectangle(img, cv::Rect(500, 300, 920, 480), cv::Scalar(150, 150, 150), -1);
  cv::putText(img, "UniCalib C++ Visualization", cv::Point(600, 540),
              cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 3);
  
  // 创建示例点云（模拟LiDAR点云）
  Eigen::MatrixXd pts_cam(10000, 3);
  for (int i = 0; i < 10000; ++i) {
    pts_cam(i, 0) = (std::rand() / RAND_MAX - 0.5) * 20.0;  // X: -10 ~ 10m
    pts_cam(i, 1) = (std::rand() / RAND_MAX - 0.5) * 20.0;  // Y: -10 ~ 10m
    pts_cam(i, 2) = 2.0 + (std::rand() / RAND_MAX) * 48.0;   // Z: 2 ~ 50m
  }
  
  // 相机内参
  Eigen::Matrix3d K;
  K << 1000, 0, 960,
       0, 1000, 540,
       0, 0, 1;
  
  // 畸变系数
  Eigen::Vector4d D;
  D << 0, 0, 0, 0;
  
  // 生成热力图
  cv::Mat heatmap = viz.DrawLidarHeatmap(img, pts_cam, K, D, config);
  cv::imwrite("./viz_output_cpp/heatmap_cpp.jpg", heatmap);
  std::cout << "  ✓ 完成\n\n";

  // ========================================
  // 示例 4: 多帧对比图
  // ========================================
  std::cout << "示例4: 生成多帧对比图...\n";
  
  std::vector<cv::Mat> images;
  std::vector<Eigen::MatrixXd> pts_list;
  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> extrinsics_list;
  std::vector<std::string> labels = {"Frame 1", "Frame 2", "Frame 3", "Frame 4"};
  
  for (int i = 0; i < 4; ++i) {
    images.push_back(img.clone());
    pts_list.push_back(pts_cam);
    
    // 模拟不同的外参
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(i * 0.1, 0, 0);
    extrinsics_list.push_back({R, t});
  }
  
  viz.SaveMultiFrameProjection(
    images, pts_list, extrinsics_list,
    K, D, labels,
    "multi_frame_cpp.jpg"
  );
  std::cout << "  ✓ 完成\n\n";
#endif

  // ========================================
  // 示例 5: 点云对齐可视化
  // ========================================
  std::cout << "示例5: 生成点云对齐可视化...\n";
  
  Eigen::MatrixXd pts_source(1000, 3);
  Eigen::MatrixXd pts_target(1000, 3);
  
  for (int i = 0; i < 1000; ++i) {
    double x = (std::rand() / RAND_MAX - 0.5) * 10.0;
    double y = (std::rand() / RAND_MAX - 0.5) * 10.0;
    double z = (std::rand() / RAND_MAX - 0.5) * 10.0;
    
    pts_source(i, 0) = x;
    pts_source(i, 1) = y;
    pts_source(i, 2) = z;
    
    pts_target(i, 0) = x + 1.5;  // 平移 (1.5, 0, 0)
    pts_target(i, 1) = y + 0.5;
    pts_target(i, 2) = z + 0.3;
  }
  
  // 变换矩阵
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t(1.5, 0.5, 0.3);
  
  viz.SavePointcloudAlignment(
    pts_source, pts_target,
    {R, t},
    "pointcloud_alignment_cpp.png"
  );
  std::cout << "  ✓ 完成\n\n";

  std::cout << "========================\n";
  std::cout << "所有可视化示例完成！\n";
  std::cout << "输出目录: ./viz_output_cpp/\n";
  std::cout << "\n生成的文件:\n";
  std::cout << "  - error_distribution_cpp.png\n";
  std::cout << "  - residual_convergence_cpp.png\n";
#ifdef UNICALIB_USE_OPENCV
  std::cout << "  - heatmap_cpp.jpg\n";
  std::cout << "  - multi_frame_cpp.jpg\n";
#endif
  std::cout << "  - pointcloud_alignment_cpp.png\n";

  return 0;
}
