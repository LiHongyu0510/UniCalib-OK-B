/**
 * @file optimization_example.cpp
 * @brief 标定精度优化示例程序
 * 
 * 演示如何使用:
 *   1. IterativeRefiner - 迭代精化
 *   2. TemporalOffsetEstimator - 时间偏移估计
 *   3. MultiResolutionOptimizer - 多分辨率优化
 *   4. LoopClosureValidator - 闭环一致性验证
 */
#include <iostream>
#include <Eigen/Dense>
#include "unicalib/iterative_refinement.hpp"
#include "unicalib/temporal_calibration.hpp"
#include "unicalib/transforms.hpp"

using namespace unicalib;

// =============================================================================
// 示例 1: 迭代精化
// =============================================================================
void demoIterativeRefinement() {
  std::cout << "\n=== 示例 1: 迭代精化 ===" << std::endl;
  
  // 配置
  RefinementConfig config;
  config.max_iterations = 5;
  config.convergence_threshold = 0.0005;
  config.translation_threshold = 0.0005;
  
  IterativeRefiner refiner(config);
  
  // 初始外参 (有误差)
  Eigen::Matrix3d initial_R = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d initial_t(0.05, 0.02, 0.01);  // 5cm 误差
  double initial_td = 0.01;  // 10ms 时间偏移
  
  // 真实外参 (用于生成模拟数据)
  Eigen::Matrix3d true_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d true_t(0.1, 0.0, 0.0);
  double true_td = 0.0;
  
  // 定义代价函数 (模拟 LiDAR-Camera 重投影误差)
  auto cost_function = [&](const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double td) -> double {
    // 误差 = ||R - true_R|| + ||t - true_t|| + |td - true_td|
    double R_err = IterativeRefiner::rotationDistance(R, true_R);
    double t_err = (t - true_t).norm();
    double td_err = std::abs(td - true_td);
    return R_err * R_err + t_err * t_err + td_err * td_err * 1000.0;
  };
  
  // 执行迭代精化
  auto [R_opt, t_opt, td_opt, history] = refiner.refine(
      initial_R, initial_t, initial_td, cost_function);
  
  // 输出结果
  std::cout << "初始状态:" << std::endl;
  std::cout << "  R_euler: " << rotation_matrix_to_euler(initial_R).transpose() << " deg" << std::endl;
  std::cout << "  t: " << initial_t.transpose() << " m" << std::endl;
  std::cout << "  td: " << initial_td * 1000.0 << " ms" << std::endl;
  
  std::cout << "\n优化后状态:" << std::endl;
  std::cout << "  R_euler: " << rotation_matrix_to_euler(R_opt).transpose() << " deg" << std::endl;
  std::cout << "  t: " << t_opt.transpose() << " m" << std::endl;
  std::cout << "  td: " << td_opt * 1000.0 << " ms" << std::endl;
  
  std::cout << "\n迭代历史:" << std::endl;
  for (const auto& iter : history) {
    std::cout << "  " << iter.toString() << std::endl;
  }
  
  if (refiner.isConverged()) {
    std::cout << "\n✓ 优化收敛!" << std::endl;
  } else {
    std::cout << "\n✗ 未收敛,可能需要更多迭代" << std::endl;
  }
}

// =============================================================================
// 示例 2: 时间偏移估计
// =============================================================================
void demoTemporalOffsetEstimation() {
  std::cout << "\n=== 示例 2: 时间偏移估计 ===" << std::endl;
  
  TemporalCalibConfig config;
  config.max_offset_ms = 50.0;
  config.sliding_window_size = 100;
  
  TemporalOffsetEstimator estimator(config);
  
  // 生成模拟数据: 真实时间偏移 = 15ms
  double true_offset_ms = 15.0;
  
  std::vector<IMUDataPoint> imu_data;
  std::vector<FeatureVelocityData> feature_data;
  
  for (int i = 0; i < 200; ++i) {
    double t = i * 0.01;  // 100Hz
    
    // IMU 数据
    IMUDataPoint imu;
    imu.timestamp = t;
    imu.gyro = Eigen::Vector3d(0.1 * std::sin(t), 0.05 * std::cos(t), 0.02);
    imu.accel = Eigen::Vector3d(0.5, 0.3, 9.8);
    imu_data.push_back(imu);
    
    // 特征数据 (有偏移)
    FeatureVelocityData feat;
    feat.timestamp = t + true_offset_ms / 1000.0;  // 添加偏移
    feat.pixel_velocity = Eigen::Vector2d(
        imu.gyro.y() * 500.0 + 0.1 * (rand() % 100 - 50) / 50.0,  // 加入噪声
        -imu.gyro.x() * 500.0 + 0.1 * (rand() % 100 - 50) / 50.0);
    feat.feature_id = i % 10;
    feature_data.push_back(feat);
  }
  
  // 估计时间偏移
  auto result = estimator.estimateFromFeatureVelocity(feature_data, imu_data);
  
  std::cout << "估计结果:" << std::endl;
  std::cout << "  " << result.toString() << std::endl;
  std::cout << "  真实偏移: " << true_offset_ms << " ms" << std::endl;
  std::cout << "  误差: " << std::abs(result.offset_ms - true_offset_ms) << " ms" << std::endl;
  
  if (result.confidence > 0.5) {
    std::cout << "\n✓ 高置信度估计" << std::endl;
  }
}

// =============================================================================
// 示例 3: 多分辨率优化
// =============================================================================
void demoMultiResolutionOptimization() {
  std::cout << "\n=== 示例 3: 多分辨率优化 ===" << std::endl;
  
  MultiResolutionOptimizer optimizer(3, {4.0, 2.0, 1.0});
  
  // 模拟参数
  Eigen::VectorXd initial_params(6);
  initial_params << 0.1, 0.05, 0.02, 0.1, 0.0, 0.05;  // [rotvec(3), trans(3)]
  
  // 代价函数工厂
  auto cost_factory = [](double knot_distance) -> IterativeRefiner::CostFunction {
    return [knot_distance](const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double td) -> double {
      // 简化的代价函数
      // 实际应基于 B-spline 轨迹和测量残差
      double R_err = R.trace() - 3.0;
      double t_err = t.norm();
      return (R_err * R_err + t_err * t_err) * (1.0 / knot_distance);
    };
  };
  
  // 执行多分辨率优化
  auto [params, levels] = optimizer.optimizeMultiresolution(
      initial_params, cost_factory, 0.02);
  
  std::cout << "各层级结果:" << std::endl;
  for (const auto& level : levels) {
    std::cout << "  Level " << level.level 
              << ": knot_dist=" << level.knot_distance 
              << "s, cost=" << level.cost << std::endl;
  }
}

// =============================================================================
// 示例 4: 闭环一致性验证
// =============================================================================
void demoLoopClosureValidation() {
  std::cout << "\n=== 示例 4: 闭环一致性验证 ===" << std::endl;
  
  LoopClosureValidator validator;
  
  // 构建外参三元组: IMU-LiDAR, LiDAR-Camera, Camera-IMU
  std::unordered_map<std::string, Eigen::Matrix4d> extrinsics;
  
  // T_IMU_LiDAR (有误差)
  Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
  T_imu_lidar.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_imu_lidar.block<3, 1>(0, 3) = Eigen::Vector3d(0.1, 0.0, 0.0);
  extrinsics["imu:lidar"] = T_imu_lidar;
  
  // T_LiDAR_Camera (有误差)
  Eigen::Matrix4d T_lidar_cam = Eigen::Matrix4d::Identity();
  T_lidar_cam.block<3, 3>(0, 0) = Eigen::AngleAxisd(-0.005, Eigen::Vector3d::UnitY()).toRotationMatrix();
  T_lidar_cam.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.1, 0.0);
  extrinsics["lidar:camera"] = T_lidar_cam;
  
  // T_Camera_IMU (应该等于 (T_IMU_LiDAR * T_LiDAR_Camera)^{-1})
  Eigen::Matrix4d T_cam_imu_expected = (T_imu_lidar * T_lidar_cam).inverse();
  // 添加误差
  Eigen::Matrix4d T_cam_imu = T_cam_imu_expected;
  T_cam_imu.block<3, 1>(0, 3) += Eigen::Vector3d(0.001, 0.001, 0.001);
  extrinsics["camera:imu"] = T_cam_imu;
  
  // 验证闭环
  auto [pass, error] = validator.validateLoopClosure(extrinsics, 0.01);
  
  std::cout << "闭环验证:" << std::endl;
  std::cout << "  通过: " << (pass ? "是" : "否") << std::endl;
  std::cout << "  最大误差: " << error << std::endl;
  
  // 计算具体闭环误差
  double loop_error = LoopClosureValidator::computeLoopError(
      T_imu_lidar, T_lidar_cam, T_cam_imu);
  std::cout << "  三元组误差: " << loop_error << std::endl;
}

// =============================================================================
// 主程序
// =============================================================================
int main() {
  std::cout << "=============================================" << std::endl;
  std::cout << "UniCalib 标定精度优化示例" << std::endl;
  std::cout << "基于 CalibRefine (2025), TON-VIO (2024)" << std::endl;
  std::cout << "=============================================" << std::endl;
  
  demoIterativeRefinement();
  demoTemporalOffsetEstimation();
  demoMultiResolutionOptimization();
  demoLoopClosureValidation();
  
  std::cout << "\n=============================================" << std::endl;
  std::cout << "所有示例完成!" << std::endl;
  std::cout << "=============================================" << std::endl;
  
  return 0;
}
