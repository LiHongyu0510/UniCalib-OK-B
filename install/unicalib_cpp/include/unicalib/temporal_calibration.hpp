/**
 * @file temporal_calibration.hpp
 * @brief 时间偏移在线估计模块
 * @reference TON-VIO (2024) - Online Time Offset Modeling Networks
 */
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <functional>
#include <optional>

namespace unicalib {

/**
 * @brief 时间偏移估计配置
 */
struct TemporalCalibConfig {
  bool enabled = true;
  double max_offset_ms = 100.0;           // 最大时间偏移 (ms)
  double initial_offset_ms = 0.0;         // 初始偏移估计
  double convergence_threshold_ms = 1.0;  // 收敛阈值
  int sliding_window_size = 100;          // 滑动窗口大小
  int max_iterations = 50;                // 最大迭代次数
  bool outlier_rejection = true;          // 离群点剔除
  double outlier_threshold_sigma = 3.0;   // 离群点阈值 (sigma)
  
  static TemporalCalibConfig Default() {
    return TemporalCalibConfig{};
  }
};

/**
 * @brief 时间偏移估计结果
 */
struct TemporalOffsetResult {
  double offset_ms;                       // 估计的时间偏移
  double uncertainty_ms;                  // 不确定性
  double confidence;                      // 置信度 [0, 1]
  int num_samples;                        // 使用的样本数
  int num_outliers;                       // 剔除的离群点数
  bool converged;                         // 是否收敛
  std::string method_used;
  
  std::string toString() const;
};

/**
 * @brief IMU 数据点
 */
struct IMUDataPoint {
  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
};

/**
 * @brief 特征速度数据 (用于 TON-VIO 方法)
 */
struct FeatureVelocityData {
  double timestamp;
  Eigen::Vector2d pixel_velocity;  // 像素速度
  int feature_id;
};

/**
 * @brief 旋转相关数据 (用于滑动窗口方法)
 */
struct RotationCorrelation {
  double timestamp_a;
  double timestamp_b;
  double correlation_value;
};

/**
 * @brief 时间偏移在线估计器
 * 
 * 支持多种估计方法:
 *   1. 特征速度法 (TON-VIO 风格)
 *   2. 旋转相关法 (函数相似度)
 *   3. 互相关法 (信号处理)
 */
class TemporalOffsetEstimator {
 public:
  explicit TemporalOffsetEstimator(const TemporalCalibConfig& config = TemporalCalibConfig::Default());
  
  /**
   * @brief 使用特征速度估计时间偏移 (TON-VIO 方法)
   * @param feature_velocities 特征速度序列
   * @param imu_data IMU 数据序列
   * @return 估计结果
   */
  TemporalOffsetResult estimateFromFeatureVelocity(
      const std::vector<FeatureVelocityData>& feature_velocities,
      const std::vector<IMUDataPoint>& imu_data);
  
  /**
   * @brief 使用旋转相关估计时间偏移
   * @param rotations_a 传感器 A 的旋转序列
   * @param rotations_b 传感器 B 的旋转序列
   * @return 估计结果
   */
  TemporalOffsetResult estimateFromRotationCorrelation(
      const std::vector<std::pair<double, Eigen::Matrix3d>>& rotations_a,
      const std::vector<std::pair<double, Eigen::Matrix3d>>& rotations_b);
  
  /**
   * @brief 使用互相关估计时间偏移
   * @param signal_a 信号 A (时间戳, 值)
   * @param signal_b 信号 B (时间戳, 值)
   * @return 估计结果
   */
  TemporalOffsetResult estimateFromCrossCorrelation(
      const std::vector<std::pair<double, double>>& signal_a,
      const std::vector<std::pair<double, double>>& signal_b);
  
  /**
   * @brief 在线更新时间偏移估计
   * @param new_data 新数据点
   */
  void updateOnline(const IMUDataPoint& imu, const FeatureVelocityData& feature);
  
  /**
   * @brief 获取当前估计
   */
  TemporalOffsetResult getCurrentEstimate() const;
  
  /**
   * @brief 重置估计器
   */
  void reset();

 private:
  TemporalCalibConfig config_;
  std::deque<IMUDataPoint> imu_buffer_;
  std::deque<FeatureVelocityData> feature_buffer_;
  double current_offset_ms_;
  double uncertainty_ms_;
  
  /**
   * @brief 计算互相关
   */
  std::vector<double> computeCrossCorrelation(
      const std::vector<double>& signal_a,
      const std::vector<double>& signal_b,
      int max_lag);
  
  /**
   * @brief 离群点剔除
   */
  std::vector<size_t> detectOutliers(
      const std::vector<double>& residuals,
      double threshold_sigma);
  
  /**
   * @brief 加权移动平均滤波
   */
  double weightedMovingAverage(
      const std::vector<double>& values,
      const std::vector<double>& weights);
};

/**
 * @brief 连续时间 B-spline 时间偏移优化
 * 
 * 在 B-spline 轨迹优化中联合估计时间偏移
 */
class BSplineTemporalOptimizer {
 public:
  struct Config {
    int spline_order = 4;
    double knot_distance = 0.02;   // 秒
    double time_offset_padding = 0.15;  // ±150ms 搜索范围
    int max_iterations = 100;
    double convergence_threshold = 1e-6;
    
    static Config Default() { return Config(); }
  };
  
  BSplineTemporalOptimizer() : config_(Config::Default()) {}
  explicit BSplineTemporalOptimizer(const Config& config) : config_(config) {}
  
  /**
   * @brief 联合优化轨迹和时间偏移
   * @param imu_data IMU 数据
   * @param lidar_timestamps LiDAR 时间戳
   * @param initial_offset 初始时间偏移
   * @return (优化后的偏移, 最终代价)
   */
  std::pair<double, double> optimize(
      const std::vector<IMUDataPoint>& imu_data,
      const std::vector<double>& lidar_timestamps,
      double initial_offset);

 private:
  Config config_;
};

/**
 * @brief 时间偏移验证器
 */
class TemporalOffsetValidator {
 public:
  struct ValidationConfig {
    double consistency_threshold_ms = 5.0;  // 一致性阈值
    int min_samples = 10;                    // 最小样本数
    double max_temporal_drift_ms = 1.0;      // 最大时间漂移 (per second)
    
    static ValidationConfig Default() { return ValidationConfig(); }
  };
  
  TemporalOffsetValidator() : config_(ValidationConfig::Default()) {}
  explicit TemporalOffsetValidator(const ValidationConfig& config) : config_(config) {}
  
  /**
   * @brief 验证时间偏移估计
   * @param estimates 时间偏移估计序列
   * @return (是否通过, 详细信息)
   */
  std::pair<bool, std::string> validate(
      const std::vector<TemporalOffsetResult>& estimates);
  
  /**
   * @brief 检查时间一致性
   */
  bool checkTemporalConsistency(
      const std::vector<TemporalOffsetResult>& estimates);

 private:
  ValidationConfig config_;
};

}  // namespace unicalib
