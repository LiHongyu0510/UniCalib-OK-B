/**
 * @file temporal_calibration.cpp
 * @brief 时间偏移在线估计模块实现
 */
#include "unicalib/temporal_calibration.hpp"
#include "unicalib/logger.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace unicalib {

// =============================================================================
// TemporalOffsetResult
// =============================================================================

std::string TemporalOffsetResult::toString() const {
  std::ostringstream oss;
  oss << "offset=" << offset_ms << "ms ±" << uncertainty_ms << "ms"
      << ", confidence=" << confidence
      << ", samples=" << num_samples
      << ", outliers=" << num_outliers
      << (converged ? " [CONVERGED]" : "");
  return oss.str();
}

// =============================================================================
// TemporalOffsetEstimator
// =============================================================================

TemporalOffsetEstimator::TemporalOffsetEstimator(const TemporalCalibConfig& config)
    : config_(config),
      current_offset_ms_(config.initial_offset_ms),
      uncertainty_ms_(config.max_offset_ms) {}

TemporalOffsetResult TemporalOffsetEstimator::estimateFromFeatureVelocity(
    const std::vector<FeatureVelocityData>& feature_velocities,
    const std::vector<IMUDataPoint>& imu_data) {
  
  TemporalOffsetResult result;
  result.method_used = "feature_velocity";
  result.num_samples = std::min(feature_velocities.size(), imu_data.size());
  
  if (feature_velocities.size() < 10 || imu_data.size() < 10) {
    LOG_WARNING("Insufficient data for temporal offset estimation");
    result.confidence = 0.0;
    return result;
  }
  
  // TON-VIO 方法: 特征速度与 IMU 旋转速度的相关性
  std::vector<double> offsets;
  std::vector<double> weights;
  
  for (const auto& fv : feature_velocities) {
    // 在 IMU 数据中找到最接近的时间戳
    double best_dt = std::numeric_limits<double>::max();
    Eigen::Vector3d best_gyro;
    
    for (const auto& imu : imu_data) {
      double dt = std::abs(imu.timestamp - fv.timestamp);
      if (dt < best_dt) {
        best_dt = dt;
        best_gyro = imu.gyro;
      }
    }
    
    // 特征速度与 IMU 旋转速度应成比例
    // 偏移量 = argmax correlation(feature_velocity, gyro)
    double pixel_speed = fv.pixel_velocity.norm();
    double gyro_speed = best_gyro.norm();
    
    if (gyro_speed > 1e-4) {
      // 简化: 假设比例关系
      double scale = pixel_speed / gyro_speed;
      // 时间偏移 = (expected_time - actual_time)
      // 这里使用启发式方法
      offsets.push_back(best_dt * 1000.0);  // 转换为 ms
      weights.push_back(1.0 / (1.0 + best_dt));
    }
  }
  
  if (offsets.empty()) {
    result.confidence = 0.0;
    return result;
  }
  
  // 离群点剔除
  if (config_.outlier_rejection && offsets.size() > 5) {
    auto outlier_indices = detectOutliers(offsets, config_.outlier_threshold_sigma);
    result.num_outliers = outlier_indices.size();
    
    std::vector<double> filtered_offsets;
    std::vector<double> filtered_weights;
    for (size_t i = 0; i < offsets.size(); ++i) {
      if (std::find(outlier_indices.begin(), outlier_indices.end(), i) == outlier_indices.end()) {
        filtered_offsets.push_back(offsets[i]);
        filtered_weights.push_back(weights[i]);
      }
    }
    offsets = filtered_offsets;
    weights = filtered_weights;
  }
  
  // 加权平均
  result.offset_ms = weightedMovingAverage(offsets, weights);
  
  // 计算不确定性
  double sum_sq = 0.0;
  for (size_t i = 0; i < offsets.size(); ++i) {
    double diff = offsets[i] - result.offset_ms;
    sum_sq += weights[i] * diff * diff;
  }
  result.uncertainty_ms = std::sqrt(sum_sq / std::accumulate(weights.begin(), weights.end(), 0.0));
  
  // 置信度
  result.confidence = std::exp(-result.uncertainty_ms / 10.0);
  result.confidence = std::min(1.0, std::max(0.0, result.confidence));
  
  result.num_samples = offsets.size();
  result.converged = result.uncertainty_ms < config_.convergence_threshold_ms;
  
  LOG_INFO_FMT("Temporal offset (feature velocity): %.2fms ± %.2fms, confidence=%.2f",
           result.offset_ms, result.uncertainty_ms, result.confidence);
  
  return result;
}

TemporalOffsetResult TemporalOffsetEstimator::estimateFromRotationCorrelation(
    const std::vector<std::pair<double, Eigen::Matrix3d>>& rotations_a,
    const std::vector<std::pair<double, Eigen::Matrix3d>>& rotations_b) {
  
  TemporalOffsetResult result;
  result.method_used = "rotation_correlation";
  
  if (rotations_a.size() < 10 || rotations_b.size() < 10) {
    LOG_WARNING("Insufficient rotation data for temporal offset estimation");
    result.confidence = 0.0;
    return result;
  }
  
  // 计算旋转速度
  auto computeAngularVelocity = [](const std::vector<std::pair<double, Eigen::Matrix3d>>& rots) 
      -> std::vector<std::pair<double, Eigen::Vector3d>> {
    std::vector<std::pair<double, Eigen::Vector3d>> velocities;
    for (size_t i = 1; i < rots.size(); ++i) {
      double dt = rots[i].first - rots[i-1].first;
      if (dt < 1e-6) continue;
      
      // 相对旋转
      Eigen::Matrix3d dR = rots[i-1].second.transpose() * rots[i].second;
      // 转换为轴角
      Eigen::AngleAxisd aa(dR);
      Eigen::Vector3d omega = aa.angle() / dt * aa.axis();
      
      velocities.push_back({(rots[i].first + rots[i-1].first) / 2.0, omega});
    }
    return velocities;
  };
  
  auto vel_a = computeAngularVelocity(rotations_a);
  auto vel_b = computeAngularVelocity(rotations_b);
  
  // 计算互相关
  std::vector<double> signal_a_y, signal_b_y;
  for (const auto& [t, omega] : vel_a) {
    signal_a_y.push_back(omega.y());  // 使用 y 分量
  }
  for (const auto& [t, omega] : vel_b) {
    signal_b_y.push_back(omega.y());
  }
  
  int max_lag = static_cast<int>(config_.max_offset_ms / 10.0);  // 假设 10ms 采样
  auto correlation = computeCrossCorrelation(signal_a_y, signal_b_y, max_lag);
  
  // 找最大相关的滞后
  int best_lag = 0;
  double max_corr = -1.0;
  for (int lag = -max_lag; lag <= max_lag; ++lag) {
    int idx = lag + max_lag;
    if (idx >= 0 && idx < static_cast<int>(correlation.size()) && correlation[idx] > max_corr) {
      max_corr = correlation[idx];
      best_lag = lag;
    }
  }
  
  // 滞后转换为时间偏移
  // 假设采样率约为 100Hz (10ms 间隔)
  double sample_interval_ms = 10.0;  // 简化
  result.offset_ms = best_lag * sample_interval_ms;
  result.confidence = (max_corr + 1.0) / 2.0;  // 归一化到 [0, 1]
  result.uncertainty_ms = sample_interval_ms;  // 简化
  result.num_samples = vel_a.size();
  result.converged = max_corr > 0.8;
  
  LOG_INFO_FMT("Temporal offset (rotation correlation): %.2fms, correlation=%.3f",
           result.offset_ms, max_corr);
  
  return result;
}

TemporalOffsetResult TemporalOffsetEstimator::estimateFromCrossCorrelation(
    const std::vector<std::pair<double, double>>& signal_a,
    const std::vector<std::pair<double, double>>& signal_b) {
  
  TemporalOffsetResult result;
  result.method_used = "cross_correlation";
  
  if (signal_a.size() < 10 || signal_b.size() < 10) {
    result.confidence = 0.0;
    return result;
  }
  
  // 提取信号值
  std::vector<double> values_a, values_b;
  for (const auto& [t, v] : signal_a) values_a.push_back(v);
  for (const auto& [t, v] : signal_b) values_b.push_back(v);
  
  // 计算互相关
  int max_lag = static_cast<int>(values_a.size() / 4);
  auto correlation = computeCrossCorrelation(values_a, values_b, max_lag);
  
  // 找最大相关
  int best_lag = 0;
  double max_corr = -1.0;
  for (int lag = -max_lag; lag <= max_lag; ++lag) {
    int idx = lag + max_lag;
    if (idx >= 0 && idx < static_cast<int>(correlation.size()) && correlation[idx] > max_corr) {
      max_corr = correlation[idx];
      best_lag = lag;
    }
  }
  
  // 采样间隔估计
  double dt_a = (signal_a.back().first - signal_a.front().first) / signal_a.size();
  double dt_b = (signal_b.back().first - signal_b.front().first) / signal_b.size();
  double sample_interval_ms = (dt_a + dt_b) / 2.0 * 1000.0;
  
  result.offset_ms = best_lag * sample_interval_ms;
  result.confidence = (max_corr + 1.0) / 2.0;
  result.uncertainty_ms = sample_interval_ms;
  result.num_samples = signal_a.size();
  result.converged = max_corr > 0.7;
  
  return result;
}

void TemporalOffsetEstimator::updateOnline(
    const IMUDataPoint& imu, 
    const FeatureVelocityData& feature) {
  
  imu_buffer_.push_back(imu);
  feature_buffer_.push_back(feature);
  
  // 保持滑动窗口大小
  while (imu_buffer_.size() > static_cast<size_t>(config_.sliding_window_size)) {
    imu_buffer_.pop_front();
  }
  while (feature_buffer_.size() > static_cast<size_t>(config_.sliding_window_size)) {
    feature_buffer_.pop_front();
  }
  
  // 当数据足够时更新估计
  if (imu_buffer_.size() >= 20 && feature_buffer_.size() >= 20) {
    std::vector<IMUDataPoint> imu_vec(imu_buffer_.begin(), imu_buffer_.end());
    std::vector<FeatureVelocityData> feature_vec(feature_buffer_.begin(), feature_buffer_.end());
    
    auto result = estimateFromFeatureVelocity(feature_vec, imu_vec);
    current_offset_ms_ = result.offset_ms;
    uncertainty_ms_ = result.uncertainty_ms;
  }
}

TemporalOffsetResult TemporalOffsetEstimator::getCurrentEstimate() const {
  TemporalOffsetResult result;
  result.offset_ms = current_offset_ms_;
  result.uncertainty_ms = uncertainty_ms_;
  result.confidence = std::exp(-uncertainty_ms_ / 10.0);
  result.num_samples = imu_buffer_.size();
  result.method_used = "online_update";
  result.converged = uncertainty_ms_ < config_.convergence_threshold_ms;
  return result;
}

void TemporalOffsetEstimator::reset() {
  imu_buffer_.clear();
  feature_buffer_.clear();
  current_offset_ms_ = config_.initial_offset_ms;
  uncertainty_ms_ = config_.max_offset_ms;
}

std::vector<double> TemporalOffsetEstimator::computeCrossCorrelation(
    const std::vector<double>& signal_a,
    const std::vector<double>& signal_b,
    int max_lag) {
  
  int n = std::min(signal_a.size(), signal_b.size());
  std::vector<double> correlation(2 * max_lag + 1, 0.0);
  
  // 标准化信号
  double mean_a = 0.0, mean_b = 0.0;
  for (int i = 0; i < n; ++i) {
    mean_a += signal_a[i];
    mean_b += signal_b[i];
  }
  mean_a /= n;
  mean_b /= n;
  
  double std_a = 0.0, std_b = 0.0;
  for (int i = 0; i < n; ++i) {
    std_a += (signal_a[i] - mean_a) * (signal_a[i] - mean_a);
    std_b += (signal_b[i] - mean_b) * (signal_b[i] - mean_b);
  }
  std_a = std::sqrt(std_a / n);
  std_b = std::sqrt(std_b / n);
  
  // 计算互相关
  for (int lag = -max_lag; lag <= max_lag; ++lag) {
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < n; ++i) {
      int j = i + lag;
      if (j >= 0 && j < n) {
        sum += (signal_a[i] - mean_a) * (signal_b[j] - mean_b);
        ++count;
      }
    }
    if (count > 0 && std_a > 1e-8 && std_b > 1e-8) {
      correlation[lag + max_lag] = sum / (count * std_a * std_b);
    }
  }
  
  return correlation;
}

std::vector<size_t> TemporalOffsetEstimator::detectOutliers(
    const std::vector<double>& residuals,
    double threshold_sigma) {
  
  std::vector<size_t> outliers;
  if (residuals.empty()) return outliers;
  
  // 计算均值和标准差
  double mean = 0.0;
  for (double r : residuals) mean += r;
  mean /= residuals.size();
  
  double std = 0.0;
  for (double r : residuals) {
    std += (r - mean) * (r - mean);
  }
  std = std::sqrt(std / residuals.size());
  
  // 检测离群点
  for (size_t i = 0; i < residuals.size(); ++i) {
    if (std::abs(residuals[i] - mean) > threshold_sigma * std) {
      outliers.push_back(i);
    }
  }
  
  return outliers;
}

double TemporalOffsetEstimator::weightedMovingAverage(
    const std::vector<double>& values,
    const std::vector<double>& weights) {
  
  if (values.empty()) return 0.0;
  
  double sum = 0.0;
  double weight_sum = 0.0;
  for (size_t i = 0; i < values.size(); ++i) {
    sum += weights[i] * values[i];
    weight_sum += weights[i];
  }
  
  return weight_sum > 0 ? sum / weight_sum : 0.0;
}

// =============================================================================
// BSplineTemporalOptimizer
// =============================================================================

// 构造函数已在头文件中内联定义

std::pair<double, double> BSplineTemporalOptimizer::optimize(
    const std::vector<IMUDataPoint>& imu_data,
    const std::vector<double>& lidar_timestamps,
    double initial_offset) {
  
  // 简化实现: 使用网格搜索
  double best_offset = initial_offset;
  double best_cost = std::numeric_limits<double>::max();
  
  double offset_min = initial_offset - config_.time_offset_padding;
  double offset_max = initial_offset + config_.time_offset_padding;
  double offset_step = 0.001;  // 1ms 步长
  
  for (double offset = offset_min; offset <= offset_max; offset += offset_step) {
    // 计算代价 (简化: 使用时间戳对齐误差)
    double cost = 0.0;
    int count = 0;
    
    for (double lidar_ts : lidar_timestamps) {
      double corrected_ts = lidar_ts + offset;
      // 找最近的 IMU 时间戳
      double min_dt = std::numeric_limits<double>::max();
      for (const auto& imu : imu_data) {
        double dt = std::abs(imu.timestamp - corrected_ts);
        min_dt = std::min(min_dt, dt);
      }
      cost += min_dt * min_dt;
      ++count;
    }
    
    if (count > 0) {
      cost /= count;
    }
    
    if (cost < best_cost) {
      best_cost = cost;
      best_offset = offset;
    }
  }
  
  LOG_INFO_FMT("B-spline temporal optimization: offset=%.2fms, cost=%.6f",
           best_offset * 1000.0, best_cost);
  
  return {best_offset, best_cost};
}

// =============================================================================
// TemporalOffsetValidator
// =============================================================================

// 构造函数已在头文件中内联定义

std::pair<bool, std::string> TemporalOffsetValidator::validate(
    const std::vector<TemporalOffsetResult>& estimates) {
  
  if (estimates.size() < static_cast<size_t>(config_.min_samples)) {
    return {false, "Insufficient samples: " + std::to_string(estimates.size())};
  }
  
  // 检查一致性
  if (!checkTemporalConsistency(estimates)) {
    return {false, "Temporal consistency check failed"};
  }
  
  // 检查平均偏移是否合理
  double mean_offset = 0.0;
  for (const auto& est : estimates) {
    mean_offset += est.offset_ms;
  }
  mean_offset /= estimates.size();
  
  if (std::abs(mean_offset) > 100.0) {
    return {false, "Mean offset too large: " + std::to_string(mean_offset) + "ms"};
  }
  
  return {true, "Validation passed"};
}

bool TemporalOffsetValidator::checkTemporalConsistency(
    const std::vector<TemporalOffsetResult>& estimates) {
  
  if (estimates.size() < 2) return true;
  
  // 计算连续估计之间的变化
  for (size_t i = 1; i < estimates.size(); ++i) {
    double diff = std::abs(estimates[i].offset_ms - estimates[i-1].offset_ms);
    if (diff > config_.consistency_threshold_ms) {
      LOG_WARNING_FMT("Temporal inconsistency detected: %.2fms between estimates %d and %d",
                  diff, static_cast<int>(i-1), static_cast<int>(i));
      return false;
    }
  }
  
  return true;
}

}  // namespace unicalib
