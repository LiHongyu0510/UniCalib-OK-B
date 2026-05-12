/**
 * UniCalib C++ — 数据验证实现
 */
#include "unicalib/data_validator.hpp"
#include "unicalib/logger.hpp"
#include <cmath>
#include <algorithm>

namespace unicalib {

DataValidator::DataValidator(const DataValidationConfig& config) : config_(config) {}

bool DataValidator::check_timestamps_monotonic(const std::vector<double>& ts) {
  for (size_t i = 1; i < ts.size(); ++i)
    if (ts[i] <= ts[i - 1]) return false;
  return true;
}

bool DataValidator::check_gyro_range(const std::vector<std::vector<double>>& gyro) {
  for (const auto& g : gyro) {
    if (g.size() < 3) return false;
    for (int i = 0; i < 3; ++i)
      if (std::abs(g[i]) > config_.max_gyro_rad_s) return false;
  }
  return true;
}

bool DataValidator::check_accel_range(const std::vector<std::vector<double>>& accel) {
  for (const auto& a : accel) {
    if (a.size() < 3) return false;
    for (int i = 0; i < 3; ++i)
      if (std::abs(a[i]) > config_.max_accel_mps2) return false;
  }
  return true;
}

size_t DataValidator::estimate_static_segments(const IMUData& data, double window_sec) {
  if (data.accel.empty() || data.sample_rate < 1e-6) return 0;
  const double g = 9.81;
  const size_t window = std::max(size_t(10), static_cast<size_t>(data.sample_rate * window_sec));
  size_t count = 0;
  for (size_t i = 0; i + window <= data.accel.size(); ) {
    double sum_norm = 0;
    for (size_t j = i; j < i + window && j < data.accel.size(); ++j) {
      const auto& a = data.accel[j];
      if (a.size() >= 3) {
        double n = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
        sum_norm += std::abs(n - g);
      }
    }
    double mean_dev = sum_norm / window;
    if (mean_dev < 0.1) {
      ++count;
      i += window;
    } else {
      i += std::max(size_t(1), window / 4);
    }
  }
  return count;
}

IMUValidationResult DataValidator::check_imu_data(const IMUData& data,
                                                   const std::string& sensor_id) {
  IMUValidationResult r;
  r.sample_count = data.timestamps.size();
  if (data.timestamps.size() < 2) {
    r.message = "Insufficient IMU samples";
    return r;
  }
  r.duration_sec = data.timestamps.back() - data.timestamps.front();
  r.sample_rate_est = data.sample_rate > 0 ? data.sample_rate : (r.sample_count / std::max(1e-9, r.duration_sec));
  r.timestamps_monotonic = check_timestamps_monotonic(data.timestamps);
  r.gyro_in_range = check_gyro_range(data.gyro);
  r.accel_in_range = check_accel_range(data.accel);
  r.static_segments_estimated = estimate_static_segments(data);

  r.passed = (r.sample_count >= config_.min_imu_samples) &&
             (r.duration_sec >= config_.min_imu_duration_sec || config_.min_imu_duration_sec <= 0) &&
             (r.sample_rate_est >= config_.min_sample_rate_hz) &&
             (!config_.strict_timestamps || r.timestamps_monotonic) &&
             r.gyro_in_range && r.accel_in_range;

  if (!r.passed) {
    if (r.sample_count < config_.min_imu_samples)
      r.message = "Sample count " + std::to_string(r.sample_count) + " < " + std::to_string(config_.min_imu_samples);
    else if (r.duration_sec < config_.min_imu_duration_sec)
      r.message = "Duration " + std::to_string(r.duration_sec) + "s < " + std::to_string(config_.min_imu_duration_sec) + "s";
    else if (r.sample_rate_est < config_.min_sample_rate_hz)
      r.message = "Sample rate " + std::to_string(r.sample_rate_est) + " < " + std::to_string(config_.min_sample_rate_hz);
    else if (!r.timestamps_monotonic)
      r.message = "Timestamps not monotonic";
    else if (!r.gyro_in_range || !r.accel_in_range)
      r.message = "Data out of range (gyro/accel)";
    else
      r.message = "Validation failed";
  } else {
    r.message = "OK";
  }
  return r;
}

IMUValidationResult DataValidator::validate_imu_data(const IMUData& data,
                                                      const std::string& sensor_id) {
  IMUValidationResult r = check_imu_data(data, sensor_id);
  if (!r.passed) {
    std::string msg = "IMU validation failed: " + r.message;
    if (!sensor_id.empty()) msg += " (sensor: " + sensor_id + ")";
    throw ValidationException(ErrorCode::DATA_QUALITY_TOO_LOW, msg);
  }
  return r;
}

}  // namespace unicalib
