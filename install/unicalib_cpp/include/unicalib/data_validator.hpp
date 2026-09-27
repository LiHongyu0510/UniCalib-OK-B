/**
 * UniCalib C++ — 数据质量验证
 * 标定前数据检查：IMU 样本数、时间戳、静态段、范围等
 */
#pragma once

#include "allan_variance.hpp"
#include "sensor_config.hpp"
#include "exceptions.hpp"
#include <string>
#include <optional>
#include <vector>

namespace unicalib {

struct IMUValidationResult {
  bool passed = false;
  size_t sample_count = 0;
  double duration_sec = 0.0;
  double sample_rate_est = 0.0;
  bool timestamps_monotonic = true;
  bool gyro_in_range = true;
  bool accel_in_range = true;
  size_t static_segments_estimated = 0;
  std::string message;
};

struct DataValidationConfig {
  size_t min_imu_samples = 100;
  double min_imu_duration_sec = 0.0;  // 0 = 不检查时长
  double min_sample_rate_hz = 10.0;
  double max_gyro_rad_s = 10.0;
  double max_accel_mps2 = 100.0;
  size_t min_static_segments_for_six_position = 3;
  bool strict_timestamps = true;
};

class DataValidator {
 public:
  explicit DataValidator(const DataValidationConfig& config = DataValidationConfig{});

  /** 验证 IMU 数据是否满足标定要求 */
  IMUValidationResult validate_imu_data(const IMUData& data, const std::string& sensor_id = "");

  /** 仅检查不抛异常，返回结果供调用方决定 */
  IMUValidationResult check_imu_data(const IMUData& data, const std::string& sensor_id = "");

  void set_config(const DataValidationConfig& config) { config_ = config; }
  const DataValidationConfig& config() const { return config_; }

 private:
  DataValidationConfig config_;

  bool check_timestamps_monotonic(const std::vector<double>& ts);
  bool check_gyro_range(const std::vector<std::vector<double>>& gyro);
  bool check_accel_range(const std::vector<std::vector<double>>& accel);
  size_t estimate_static_segments(const IMUData& data, double window_sec = 2.0);
};

}  // namespace unicalib
