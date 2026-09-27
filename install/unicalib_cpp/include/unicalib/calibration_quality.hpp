/**
 * UniCalib C++ — 标定质量评估
 * 对 IMU 内参等结果做合理性检查与精度指标
 */
#pragma once

#include "calib_result.hpp"
#include "exceptions.hpp"
#include <string>
#include <optional>

namespace unicalib {

struct IMUQualityResult {
  bool passed = false;
  bool gyro_noise_reasonable = true;
  bool gyro_bias_reasonable = true;
  bool accel_noise_reasonable = true;
  bool accel_bias_reasonable = true;
  std::string message;
};

struct CalibrationQualityConfig {
  double gyro_noise_min = 1e-6;
  double gyro_noise_max = 1e-1;
  double gyro_bias_instability_min = 1e-8;
  double gyro_bias_instability_max = 1e-2;
  double accel_noise_min = 1e-5;
  double accel_noise_max = 1.0;
  double accel_bias_instability_min = 1e-6;
  double accel_bias_instability_max = 1e-1;
};

class CalibrationQualityChecker {
 public:
  explicit CalibrationQualityChecker(const CalibrationQualityConfig& config = CalibrationQualityConfig{});

  /** 检查 IMU 内参是否在合理范围 */
  IMUQualityResult check_imu_intrinsic(const IMUIntrinsic& intrinsic,
                                       const std::string& sensor_id = "");

  /** 不抛异常，仅返回结果 */
  IMUQualityResult evaluate_imu_intrinsic(const IMUIntrinsic& intrinsic,
                                           const std::string& sensor_id = "");

  void set_config(const CalibrationQualityConfig& config) { config_ = config; }

 private:
  CalibrationQualityConfig config_;
};

}  // namespace unicalib
