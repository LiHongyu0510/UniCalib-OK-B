/**
 * UniCalib C++ — 标定质量评估实现
 */
#include "unicalib/calibration_quality.hpp"
#include "unicalib/logger.hpp"
#include <cmath>

namespace unicalib {

CalibrationQualityChecker::CalibrationQualityChecker(const CalibrationQualityConfig& config)
    : config_(config) {}

IMUQualityResult CalibrationQualityChecker::evaluate_imu_intrinsic(const IMUIntrinsic& intrinsic,
                                                                   const std::string& sensor_id) {
  IMUQualityResult r;
  auto in = [](double v, double lo, double hi) {
    return !std::isnan(v) && !std::isinf(v) && v >= lo && v <= hi;
  };

  r.gyro_noise_reasonable = in(intrinsic.gyro_noise, config_.gyro_noise_min, config_.gyro_noise_max);
  r.gyro_bias_reasonable = in(intrinsic.gyro_bias_instability, config_.gyro_bias_instability_min,
                              config_.gyro_bias_instability_max);
  r.accel_noise_reasonable = in(intrinsic.accel_noise, config_.accel_noise_min, config_.accel_noise_max);
  r.accel_bias_reasonable = in(intrinsic.accel_bias_instability, config_.accel_bias_instability_min,
                               config_.accel_bias_instability_max);

  r.passed = r.gyro_noise_reasonable && r.gyro_bias_reasonable &&
             r.accel_noise_reasonable && r.accel_bias_reasonable;

  if (!r.passed) {
    if (!r.gyro_noise_reasonable) r.message += "gyro_noise out of range; ";
    if (!r.gyro_bias_reasonable) r.message += "gyro_bias_instability out of range; ";
    if (!r.accel_noise_reasonable) r.message += "accel_noise out of range; ";
    if (!r.accel_bias_reasonable) r.message += "accel_bias_instability out of range; ";
  } else {
    r.message = "OK";
  }
  if (!sensor_id.empty()) r.message = "[" + sensor_id + "] " + r.message;
  return r;
}

IMUQualityResult CalibrationQualityChecker::check_imu_intrinsic(const IMUIntrinsic& intrinsic,
                                                                 const std::string& sensor_id) {
  IMUQualityResult r = evaluate_imu_intrinsic(intrinsic, sensor_id);
  if (!r.passed)
    throw CalibrationException(ErrorCode::CALIBRATION_INVALID_RESULT, r.message);
  return r;
}

}  // namespace unicalib
