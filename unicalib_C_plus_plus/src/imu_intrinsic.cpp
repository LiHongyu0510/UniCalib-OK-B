/**
 * UniCalib C++ — IMU 内参标定实现（含数据验证与质量检查）
 */
#include "unicalib/imu_intrinsic.hpp"
#include "unicalib/data_validator.hpp"
#include "unicalib/calibration_quality.hpp"
#include "unicalib/logger.hpp"
#include "unicalib/exceptions.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace unicalib {

namespace {

const double g_nominal = 9.81;

double norm3(const Eigen::Vector3d& v) {
  return std::sqrt(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
}

double stddev(const std::vector<double>& v) {
  if (v.size() < 2) return 0.0;
  double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  double sq = 0.0;
  for (double x : v) sq += (x - mean) * (x - mean);
  return std::sqrt(sq / (v.size() - 1));
}

}  // namespace

std::optional<std::pair<std::array<double, 3>, Eigen::Vector3d>>
IMUIntrinsicCalibrator::six_position_calibration(const IMUData& imu_data) {
  if (imu_data.accel.empty()) return std::nullopt;
  const double rate = imu_data.sample_rate;
  const size_t window = std::max(size_t(10), static_cast<size_t>(rate * 2));
  std::vector<Eigen::Vector3d> static_means;

  for (size_t i = 0; i + window <= imu_data.accel.size(); ) {
    Eigen::Vector3d sum(0, 0, 0);
    for (size_t j = i; j < i + window && j < imu_data.accel.size(); ++j) {
      const auto& a = imu_data.accel[j];
      if (a.size() >= 3) sum += Eigen::Vector3d(a[0], a[1], a[2]);
    }
    Eigen::Vector3d mean = sum / window;
    double std_x = 0, std_y = 0, std_z = 0;
    for (size_t j = i; j < i + window && j < imu_data.accel.size(); ++j) {
      const auto& a = imu_data.accel[j];
      if (a.size() >= 3) {
        std_x += (a[0] - mean(0)) * (a[0] - mean(0));
        std_y += (a[1] - mean(1)) * (a[1] - mean(1));
        std_z += (a[2] - mean(2)) * (a[2] - mean(2));
      }
    }
    double seg_std = std::sqrt((std_x + std_y + std_z) / (3 * window));
    if (seg_std < 0.05) {
      double n = norm3(mean);
      if (std::abs(n - g_nominal) < 0.5) {
        static_means.push_back(mean);
        i += window;
        continue;
      }
    }
    i += std::max(size_t(1), window / 4);
  }

  if (static_means.size() < 3) return std::nullopt;

  Eigen::Vector3d mean_all(0, 0, 0);
  for (const auto& m : static_means) mean_all += m;
  mean_all /= static_means.size();
  Eigen::Vector3d scale(1, 1, 1);
  for (int i = 0; i < 3; ++i) {
    double max_abs = 0;
    for (const auto& m : static_means) max_abs = std::max(max_abs, std::abs(m(i)));
    if (g_nominal > 1e-9) scale(i) = max_abs / g_nominal;
  }
  Eigen::Vector3d ba = mean_all - g_nominal * Eigen::Vector3d(
    mean_all(0) >= 0 ? 1 : -1, mean_all(1) >= 0 ? 1 : -1, mean_all(2) >= 0 ? 1 : -1);
  std::array<double, 3> Sa_diag = {scale(0), scale(1), scale(2)};
  return std::make_pair(Sa_diag, ba);
}

IMUIntrinsic IMUIntrinsicCalibrator::build_from_allan(
  const AllanVarianceResult& allan,
  const std::optional<std::pair<std::array<double, 3>, Eigen::Vector3d>>& accel_params) {
  IMUIntrinsic out;
  out.gyro_noise = allan.gyro_noise_avg;
  out.gyro_bias_instability = allan.gyro_bias_avg;
  out.accel_noise = allan.accel_noise_avg;
  out.accel_bias_instability = allan.accel_bias_avg;
  out.Ma = Eigen::Matrix3d::Identity();
  out.Mg = Eigen::Matrix3d::Identity();
  out.Sg = Eigen::Matrix3d::Identity();
  out.Tg = Eigen::Matrix3d::Zero();
  if (accel_params) {
    out.Sa = Eigen::Matrix3d::Identity();
    out.Sa(0, 0) = accel_params->first[0];
    out.Sa(1, 1) = accel_params->first[1];
    out.Sa(2, 2) = accel_params->first[2];
    out.accel_bias = accel_params->second;
  } else {
    out.Sa = Eigen::Matrix3d::Identity();
  }
  out.method = "AllanVariance_cpp";
  return out;
}

IMUIntrinsic IMUIntrinsicCalibrator::default_intrinsic() {
  IMUIntrinsic out;
  out.gyro_noise = 1.6968e-4;
  out.gyro_bias_instability = 1e-5;
  out.accel_noise = 2.0e-3;
  out.accel_bias_instability = 3e-4;
  out.method = "default_mems";
  return out;
}

IMUIntrinsic IMUIntrinsicCalibrator::calibrate(DataManager& data_mgr, const SensorConfig& sensor,
                                               DataValidator* validator,
                                               CalibrationQualityChecker* quality_checker) {
  auto imu_opt = data_mgr.load_imu_data(sensor.topic);
  if (!imu_opt) {
    LOG_WARNING("No IMU data for " + sensor.sensor_id + ", using default intrinsic.");
    return default_intrinsic();
  }
  if (imu_opt->timestamps.size() < 100) {
    LOG_WARNING("Insufficient IMU samples (" + std::to_string(imu_opt->timestamps.size()) +
                ") for " + sensor.sensor_id + ", using default intrinsic.");
    return default_intrinsic();
  }

  if (validator) {
    IMUValidationResult vr = validator->check_imu_data(*imu_opt, sensor.sensor_id);
    if (!vr.passed) {
      LOG_WARNING("IMU data validation failed for " + sensor.sensor_id + ": " + vr.message);
      throw ValidationException(ErrorCode::DATA_QUALITY_TOO_LOW, vr.message);
    }
  }

  AllanVarianceAnalyzer allan;
  AllanVarianceResult ar = allan.analyze(*imu_opt);
  auto accel_params = six_position_calibration(*imu_opt);
  IMUIntrinsic result = build_from_allan(ar, accel_params);

  if (quality_checker) {
    IMUQualityResult qr = quality_checker->evaluate_imu_intrinsic(result, sensor.sensor_id);
    if (!qr.passed) {
      LOG_WARNING("IMU calibration quality check failed for " + sensor.sensor_id + ": " + qr.message);
      throw CalibrationException(ErrorCode::CALIBRATION_INVALID_RESULT, "calibrate", qr.message);
    }
  }
  return result;
}

}  // namespace unicalib
