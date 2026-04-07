/**
 * UniCalib C++ — IMU 内参标定
 * 对应 Python: unicalib/intrinsic/imu_intrinsic.py
 * 实现: Allan 方差 + 六面法简化 + build_from_allan，不调用 iKalibr/Transformer。
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include "allan_variance.hpp"
#include <array>
#include <optional>

namespace unicalib {

class DataValidator;
class CalibrationQualityChecker;

class IMUIntrinsicCalibrator {
 public:
  /** 使用 IMU 数据执行内参标定；可选传入验证器与质量检查器 */
  IMUIntrinsic calibrate(DataManager& data_mgr, const SensorConfig& sensor,
                         DataValidator* validator = nullptr,
                         CalibrationQualityChecker* quality_checker = nullptr);

 private:
  /** 六面法简化：从静态区段估计加速度计比例与零偏 */
  std::optional<std::pair<std::array<double, 3>, Eigen::Vector3d>> six_position_calibration(
    const IMUData& imu_data);

  /** 仅用 Allan + 可选六面结果构建 IMUIntrinsic */
  IMUIntrinsic build_from_allan(
    const AllanVarianceResult& allan,
    const std::optional<std::pair<std::array<double, 3>, Eigen::Vector3d>>& accel_params);

  IMUIntrinsic default_intrinsic();
};

}  // namespace unicalib
