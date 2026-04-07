/**
 * UniCalib C++ — learn-to-calibrate 深度融合封装（IMU-LiDAR 粗外参）
 * 子进程调用 rl_solver/calib_rl.py，解析 YAML 输出。
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include "external_tools_config.hpp"
#include <optional>
#include <string>

namespace unicalib {

/** 执行 IMU-LiDAR 粗外参标定；失败或未配置时返回 nullopt */
std::optional<CalibResult> run_learn_to_calib(
  DataManager& data_mgr,
  const std::string& data_path,
  const SensorConfig& sensor_imu,
  const SensorConfig& sensor_lidar,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds = 600);

}  // namespace unicalib
