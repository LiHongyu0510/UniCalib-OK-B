/**
 * UniCalib C++ — MIAS-LCEC 深度融合封装（LiDAR-Camera 粗/精外参）
 * 子进程调用 mias_lcec 可执行文件，解析 result.yaml。
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include "external_tools_config.hpp"
#include "system.hpp"
#include <optional>
#include <string>

namespace unicalib {

/** 粗标定：无靶标 LiDAR-Camera 粗外参 */
std::optional<CalibResult> run_mias_lcec_coarse(
  DataManager& data_mgr,
  const std::string& data_path,
  const SensorConfig& sensor_lidar,
  const SensorConfig& sensor_cam,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds = 600);

/** 精标定：在粗结果基础上时空精化 */
std::optional<CalibResult> run_mias_lcec_fine(
  DataManager& data_mgr,
  const std::string& data_path,
  const SensorConfig& sensor_lidar,
  const SensorConfig& sensor_cam,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const CalibResult& initial,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds = 600);

}  // namespace unicalib
