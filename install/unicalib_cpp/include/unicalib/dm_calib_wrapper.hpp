/**
 * UniCalib C++ — DM-Calib 深度融合封装（针孔相机内参无靶初估计）
 * 通过子进程调用 Python 脚本，解析 JSON 输出。
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include "process_runner.hpp"
#include "external_tools_config.hpp"
#include <optional>
#include <string>

namespace unicalib {

/** 调用 DM-Calib 推理，返回针孔内参；失败或未配置时返回 nullopt */
std::optional<CameraIntrinsic> run_dm_calib(
  DataManager& data_mgr,
  const SensorConfig& sensor,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds = 300);

}  // namespace unicalib
