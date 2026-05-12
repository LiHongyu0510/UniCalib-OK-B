/**
 * UniCalib C++ — iKalibr 联合精化封装
 * 子进程调用 ros2 run ikalibr ikalibr_prog，解析 calibration_result.yaml。
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include "external_tools_config.hpp"
#include "system.hpp"
#include <optional>
#include <string>
#include <unordered_map>

namespace unicalib {

/**
 * 运行 iKalibr 多传感器联合 B-spline 精化。
 * 输入为当前粗外参 (extrinsic_results)，输出为精化后的外参（按 pair key 更新）。
 * 若 ros2/ikalibr 不可用或失败，返回 nullopt；否则返回更新后的外参映射（仅包含 iKalibr 输出的对）。
 */
std::optional<std::unordered_map<std::string, CalibResult>> run_ikalibr_joint(
  DataManager& data_mgr,
  const std::string& data_path,
  const std::vector<SensorConfig>& sensors,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const std::unordered_map<std::string, CalibResult>& coarse_extrinsics,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds = 1800);

}  // namespace unicalib
