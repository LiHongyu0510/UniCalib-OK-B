/**
 * UniCalib C++ — click_calib Camera-Camera BA 精化封装
 * 自动关键点（SIFT+匹配）→ calib_init.yaml + keypoints.yaml → optimize.py → 解析 result
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include "external_tools_config.hpp"
#include "system.hpp"
#include <optional>

namespace unicalib {

/**
 * 运行 click_calib BA 精化（Camera-Camera）。
 * 从 data_mgr 取双目图像，自动提取匹配关键点，调用 optimize.py，解析外参。
 * 若 click_calib 不可用或关键点不足，返回 nullopt。
 */
std::optional<CalibResult> run_click_calib_ba(
  DataManager& data_mgr,
  const SensorConfig& sensor_a,
  const SensorConfig& sensor_b,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const std::optional<CalibResult>& initial,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds = 300);

}  // namespace unicalib
