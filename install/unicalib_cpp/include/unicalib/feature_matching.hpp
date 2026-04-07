/**
 * UniCalib C++ — Camera-Camera 粗外参（特征匹配 + 本质矩阵）
 * 需 OpenCV；无 OpenCV 时返回 nullopt。
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include "system.hpp"
#include <optional>

namespace unicalib {

/** 基于特征匹配估计 Camera-Camera 外参 (R, t 无尺度) */
std::optional<CalibResult> run_feature_matching_coarse(
  DataManager& data_mgr,
  const SensorConfig& sensor_a,
  const SensorConfig& sensor_b,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  int max_frames = 30,
  int min_matches = 20);

}  // namespace unicalib
