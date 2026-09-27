/**
 * UniCalib C++ — 重投影误差验证 (占位/简化)
 * 对应 Python: unicalib/validation/reprojection.py
 * 无 OpenCV 时仅返回占位指标；后续可接 OpenCV projectPoints + 边缘距离。
 */
#pragma once

#include <string>
#include <unordered_map>
#include "calib_result.hpp"
#include "data_manager.hpp"
#include "sensor_config.hpp"
#include "system.hpp"

namespace unicalib {

/** LiDAR-Camera 重投影验证：返回 mean/median/std/max_error_px, pct_within_1px/3px 等 */
class ReprojectionValidator {
 public:
  /** 计算验证指标；无图像/点云或内参缺失时返回占位值 */
  std::unordered_map<std::string, double> validate(
    DataManager& data_mgr,
    const std::string& lidar_id,
    const std::string& cam_id,
    const std::vector<SensorConfig>& sensors,
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const CalibResult& extrinsic);
};

}  // namespace unicalib
