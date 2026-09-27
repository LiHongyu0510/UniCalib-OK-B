/**
 * UniCalib C++ — 鱼眼相机内参标定（系统必备能力）
 * 对应 Python: unicalib/intrinsic/camera_fisheye.py
 * 当启用 OpenCV 时使用 cv::fisheye::calibrate (equidistant 模型)。
 */
#pragma once

#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "data_manager.hpp"
#include <optional>

namespace unicalib {

class CameraFisheyeCalibrator {
 public:
  /** 使用棋盘格图像执行鱼眼内参标定；无 OpenCV 或图像不足时返回 nullopt */
  std::optional<FisheyeIntrinsic> calibrate(DataManager& data_mgr,
                                            const SensorConfig& sensor);

 private:
  static constexpr int kCheckerRows = 8;
  static constexpr int kCheckerCols = 11;
  static constexpr double kSquareSize = 0.03;
  static constexpr int kMaxFrames = 100;
  static constexpr int kFrameSkip = 3;
  static constexpr int kMinViews = 5;
};

}  // namespace unicalib
