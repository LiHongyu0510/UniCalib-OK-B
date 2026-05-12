/**
 * UniCalib C++ — 传感器配置数据结构
 * 对应 Python: unicalib/core/sensor_config.py
 */
#pragma once

#include <string>
#include <vector>
#include <optional>
#include <utility>

namespace unicalib {

enum class SensorType {
  CAMERA_PINHOLE,
  CAMERA_FISHEYE,
  LIDAR,
  IMU
};

enum class LidarType {
  SPINNING,      // Velodyne, Ouster
  SOLID_STATE    // Livox
};

enum class CalibStage {
  INTRINSIC,
  COARSE_EXTRINSIC,
  FINE_EXTRINSIC,
  VALIDATION
};

inline const char* to_string(SensorType t) {
  switch (t) {
    case SensorType::CAMERA_PINHOLE: return "camera_pinhole";
    case SensorType::CAMERA_FISHEYE: return "camera_fisheye";
    case SensorType::LIDAR: return "lidar";
    case SensorType::IMU: return "imu";
    default: return "unknown";
  }
}

inline SensorType sensor_type_from_string(const std::string& s) {
  if (s == "camera_pinhole") return SensorType::CAMERA_PINHOLE;
  if (s == "camera_fisheye") return SensorType::CAMERA_FISHEYE;
  if (s == "lidar") return SensorType::LIDAR;
  if (s == "imu") return SensorType::IMU;
  return SensorType::CAMERA_PINHOLE;
}

/** 单个传感器配置 */
struct SensorConfig {
  std::string sensor_id;
  SensorType sensor_type{SensorType::CAMERA_PINHOLE};
  std::string topic;
  std::string frame_id;

  std::optional<double> rate;                    // Hz
  std::optional<std::pair<int, int>> resolution;  // (width, height)
  std::optional<LidarType> lidar_type;
  // intrinsic_prior 暂用 string/后续可扩展为 map

  bool is_camera() const;
  bool is_fisheye() const;
  bool is_lidar() const;
  bool is_imu() const;
};

/** 标定传感器对 */
struct CalibPair {
  std::string sensor_a;
  std::string sensor_b;
  std::string method_coarse;
  std::string method_fine;
  int priority{0};

  std::pair<std::string, std::string> key() const { return {sensor_a, sensor_b}; }
};

/** 根据传感器列表自动推断标定对（按优先级排序） */
std::vector<CalibPair> auto_infer_calib_pairs(
  const std::vector<SensorConfig>& sensors);

}  // namespace unicalib
