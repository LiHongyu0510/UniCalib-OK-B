/**
 * UniCalib C++ — 标定结果数据结构
 * 对应 Python: unicalib/core/calib_result.py
 */
#pragma once

#include <string>
#include <limits>
#include <vector>
#include <unordered_map>
#include <optional>
#include <Eigen/Dense>

namespace unicalib {

/** 相机内参 (针孔模型) */
struct CameraIntrinsic {
  Eigen::Matrix3d K{Eigen::Matrix3d::Identity()};
  std::vector<double> dist_coeffs;
  std::pair<int, int> image_size{0, 0};  // (width, height)
  double reprojection_error = std::numeric_limits<double>::infinity();
  std::string method;
};

/** 鱼眼相机内参 */
struct FisheyeIntrinsic {
  std::string model_type;  // eucm / double_sphere / kannala_brandt / equidistant
  std::unordered_map<std::string, double> params;
  std::pair<int, int> image_size{0, 0};
  double reprojection_error = std::numeric_limits<double>::infinity();
  std::string method;
};

/** IMU 内参 */
struct IMUIntrinsic {
  double gyro_noise = 0.0;
  double gyro_bias_instability = 0.0;
  double accel_noise = 0.0;
  double accel_bias_instability = 0.0;
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d Ma{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d Sa{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d Mg{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d Sg{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d Tg{Eigen::Matrix3d::Zero()};
  std::string method;
};

/** 传感器对外参标定结果 */
struct CalibResult {
  std::pair<std::string, std::string> pair;
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  double time_offset = 0.0;
  double reprojection_error = std::numeric_limits<double>::infinity();
  double confidence = 0.0;
  std::string method_used;

  /** 返回 ZYX 欧拉角 [度] */
  Eigen::Vector3d rotation_euler_deg() const;
};

/** 单对验证指标 */
struct ValidationMetrics {
  std::string pair_key;
  double mean_error_px = std::numeric_limits<double>::infinity();
  double median_error_px = std::numeric_limits<double>::infinity();
  double std_error_px = std::numeric_limits<double>::infinity();
  double max_error_px = std::numeric_limits<double>::infinity();
  double pct_within_1px = 0.0;
  double pct_within_3px = 0.0;
  bool pass_threshold = false;
  std::unordered_map<std::string, double> extra;
};

/** 全局验证报告 */
struct ValidationReport {
  std::unordered_map<std::string, ValidationMetrics> metrics;
  bool overall_pass = false;
  std::string summary;

  void add_metric(const std::string& key, const std::unordered_map<std::string, double>& data);
};

/** 内参结果持有器（支持多种传感器类型） */
struct IntrinsicResultHolder {
  std::string sensor_id;
  std::string sensor_type;  // "camera_pinhole", "camera_fisheye", "lidar", "imu"

  std::optional<CameraIntrinsic> camera_pinhole;
  std::optional<FisheyeIntrinsic> camera_fisheye;
  std::optional<IMUIntrinsic> imu;
  
  // LiDAR 特有字段（可选）
  std::optional<std::string> lidar_type;
  std::optional<int> num_channels;
  std::optional<double> max_range;
};

}  // namespace unicalib
