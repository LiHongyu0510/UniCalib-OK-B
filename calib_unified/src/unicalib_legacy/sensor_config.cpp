/**
 * UniCalib C++ — sensor_config 实现
 */
#include "unicalib/sensor_config.hpp"
#include <algorithm>
#include <set>

namespace unicalib {

bool SensorConfig::is_camera() const {
  return sensor_type == SensorType::CAMERA_PINHOLE ||
         sensor_type == SensorType::CAMERA_FISHEYE;
}

bool SensorConfig::is_fisheye() const {
  return sensor_type == SensorType::CAMERA_FISHEYE;
}

bool SensorConfig::is_lidar() const {
  return sensor_type == SensorType::LIDAR;
}

bool SensorConfig::is_imu() const {
  return sensor_type == SensorType::IMU;
}

namespace {

std::optional<CalibPair> select_method(const SensorConfig& sa, const SensorConfig& sb) {
  using ST = SensorType;
  const ST a = sa.sensor_type;
  const ST b = sb.sensor_type;

  auto both = [&](ST x, ST y) {
    return (a == x && b == y) || (a == y && b == x);
  };
  auto both_camera_pinhole = [&]() {
    return a == ST::CAMERA_PINHOLE && b == ST::CAMERA_PINHOLE;
  };
  auto both_camera_fisheye = [&]() {
    return a == ST::CAMERA_FISHEYE && b == ST::CAMERA_FISHEYE;
  };
  auto both_camera = [&]() {
    return (a == ST::CAMERA_PINHOLE && b == ST::CAMERA_FISHEYE) ||
           (a == ST::CAMERA_FISHEYE && b == ST::CAMERA_PINHOLE);
  };

  if (both(ST::IMU, ST::LIDAR)) {
    return CalibPair{sa.sensor_id, sb.sensor_id,
                     "l2calib_rl_init", "ikalibr_bspline", 1};
  }
  if (both(ST::LIDAR, ST::CAMERA_PINHOLE)) {
    return CalibPair{sa.sensor_id, sb.sensor_id,
                     "mias_lcec_coarse", "mias_lcec_fine", 2};
  }
  if (both(ST::LIDAR, ST::CAMERA_FISHEYE)) {
    return CalibPair{sa.sensor_id, sb.sensor_id,
                     "mias_lcec_coarse", "ikalibr_bspline", 2};
  }
  if (both_camera_pinhole()) {
    return CalibPair{sa.sensor_id, sb.sensor_id,
                     "feature_matching", "click_calib_ba", 3};
  }
  if (both_camera_fisheye()) {
    return CalibPair{sa.sensor_id, sb.sensor_id,
                     "feature_matching", "click_calib_ba", 3};
  }
  if (both_camera()) {
    return CalibPair{sa.sensor_id, sb.sensor_id,
                     "feature_matching", "click_calib_ba", 3};
  }
  return std::nullopt;
}

}  // namespace

std::vector<CalibPair> auto_infer_calib_pairs(const std::vector<SensorConfig>& sensors) {
  std::vector<CalibPair> pairs;
  const size_t n = sensors.size();
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      auto p = select_method(sensors[i], sensors[j]);
      if (p) pairs.push_back(*p);
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const CalibPair& a, const CalibPair& b) { return a.priority < b.priority; });
  return pairs;
}

}  // namespace unicalib
