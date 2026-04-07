/**
 * UniCalib C++ — calib_result 实现 (含欧拉角)
 */
#include "unicalib/calib_result.hpp"
#include "unicalib/transforms.hpp"
#include <cmath>

namespace unicalib {

Eigen::Vector3d CalibResult::rotation_euler_deg() const {
  return rotation_matrix_to_euler(rotation, true);
}

void ValidationReport::add_metric(
  const std::string& key,
  const std::unordered_map<std::string, double>& data) {
  ValidationMetrics m;
  m.pair_key = key;
  auto get = [&](const std::string& k, double def) {
    auto it = data.find(k);
    return it != data.end() ? it->second : def;
  };
  m.mean_error_px = get("mean_error_px", std::numeric_limits<double>::infinity());
  m.median_error_px = get("median_error_px", std::numeric_limits<double>::infinity());
  m.std_error_px = get("std_error_px", std::numeric_limits<double>::infinity());
  m.max_error_px = get("max_error_px", std::numeric_limits<double>::infinity());
  m.pct_within_1px = get("pct_within_1px", 0.0);
  m.pct_within_3px = get("pct_within_3px", 0.0);
  m.pass_threshold = (get("pass", 0.0) >= 0.5);
  for (const auto& [k, v] : data) {
    if (k != "mean_error_px" && k != "median_error_px" &&
        k != "std_error_px" && k != "max_error_px" &&
        k != "pct_within_1px" && k != "pct_within_3px" && k != "pass")
      m.extra[k] = v;
  }
  m.extra["pass"] = get("pass", 0.0);
  metrics[key] = std::move(m);
}

}  // namespace unicalib
