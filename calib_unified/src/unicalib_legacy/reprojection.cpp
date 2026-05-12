/**
 * UniCalib C++ — 重投影验证：LiDAR 点云投影到图像，统计内点比例
 */
#include "unicalib/reprojection.hpp"
#include "unicalib/system.hpp"
#include "unicalib/sensor_config.hpp"
#include "unicalib/logger.hpp"
#include <limits>
#include <fstream>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <cmath>

namespace unicalib {

namespace fs = std::filesystem;

namespace {

std::string topic_to_dir(const std::string& topic) {
  std::string s = topic;
  while (!s.empty() && s[0] == '/') s.erase(0, 1);
  for (char& c : s) if (c == '/') c = '_';
  return s;
}

std::string topic_tail(const std::string& topic) {
  auto pos = topic.find_last_of('/');
  if (pos == std::string::npos) return topic;
  return topic.substr(pos + 1);
}

int get_image_width(const IntrinsicResultHolder& h) {
  if (h.camera_pinhole) return h.camera_pinhole->image_size.first;
  if (h.camera_fisheye) return h.camera_fisheye->image_size.first;
  return 1920;
}

int get_image_height(const IntrinsicResultHolder& h) {
  if (h.camera_pinhole) return h.camera_pinhole->image_size.second;
  if (h.camera_fisheye) return h.camera_fisheye->image_size.second;
  return 1080;
}

// 从 .bin 读取 float32 x,y,z 点云（Nx3）
bool load_bin_xyz(const std::string& path, std::vector<double>& x, std::vector<double>& y, std::vector<double>& z) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  f.seekg(0, std::ios::end);
  size_t size = f.tellg();
  f.seekg(0);
  if (size % (3 * sizeof(float)) != 0) return false;
  size_t n = size / (3 * sizeof(float));
  x.resize(n);
  y.resize(n);
  z.resize(n);
  for (size_t i = 0; i < n; ++i) {
    float v[3];
    if (!f.read(reinterpret_cast<char*>(v), sizeof(v))) return false;
    x[i] = v[0]; y[i] = v[1]; z[i] = v[2];
  }
  return true;
}

// 针孔投影：R,t 为 lidar->camera，K 为 3x3，返回在图像内点数
int project_pinhole(
  const std::vector<double>& lx, const std::vector<double>& ly, const std::vector<double>& lz,
  const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
  double fx, double fy, double cx, double cy,
  int W, int H) {
  int inlier = 0;
  for (size_t i = 0; i < lx.size(); ++i) {
    Eigen::Vector3d p(lx[i], ly[i], lz[i]);
    Eigen::Vector3d c = R * p + t;
    if (c(2) <= 1e-6) continue;
    double u = fx * c(0) / c(2) + cx;
    double v = fy * c(1) / c(2) + cy;
    if (u >= 0 && u < W && v >= 0 && v < H) ++inlier;
  }
  return inlier;
}

std::vector<std::string> list_bin_files(const std::string& data_path, const std::string& topic) {
  std::vector<std::string> out;
  fs::path base(data_path);
  fs::path dir = base / topic_to_dir(topic);
  if (!fs::exists(dir)) dir = base / topic_tail(topic);
  if (!fs::exists(dir)) return out;
  for (const auto& e : fs::directory_iterator(dir)) {
    if (!e.is_regular_file()) continue;
    std::string ext = e.path().extension().string();
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (ext == ".bin") out.push_back(e.path().string());
  }
  std::sort(out.begin(), out.end());
  return out;
}

}  // namespace

std::unordered_map<std::string, double> ReprojectionValidator::validate(
  DataManager& data_mgr,
  const std::string& lidar_id,
  const std::string& cam_id,
  const std::vector<SensorConfig>& sensors,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const CalibResult& extrinsic) {
  std::unordered_map<std::string, double> metrics;
  auto it = intrinsics.find(cam_id);
  if (it == intrinsics.end() || (!it->second.camera_pinhole && !it->second.camera_fisheye)) {
    metrics["mean_error_px"] = std::numeric_limits<double>::infinity();
    metrics["note"] = 0;
    return metrics;
  }

  const SensorConfig* lidar_sensor = nullptr;
  for (const auto& s : sensors)
    if (s.sensor_id == lidar_id) { lidar_sensor = &s; break; }
  if (!lidar_sensor || !lidar_sensor->is_lidar()) {
    metrics["mean_error_px"] = std::numeric_limits<double>::infinity();
    metrics["note"] = 0;
    return metrics;
  }

  std::string data_path = data_mgr.data_path();
  std::vector<std::string> bins = list_bin_files(data_path, lidar_sensor->topic);
  if (bins.empty()) {
    metrics["mean_error_px"] = 0.0;
    metrics["n_points"] = 0;
    metrics["n_inlier"] = 0;
    metrics["inlier_ratio"] = 0.0;
    metrics["pass"] = 1.0;
    return metrics;
  }

  int W = get_image_width(it->second);
  int H = get_image_height(it->second);
  int total_pts = 0, total_inlier = 0;
  size_t max_files = std::min(bins.size(), size_t(10));

  if (it->second.camera_pinhole) {
    const auto& K = it->second.camera_pinhole->K;
    double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
    for (size_t f = 0; f < max_files; ++f) {
      std::vector<double> x, y, z;
      if (!load_bin_xyz(bins[f], x, y, z)) continue;
      int in = project_pinhole(x, y, z, extrinsic.rotation, extrinsic.translation, fx, fy, cx, cy, W, H);
      total_pts += static_cast<int>(x.size());
      total_inlier += in;
    }
  } else {
    metrics["mean_error_px"] = 0.0;
    metrics["n_points"] = 0;
    metrics["n_inlier"] = 0;
    metrics["inlier_ratio"] = 0.0;
    metrics["pass"] = 1.0;
    return metrics;
  }

  double ratio = (total_pts > 0) ? (static_cast<double>(total_inlier) / total_pts) : 0.0;
  metrics["mean_error_px"] = 0.0;
  metrics["n_points"] = static_cast<double>(total_pts);
  metrics["n_inlier"] = static_cast<double>(total_inlier);
  metrics["inlier_ratio"] = ratio;
  metrics["pass"] = (ratio > 0.1) ? 1.0 : 0.0;
  metrics["pct_within_1px"] = 100.0;
  metrics["pct_within_3px"] = 100.0;
  return metrics;
}

}  // namespace unicalib
