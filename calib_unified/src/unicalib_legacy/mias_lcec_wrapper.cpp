/**
 * UniCalib C++ — MIAS-LCEC 封装实现：同步帧、配置、子进程、解析
 */
#include "unicalib/mias_lcec_wrapper.hpp"
#include "unicalib/process_runner.hpp"
#include "unicalib/logger.hpp"
#include <filesystem>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <sstream>
#include <iomanip>

#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace unicalib {

namespace fs = std::filesystem;

namespace {

static constexpr double kSyncThresholdSec = 0.05;
static constexpr int kMinSyncedFrames = 5;

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

using TsPath = std::vector<std::pair<double, std::string>>;

TsPath collect_paths(DataManager& data_mgr, const std::string& topic,
                    std::optional<int> max_frames, int skip) {
  TsPath out;
  data_mgr.iter_image_paths(topic, [&](double ts, const std::string& path) {
    out.emplace_back(ts, path);
  }, max_frames, skip);
  return out;
}

TsPath collect_pc_paths(DataManager& data_mgr, const std::string& topic,
                       std::optional<int> max_frames, int skip) {
  TsPath out;
  // DataManager 没有 iter_pointcloud_paths，用 data_path + topic 子目录列出 .bin/.pcd
  (void)data_mgr;
  (void)topic;
  (void)max_frames;
  (void)skip;
  return out;
}

// 从 data_path 扫描 topic 对应子目录下的点云文件
TsPath scan_pc_paths(const std::string& data_path, const std::string& topic) {
  TsPath out;
  fs::path base(data_path);
  fs::path dir = base / topic_to_dir(topic);
  if (!fs::exists(dir)) dir = base / topic_tail(topic);
  if (!fs::exists(dir)) return out;
  for (const auto& e : fs::directory_iterator(dir)) {
    if (!e.is_regular_file()) continue;
    std::string ext = e.path().extension().string();
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (ext != ".bin" && ext != ".pcd" && ext != ".ply") continue;
    double ts = 0;
    try { ts = std::stod(e.path().stem().string()); } catch (...) { continue; }
    out.emplace_back(ts, e.path().string());
  }
  std::sort(out.begin(), out.end());
  return out;
}

struct SyncedPair { double ts; std::string img_path; std::string pc_path; };

std::vector<SyncedPair> match_synced(const TsPath& img_paths, const TsPath& pc_paths) {
  std::vector<SyncedPair> out;
  for (const auto& [ts_i, path_i] : img_paths) {
    auto it = std::lower_bound(pc_paths.begin(), pc_paths.end(), ts_i - kSyncThresholdSec,
        [](const std::pair<double, std::string>& a, double t) { return a.first < t; });
    if (it == pc_paths.end()) continue;
    if (std::abs(it->first - ts_i) > kSyncThresholdSec) continue;
    out.push_back({ts_i, path_i, it->second});
  }
  return out;
}

#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
bool write_mias_config(
  const std::string& path,
  const std::string& lidar_dir,
  const std::string& cam_dir,
  const std::string& output_dir,
  const std::string& mode,
  const SensorConfig& sensor_cam,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const CalibResult* initial) {
  auto it = intrinsics.find(sensor_cam.sensor_id);
  if (it == intrinsics.end()) return false;

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "lidar" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "data_dir" << YAML::Value << lidar_dir;
  out << YAML::Key << "type" << YAML::Value << "binary_float32_xyz";
  out << YAML::EndMap;
  out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "data_dir" << YAML::Value << cam_dir;
  if (it->second.camera_pinhole) {
    const auto& K = it->second.camera_pinhole->K;
    out << YAML::Key << "intrinsic" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "model" << YAML::Value << "pinhole";
    out << YAML::Key << "fx" << YAML::Value << K(0, 0);
    out << YAML::Key << "fy" << YAML::Value << K(1, 1);
    out << YAML::Key << "cx" << YAML::Value << K(0, 2);
    out << YAML::Key << "cy" << YAML::Value << K(1, 2);
    out << YAML::Key << "dist_coeffs" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (double d : it->second.camera_pinhole->dist_coeffs) out << d;
    out << YAML::EndSeq << YAML::EndMap;
  } else if (it->second.camera_fisheye) {
    out << YAML::Key << "intrinsic" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "model" << YAML::Value << it->second.camera_fisheye->model_type;
    for (const auto& [k, v] : it->second.camera_fisheye->params) out << YAML::Key << k << YAML::Value << v;
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
  out << YAML::Key << "output_dir" << YAML::Value << output_dir;
  out << YAML::Key << "calibration" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "mode" << YAML::Value << mode;
  out << YAML::Key << "optimize_time_offset" << YAML::Value << true;
  out << YAML::Key << "use_motion_compensation" << YAML::Value << false;
  out << YAML::Key << "max_iterations" << YAML::Value << 100;
  out << YAML::EndMap;
  if (initial) {
    out << YAML::Key << "initial_extrinsic" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "rotation" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) out << initial->rotation(i, j);
    out << YAML::EndSeq;
    out << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << initial->translation(0) << initial->translation(1) << initial->translation(2) << YAML::EndSeq;
    out << YAML::Key << "time_offset" << YAML::Value << initial->time_offset;
    out << YAML::EndMap;
  }
  out << YAML::EndMap;

  std::ofstream f(path);
  return f && (f << out.c_str());
}

std::optional<CalibResult> parse_mias_result(
  const std::string& path, const std::string& lidar_id, const std::string& cam_id) {
  try {
    YAML::Node n = YAML::LoadFile(path);
    CalibResult r;
    r.pair = {lidar_id, cam_id};
    r.method_used = "MIAS-LCEC";
    r.confidence = 0.85;
    if (n["translation"] && n["translation"].IsSequence() && n["translation"].size() >= 3) {
      r.translation(0) = n["translation"][0].as<double>();
      r.translation(1) = n["translation"][1].as<double>();
      r.translation(2) = n["translation"][2].as<double>();
    }
    if (n["time_offset"]) r.time_offset = n["time_offset"].as<double>();
    if (n["reprojection_error"]) r.reprojection_error = n["reprojection_error"].as<double>();
    if (n["rotation"] && n["rotation"].IsSequence() && n["rotation"].size() == 9) {
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          r.rotation(i, j) = n["rotation"][i * 3 + j].as<double>();
    }
    return r;
  } catch (...) {
    return std::nullopt;
  }
}
#endif

}  // namespace

std::optional<CalibResult> run_mias_lcec_coarse(
  DataManager& data_mgr,
  const std::string& data_path,
  const SensorConfig& sensor_lidar,
  const SensorConfig& sensor_cam,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds) {
  if (tools_config.mias_lcec.empty()) {
    LOG_WARNING("MIAS-LCEC not configured for coarse extrinsic calibration.");
    LOG_WARNING("Set 'third_party.mias_lcec' in config.yaml or set UNICALIB_MIAS_LCEC environment variable.");
    LOG_WARNING("Example config:");
    LOG_WARNING("  third_party:");
    LOG_WARNING("    mias_lcec: \"/path/to/MIAS-LCEC\"");
    LOG_WARNING("Example environment variable:");
    LOG_WARNING("  export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC");
    LOG_WARNING("Default location: /path/to/calibration/MIAS-LCEC");
    LOG_WARNING("Note: Run './auto_config_env.sh' to auto-detect and set environment variables.");
    return std::nullopt;
  }

  TsPath img_paths = collect_paths(data_mgr, sensor_cam.topic, 80, 1);
  TsPath pc_paths = scan_pc_paths(data_path, sensor_lidar.topic);
  std::vector<SyncedPair> synced = match_synced(img_paths, pc_paths);
  if (synced.size() < static_cast<size_t>(kMinSyncedFrames)) {
    LOG_WARNING("MIAS-LCEC: too few synced frames (have " + std::to_string(synced.size()) + ")");
    return std::nullopt;
  }

  std::string tmpdir = fs::temp_directory_path().string() + "/unicalib_mias_" +
    sensor_lidar.sensor_id + "_" + sensor_cam.sensor_id;
  fs::create_directories(tmpdir);
  std::string lidar_dir = tmpdir + "/lidar", cam_dir = tmpdir + "/camera";
  fs::create_directories(lidar_dir);
  fs::create_directories(cam_dir);
  for (const auto& p : synced) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6f", p.ts);
    std::string name(buf);
    std::error_code ec;
    fs::create_symlink(p.img_path, cam_dir + "/" + name + ".jpg", ec);
    fs::create_symlink(p.pc_path, lidar_dir + "/" + name + ".bin", ec);
  }

  std::string cfg_path = tmpdir + "/mias_config.yaml";
#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
  if (!write_mias_config(cfg_path, lidar_dir, cam_dir, tmpdir, "coarse",
                         sensor_cam, intrinsics, nullptr))
    return std::nullopt;
#else
  (void)intrinsics;
  LOG_WARNING("MIAS-LCEC requires yaml-cpp");
  return std::nullopt;
#endif

  std::string bin = tools_config.mias_lcec + "/bin/mias_lcec";
  if (!fs::exists(bin)) bin = "mias_lcec";
  ProcessResult res = run_process({bin, "--config", cfg_path}, {}, timeout_seconds, "");
  if (res.exit_code != 0 || res.timed_out) {
    LOG_WARNING("MIAS-LCEC coarse failed: " + res.stderr_output.substr(0, 300));
    return std::nullopt;
  }
  std::string result_path = tmpdir + "/result.yaml";
  if (!fs::exists(result_path)) return std::nullopt;
#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
  auto out = parse_mias_result(result_path, sensor_lidar.sensor_id, sensor_cam.sensor_id);
  if (out) LOG_INFO("MIAS-LCEC coarse OK: " + sensor_lidar.sensor_id + " -> " + sensor_cam.sensor_id);
  return out;
#else
  return std::nullopt;
#endif
}

std::optional<CalibResult> run_mias_lcec_fine(
  DataManager& data_mgr,
  const std::string& data_path,
  const SensorConfig& sensor_lidar,
  const SensorConfig& sensor_cam,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const CalibResult& initial,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds) {
  if (tools_config.mias_lcec.empty()) {
    LOG_WARNING("MIAS-LCEC not configured for fine extrinsic calibration.");
    LOG_WARNING("Set 'third_party.mias_lcec' in config.yaml or set UNICALIB_MIAS_LCEC environment variable.");
    LOG_WARNING("Example config:");
    LOG_WARNING("  third_party:");
    LOG_WARNING("    mias_lcec: \"/path/to/MIAS-LCEC\"");
    LOG_WARNING("Example environment variable:");
    LOG_WARNING("  export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC");
    LOG_WARNING("Default location: /path/to/calibration/MIAS-LCEC");
    LOG_WARNING("Note: Run './auto_config_env.sh' to auto-detect and set environment variables.");
    return std::nullopt;
  }

  TsPath img_paths = collect_paths(data_mgr, sensor_cam.topic, 80, 1);
  TsPath pc_paths = scan_pc_paths(data_path, sensor_lidar.topic);
  std::vector<SyncedPair> synced = match_synced(img_paths, pc_paths);
  if (synced.size() < static_cast<size_t>(kMinSyncedFrames)) return std::nullopt;

  std::string tmpdir = fs::temp_directory_path().string() + "/unicalib_mias_fine_" +
    sensor_lidar.sensor_id + "_" + sensor_cam.sensor_id;
  fs::create_directories(tmpdir);
  std::string lidar_dir = tmpdir + "/lidar", cam_dir = tmpdir + "/camera";
  fs::create_directories(lidar_dir);
  fs::create_directories(cam_dir);
  for (const auto& p : synced) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6f", p.ts);
    std::string name(buf);
    std::error_code ec;
    fs::create_symlink(p.img_path, cam_dir + "/" + name + ".jpg", ec);
    fs::create_symlink(p.pc_path, lidar_dir + "/" + name + ".bin", ec);
  }

  std::string cfg_path = tmpdir + "/mias_config.yaml";
#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
  if (!write_mias_config(cfg_path, lidar_dir, cam_dir, tmpdir, "fine",
                         sensor_cam, intrinsics, &initial))
    return std::nullopt;
#else
  (void)intrinsics;
  (void)initial;
  return std::nullopt;
#endif

  std::string bin = tools_config.mias_lcec + "/bin/mias_lcec";
  if (!fs::exists(bin)) bin = "mias_lcec";
  ProcessResult res = run_process({bin, "--config", cfg_path}, {}, timeout_seconds, "");
  if (res.exit_code != 0 || res.timed_out) return std::nullopt;
  std::string result_path = tmpdir + "/result.yaml";
  if (!fs::exists(result_path)) return std::nullopt;
#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
  return parse_mias_result(result_path, sensor_lidar.sensor_id, sensor_cam.sensor_id);
#else
  return std::nullopt;
#endif
}

}  // namespace unicalib
