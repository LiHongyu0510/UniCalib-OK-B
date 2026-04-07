/**
 * UniCalib C++ — iKalibr 联合精化：构建配置、调用 ros2、解析结果
 */
#include "unicalib/ikalibr_wrapper.hpp"
#include "unicalib/process_runner.hpp"
#include "unicalib/logger.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>

#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace fs = std::filesystem;
namespace unicalib {

namespace {

std::string sensor_type_to_ikalibr(SensorType t) {
  switch (t) {
    case SensorType::IMU: return "SENSOR_IMU";
    case SensorType::LIDAR: return "SENSOR_LIDAR";
    case SensorType::CAMERA_PINHOLE: return "SENSOR_CAMERA";
    case SensorType::CAMERA_FISHEYE: return "SENSOR_CAMERA_FISHEYE";
    default: return "SENSOR_UNKNOWN";
  }
}

std::string lidar_type_to_string(LidarType t) {
  switch (t) {
    case LidarType::SPINNING: return "spinning";
    case LidarType::SOLID_STATE: return "solid_state";
    default: return "spinning";
  }
}

}  // namespace

std::optional<std::unordered_map<std::string, CalibResult>> run_ikalibr_joint(
  DataManager& data_mgr,
  const std::string& data_path,
  const std::vector<SensorConfig>& sensors,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const std::unordered_map<std::string, CalibResult>& coarse_extrinsics,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds) {

  (void)data_mgr;
  if (tools_config.ikalibr.empty()) {
    LOG_WARNING("iKalibr not configured for joint optimization.");
    LOG_WARNING("Set 'third_party.ikalibr' in config.yaml or set UNICALIB_IKALIBR environment variable.");
    LOG_WARNING("Example config:");
    LOG_WARNING("  third_party:");
    LOG_WARNING("    ikalibr: \"/path/to/iKalibr\"");
    LOG_WARNING("Example environment variable:");
    LOG_WARNING("  export UNICALIB_IKALIBR=/path/to/iKalibr");
    LOG_WARNING("Note: iKalibr requires ROS2 environment. Ensure ROS_DOMAIN_ID is set.");
    return std::nullopt;
  }

#if !(defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML)
  LOG_WARNING("iKalibr requires yaml-cpp");
  return std::nullopt;
#else
  std::string tmpdir = (fs::temp_directory_path() / "unicalib_ikalibr").string();
  try {
    fs::create_directories(tmpdir);
  } catch (...) {
    LOG_ERROR("iKalibr: failed to create " + tmpdir);
    return std::nullopt;
  }

  std::string config_path = tmpdir + "/ikalibr_config.yaml";
  std::string result_dir = tmpdir + "/result";

  // --- 构建 Calibration 配置 ---
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "Calibration" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "Type" << YAML::Value << "MULTI_SENSOR";
  out << YAML::Key << "OutputPath" << YAML::Value << result_dir;
  out << YAML::Key << "DataPath" << YAML::Value << data_path;
  out << YAML::Key << "BSpline" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "SplineOrder" << YAML::Value << 4;
  out << YAML::Key << "KnotTimeDistance" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "SO3" << YAML::Value << 0.2;
  out << YAML::Key << "Pos" << YAML::Value << 0.2;
  out << YAML::EndMap << YAML::EndMap;
  out << YAML::Key << "Optimization" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "MaxIterations" << YAML::Value << 200;
  out << YAML::Key << "EnableTimeOffset" << YAML::Value << true;
  out << YAML::Key << "TimeOffsetPadding" << YAML::Value << 0.1;
  out << YAML::Key << "RobustKernel" << YAML::Value << true;
  out << YAML::Key << "RobustKernelParam" << YAML::Value << 1.0;
  out << YAML::EndMap;

  out << YAML::Key << "Sensors" << YAML::Value << YAML::BeginMap;
  for (const auto& s : sensors) {
    out << YAML::Key << s.sensor_id << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "Type" << YAML::Value << sensor_type_to_ikalibr(s.sensor_type);
    out << YAML::Key << "Topic" << YAML::Value << s.topic;
    out << YAML::Key << "FrameId" << YAML::Value << s.frame_id;

    auto it_i = intrinsics.find(s.sensor_id);
    if (s.is_camera() && it_i != intrinsics.end()) {
      if (it_i->second.camera_pinhole) {
        const auto& K = it_i->second.camera_pinhole->K;
        const auto& d = it_i->second.camera_pinhole->dist_coeffs;
        out << YAML::Key << "Intrinsic" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "K" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) out << K(i, j);
        out << YAML::EndSeq;
        out << YAML::Key << "dist_coeffs" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (double c : d) out << c;
        out << YAML::EndSeq << YAML::EndMap;
      } else if (it_i->second.camera_fisheye) {
        out << YAML::Key << "Intrinsic" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "model_type" << YAML::Value << it_i->second.camera_fisheye->model_type;
        for (const auto& [k, v] : it_i->second.camera_fisheye->params)
          out << YAML::Key << k << YAML::Value << v;
        out << YAML::EndMap;
      }
    } else if (s.is_lidar() && s.lidar_type) {
      out << YAML::Key << "LidarType" << YAML::Value << lidar_type_to_string(*s.lidar_type);
    }

    if (s.rate) out << YAML::Key << "Rate" << YAML::Value << *s.rate;
    out << YAML::EndMap;
  }
  out << YAML::EndMap;

  out << YAML::Key << "InitialExtrinsics" << YAML::Value << YAML::BeginMap;
  for (const auto& [key, r] : coarse_extrinsics) {
    std::string ext_key = r.pair.first + "_to_" + r.pair.second;
    out << YAML::Key << ext_key << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "Rotation" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i)
      out << YAML::Flow << YAML::BeginSeq << r.rotation(i,0) << r.rotation(i,1) << r.rotation(i,2) << YAML::EndSeq;
    out << YAML::EndSeq;
    out << YAML::Key << "Translation" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << r.translation(0) << r.translation(1) << r.translation(2) << YAML::EndSeq;
    out << YAML::Key << "TimeOffset" << YAML::Value << r.time_offset;
    out << YAML::EndMap;
  }
  out << YAML::EndMap << YAML::EndMap << YAML::EndMap;  // InitialExtrinsics, Calibration, root

  try {
    std::ofstream f(config_path);
    if (!f) { LOG_ERROR("iKalibr: cannot write " + config_path); return std::nullopt; }
    f << out.c_str();
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("iKalibr config write: ") + e.what());
    return std::nullopt;
  }

  // --- 调用 ros2 run ikalibr ikalibr_prog ---
  std::string abs_config = fs::absolute(config_path).string();
  std::vector<std::string> argv = {
    "ros2", "run", "ikalibr", "ikalibr_prog",
    "--ros-args", "-p", "config_path:=" + abs_config
  };
  ProcessResult res = run_process(argv, {}, timeout_seconds, "");
  if (res.exit_code != 0 || res.timed_out) {
    LOG_WARNING("iKalibr process exit_code=" + std::to_string(res.exit_code)
                + " timed_out=" + (res.timed_out ? "1" : "0"));
    if (!res.stderr_output.empty()) LOG_WARNING("iKalibr stderr: " + res.stderr_output.substr(0, 500));
    return std::nullopt;
  }

  // --- 解析 result_dir/calibration_result.yaml ---
  fs::create_directories(result_dir);
  std::string result_yaml = result_dir + "/calibration_result.yaml";
  if (!fs::exists(result_yaml)) {
    for (const auto& e : fs::directory_iterator(result_dir)) {
      if (e.path().extension() == ".yaml") {
        result_yaml = e.path().string();
        break;
      }
    }
  }
  if (!fs::exists(result_yaml)) {
    LOG_WARNING("iKalibr result YAML not found in " + result_dir);
    return std::nullopt;
  }

  YAML::Node data;
  try {
    data = YAML::LoadFile(result_yaml);
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("iKalibr result parse: ") + e.what());
    return std::nullopt;
  }

  auto extr = data["Extrinsics"] ? data["Extrinsics"] : data["extrinsics"];
  std::unordered_map<std::string, CalibResult> results;
  if (!extr || !extr.IsMap()) return std::nullopt;

  for (auto it = extr.begin(); it != extr.end(); ++it) {
    std::string key_str = it->first.as<std::string>();
    const YAML::Node& ext_data = it->second;
    // key_str 形如 "cam_to_lidar" 或 "sensor_a_to_sensor_b"
    std::string delim = "_to_";
    size_t pos = key_str.find(delim);
    if (pos == std::string::npos) continue;
    std::string sa = key_str.substr(0, pos);
    std::string sb = key_str.substr(pos + delim.size());
    std::string our_key = sa + ":" + sb;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(0, 0, 0);
    double td = 0.0, reproj = std::numeric_limits<double>::infinity();
    if (ext_data["Rotation"]) {
      const auto& Rnode = ext_data["Rotation"];
      for (int i = 0; i < 3 && i < static_cast<int>(Rnode.size()); ++i)
        for (int j = 0; j < 3 && j < static_cast<int>(Rnode[i].size()); ++j)
          R(i, j) = Rnode[i][j].as<double>();
    }
    if (ext_data["Translation"]) {
      const auto& tnode = ext_data["Translation"];
      for (size_t i = 0; i < 3 && i < tnode.size(); ++i) t(i) = tnode[i].as<double>();
    }
    if (ext_data["TimeOffset"]) td = ext_data["TimeOffset"].as<double>();
    if (ext_data["ReprojectionError"]) reproj = ext_data["ReprojectionError"].as<double>();

    CalibResult cr;
    cr.pair = {sa, sb};
    cr.rotation = R;
    cr.translation = t;
    cr.time_offset = td;
    cr.reprojection_error = reproj;
    cr.confidence = 0.9;
    cr.method_used = "iKalibr_bspline";
    results[our_key] = std::move(cr);
    LOG_INFO("iKalibr result [" + sa + "->" + sb + "] td=" + std::to_string(td*1000) + "ms reproj=" + std::to_string(reproj));
  }

  return results;
#endif
}

}  // namespace unicalib
