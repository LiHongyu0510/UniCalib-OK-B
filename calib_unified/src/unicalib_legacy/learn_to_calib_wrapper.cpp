/**
 * UniCalib C++ — learn-to-calibrate 封装实现
 */
#include "unicalib/learn_to_calib_wrapper.hpp"
#include "unicalib/process_runner.hpp"
#include "unicalib/logger.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>

#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
#include <yaml-cpp/yaml.h>
#endif

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

bool write_imu_csv(const std::string& path, const IMUData& imu) {
  std::ofstream f(path);
  if (!f) return false;
  f << "timestamp,gx,gy,gz,ax,ay,az\n";
  for (size_t i = 0; i < imu.timestamps.size(); ++i) {
    f << imu.timestamps[i] << ","
      << imu.gyro[i][0] << "," << imu.gyro[i][1] << "," << imu.gyro[i][2] << ","
      << imu.accel[i][0] << "," << imu.accel[i][1] << "," << imu.accel[i][2] << "\n";
  }
  return true;
}

}  // namespace

std::optional<CalibResult> run_learn_to_calib(
  DataManager& data_mgr,
  const std::string& data_path,
  const SensorConfig& sensor_imu,
  const SensorConfig& sensor_lidar,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds) {
  if (tools_config.learn_to_calibrate.empty()) {
    LOG_WARNING("learn-to-calibrate not configured. Set 'third_party.learn_to_calibrate' in config.yaml or set UNICALIB_LEARN_TO_CALIB environment variable.");
    LOG_WARNING("Example config:");
    LOG_WARNING("  third_party:");
    LOG_WARNING("    learn_to_calibrate: \"/path/to/learn-to-calibrate\"");
    LOG_WARNING("Example environment variable:");
    LOG_WARNING("  export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate");
    LOG_WARNING("Default location: /path/to/calibration/learn-to-calibrate");
    return std::nullopt;
  }

  std::string calib_rl = tools_config.learn_to_calibrate + "/rl_solver/calib_rl.py";
  if (!fs::exists(calib_rl)) {
    LOG_WARNING("learn-to-calibrate calib_rl.py not found: " + calib_rl);
    return std::nullopt;
  }

  auto imu_opt = data_mgr.load_imu_data(sensor_imu.topic);
  if (!imu_opt || imu_opt->timestamps.size() < 50) {
    LOG_WARNING("learn-to-calibrate: insufficient IMU data for " + sensor_imu.sensor_id);
    return std::nullopt;
  }

  fs::path lidar_dir = fs::path(data_path) / topic_to_dir(sensor_lidar.topic);
  if (!fs::exists(lidar_dir)) lidar_dir = fs::path(data_path) / topic_tail(sensor_lidar.topic);
  if (!fs::exists(lidar_dir)) {
    LOG_WARNING("learn-to-calibrate: lidar dir not found: " + lidar_dir.string());
    return std::nullopt;
  }

  std::string tmpdir = fs::temp_directory_path().string() + "/unicalib_l2calib_" +
    sensor_imu.sensor_id + "_" + sensor_lidar.sensor_id;
  fs::create_directories(tmpdir);
  std::string imu_csv = tmpdir + "/imu.csv";
  if (!write_imu_csv(imu_csv, *imu_opt)) {
    LOG_WARNING("learn-to-calibrate: failed to write imu.csv");
    return std::nullopt;
  }

#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
  std::string cfg_path = tmpdir + "/config.yaml";
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "imu_topic" << YAML::Value << sensor_imu.topic;
    out << YAML::Key << "lidar_topic" << YAML::Value << sensor_lidar.topic;
    out << YAML::Key << "imu_data" << YAML::Value << imu_csv;
    out << YAML::Key << "lidar_data" << YAML::Value << lidar_dir.string();
    out << YAML::Key << "output_dir" << YAML::Value << tmpdir;
    out << YAML::EndMap;
    std::ofstream f(cfg_path);
    if (!f || !(f << out.c_str())) {
      LOG_WARNING("learn-to-calibrate: failed to write config.yaml");
      return std::nullopt;
    }
  }

  std::string python_exe = find_python3();
  std::unordered_map<std::string, std::string> env;
  env["PYTHONPATH"] = tools_config.learn_to_calibrate + ":" +
    tools_config.learn_to_calibrate + "/rl_solver";

  ProcessResult res = run_process(
    {python_exe, calib_rl, "--config", cfg_path},
    env, timeout_seconds, "");
  if (res.exit_code != 0 || res.timed_out) {
    LOG_WARNING("learn-to-calibrate process failed: exit=" + std::to_string(res.exit_code) +
      " stderr=" + res.stderr_output.substr(0, 400));
    return std::nullopt;
  }

  std::string result_yaml = tmpdir + "/extrinsic_result.yaml";
  if (!fs::exists(result_yaml)) {
    LOG_WARNING("learn-to-calibrate did not produce extrinsic_result.yaml");
    return std::nullopt;
  }

  try {
    YAML::Node node = YAML::LoadFile(result_yaml);
    CalibResult out;
    out.pair = {sensor_imu.sensor_id, sensor_lidar.sensor_id};
    out.method_used = "learn-to-calibrate_RL";
    out.confidence = node["confidence"] ? node["confidence"].as<double>(0.5) : 0.5;

    if (node["translation"] && node["translation"].IsSequence() && node["translation"].size() >= 3) {
      out.translation(0) = node["translation"][0].as<double>();
      out.translation(1) = node["translation"][1].as<double>();
      out.translation(2) = node["translation"][2].as<double>();
    }

    if (node["rotation"]) {
      const YAML::Node& R = node["rotation"];
      if (R.IsSequence()) {
        if (R.size() == 4) {
          // scipy / Python 常用 (x,y,z,w)；Eigen::Quaterniond 为 (w,x,y,z)
          double x = R[0].as<double>(), y = R[1].as<double>(), z = R[2].as<double>(), w = R[3].as<double>();
          Eigen::Quaterniond q(w, x, y, z);
          out.rotation = q.toRotationMatrix();
        } else if (R.size() == 9) {
          for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
              out.rotation(i, j) = R[i * 3 + j].as<double>();
        }
      }
    }
    LOG_INFO("learn-to-calibrate OK: " + sensor_imu.sensor_id + " -> " + sensor_lidar.sensor_id);
    return out;
  } catch (const std::exception& e) {
    LOG_WARNING("learn-to-calibrate parse result failed: " + std::string(e.what()));
    return std::nullopt;
  }
#else
  (void)timeout_seconds;
  LOG_WARNING("learn-to-calibrate requires yaml-cpp for config/result.");
  return std::nullopt;
#endif
}

}  // namespace unicalib
