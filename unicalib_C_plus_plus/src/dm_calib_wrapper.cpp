/**
 * UniCalib C++ — DM-Calib 封装实现：子进程调用 + 简单 JSON 解析
 */
#include "unicalib/dm_calib_wrapper.hpp"
#include "unicalib/logger.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <cmath>

namespace unicalib {

namespace {

// 从 JSON 文件内容中解析指定 key 对应的 double 值（简单扫描）
bool parse_json_double(const std::string& content, const std::string& key, double& out) {
  std::string search = "\"" + key + "\"";
  auto pos = content.find(search);
  if (pos == std::string::npos) return false;
  pos = content.find_first_not_of(" \t:", pos + search.size());
  if (pos == std::string::npos) return false;
  char* end = nullptr;
  out = std::strtod(content.c_str() + pos, &end);
  return end != content.c_str() + pos;
}

std::optional<CameraIntrinsic> parse_dm_calib_json(
  const std::string& json_path, int image_width, int image_height) {
  std::ifstream f(json_path);
  if (!f) return std::nullopt;
  std::stringstream buf;
  buf << f.rdbuf();
  std::string content = buf.str();

  double fx = image_width * 0.8, fy = image_height * 0.8;
  double cx = image_width / 2.0, cy = image_height / 2.0;
  double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0, confidence = 1.0;

  parse_json_double(content, "fx", fx);
  parse_json_double(content, "fy", fy);
  parse_json_double(content, "cx", cx);
  parse_json_double(content, "cy", cy);
  parse_json_double(content, "k1", k1);
  parse_json_double(content, "k2", k2);
  parse_json_double(content, "p1", p1);
  parse_json_double(content, "p2", p2);
  parse_json_double(content, "k3", k3);
  parse_json_double(content, "confidence", confidence);

  // 如果 JSON 中有 image_width/height，使用它们
  int w = image_width, h = image_height;
  double tmp;
  if (parse_json_double(content, "image_width", tmp)) w = static_cast<int>(tmp);
  if (parse_json_double(content, "image_height", tmp)) h = static_cast<int>(tmp);

  CameraIntrinsic out;
  out.K.setIdentity();
  out.K(0, 0) = fx;
  out.K(1, 1) = fy;
  out.K(0, 2) = cx;
  out.K(1, 2) = cy;
  out.image_size = {w, h};
  out.reprojection_error = confidence;
  out.method = "DM-Calib";
  out.dist_coeffs = {k1, k2, p1, p2, k3};
  return out;
}

}  // namespace

std::optional<CameraIntrinsic> run_dm_calib(
  DataManager& data_mgr,
  const SensorConfig& sensor,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds) {
  if (tools_config.dm_calib.empty()) {
    LOG_WARNING("DM-Calib not configured. Set 'third_party.dm_calib' in config.yaml or set UNICALIB_DM_CALIB environment variable.");
    LOG_WARNING("Example config:");
    LOG_WARNING("  third_party:");
    LOG_WARNING("    dm_calib: \"/path/to/DM-Calib\"");
    LOG_WARNING("Example environment variable:");
    LOG_WARNING("  export UNICALIB_DM_CALIB=/path/to/DM-Calib");
    LOG_WARNING("Default location: /path/to/calibration/DM-Calib");
    return std::nullopt;
  }

  namespace fs = std::filesystem;
  
  // 获取图像目录
  std::string image_dir;
  int path_count = 0;
  data_mgr.iter_image_paths(sensor.topic,
    [&](double /*ts*/, const std::string& path) {
      if (path_count++ == 0)
        image_dir = fs::path(path).parent_path().string();
    }, 100, 1);  // 获取更多图像以支持 ensemble
  
  if (image_dir.empty()) {
    LOG_WARNING("DM-Calib: no image path for " + sensor.sensor_id);
    return std::nullopt;
  }

  // 优先使用 UniCalib 专用推理脚本（输出 JSON）
  std::string infer_unicalib = tools_config.dm_calib + "/DMCalib/tools/infer_unicalib.py";
  std::string infer_py_orig = tools_config.dm_calib + "/DMCalib/tools/infer.py";
  std::string infer_py;
  bool use_unicalib_script = false;
  
  if (fs::exists(infer_unicalib)) {
    infer_py = infer_unicalib;
    use_unicalib_script = true;
  } else if (fs::exists(infer_py_orig)) {
    infer_py = infer_py_orig;
  } else {
    LOG_WARNING("DM-Calib infer script not found");
    return std::nullopt;
  }

  std::string model_dir = tools_config.dm_calib + "/model";
  std::string out_json = fs::temp_directory_path().string() + "/unicalib_dm_calib_" +
    sensor.sensor_id + ".json";
  std::string python_exe = find_python3();

  std::unordered_map<std::string, std::string> env;
  env["PYTHONPATH"] = tools_config.dm_calib;

  std::vector<std::string> argv;
  
  if (use_unicalib_script) {
    // 使用 UniCalib 专用脚本，直接输出 JSON
    argv = {
      python_exe, infer_py,
      "--pretrained_model_path", model_dir,
      "--image_dir", image_dir,
      "--output", out_json,
      "--max_images", "5",
      "--ensemble_size", "3"
    };
  } else {
    // 使用原始脚本（需要从输出目录读取）
    LOG_WARNING("Using original infer.py, JSON output may not be available");
    std::string out_dir = fs::temp_directory_path().string() + "/unicalib_dm_calib_" + sensor.sensor_id;
    fs::create_directories(out_dir);
    argv = {
      python_exe, infer_py,
      "--pretrained_model_path", model_dir,
      "--input_dir", image_dir,
      "--output_dir", out_dir
    };
    // 原始脚本不输出 JSON，这里无法解析
    out_json = out_dir + "/intrinsic.json";  // 尝试可能的输出路径
  }

  ProcessResult res = run_process(argv, env, timeout_seconds, "");
  if (res.exit_code != 0 || res.timed_out) {
    LOG_WARNING("DM-Calib process failed: exit=" + std::to_string(res.exit_code) +
      " timed_out=" + (res.timed_out ? "1" : "0") +
      " stderr=" + res.stderr_output.substr(0, 500));
    return std::nullopt;
  }

  if (!fs::exists(out_json)) {
    LOG_WARNING("DM-Calib did not produce output JSON: " + out_json);
    // 尝试从 stdout 解析 JSON（如果脚本输出到 stdout）
    if (!res.stdout_output.empty() && res.stdout_output.find("{") != std::string::npos) {
      // 将 stdout 写入临时文件尝试解析
      std::ofstream tmp(out_json);
      tmp << res.stdout_output;
      tmp.close();
    } else {
      return std::nullopt;
    }
  }

  int w = sensor.resolution.has_value() && sensor.resolution.value().first > 0
          ? sensor.resolution.value().first : 1920;
  int h = sensor.resolution.has_value() && sensor.resolution.value().second > 0
          ? sensor.resolution.value().second : 1080;
  auto intrinsic = parse_dm_calib_json(out_json, w, h);
  if (intrinsic) {
    LOG_INFO("DM-Calib intrinsic OK for " + sensor.sensor_id +
             ": fx=" + std::to_string(intrinsic->K(0,0)) +
             ", fy=" + std::to_string(intrinsic->K(1,1)));
  }
  return intrinsic;
}

}  // namespace unicalib
