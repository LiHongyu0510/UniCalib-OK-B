/**
 * UniCalib C++ — click_calib BA：自动关键点、写 YAML、调 optimize.py、解析结果
 */
#include "unicalib/click_calib_wrapper.hpp"
#include "unicalib/process_runner.hpp"
#include "unicalib/logger.hpp"
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cmath>

#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
#include <yaml-cpp/yaml.h>
#endif

#if defined(UNICALIB_USE_OPENCV) && UNICALIB_USE_OPENCV
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace fs = std::filesystem;
namespace unicalib {

namespace {

constexpr double kSyncThresholdSec = 0.1;
constexpr int kMinKeypoints = 10;
constexpr int kMaxFramesForKeypoints = 5;

}  // namespace

std::optional<CalibResult> run_click_calib_ba(
  DataManager& data_mgr,
  const SensorConfig& sensor_a,
  const SensorConfig& sensor_b,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const std::optional<CalibResult>& initial,
  const ExternalToolsConfig& tools_config,
  int timeout_seconds) {

  if (tools_config.click_calib.empty()) {
    LOG_WARNING("click_calib not configured for Camera-Camera bundle adjustment.");
    LOG_WARNING("Set 'third_party.click_calib' in config.yaml or set UNICALIB_CLICK_CALIB environment variable.");
    LOG_WARNING("Example config:");
    LOG_WARNING("  third_party:");
    LOG_WARNING("    click_calib: \"/path/to/click_calib\"");
    LOG_WARNING("Example environment variable:");
    LOG_WARNING("  export UNICALIB_CLICK_CALIB=/path/to/click_calib");
    return std::nullopt;
  }

  auto it_a = intrinsics.find(sensor_a.sensor_id);
  auto it_b = intrinsics.find(sensor_b.sensor_id);
  if (it_a == intrinsics.end() || it_b == intrinsics.end()) {
    LOG_WARNING("click_calib: missing intrinsics for " + sensor_a.sensor_id + " or " + sensor_b.sensor_id);
    return std::nullopt;
  }

#if !(defined(UNICALIB_USE_OPENCV) && UNICALIB_USE_OPENCV) || !(defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML)
  LOG_WARNING("click_calib BA requires OpenCV and yaml-cpp");
  return std::nullopt;
#else

  // --- 收集同步图像路径 ---
  std::vector<std::pair<double, std::string>> paths_a, paths_b;
  data_mgr.iter_image_paths(sensor_a.topic, [&](double ts, const std::string& path) {
    paths_a.emplace_back(ts, path);
  }, (kMaxFramesForKeypoints + 2) * 2, 1);
  data_mgr.iter_image_paths(sensor_b.topic, [&](double ts, const std::string& path) {
    paths_b.emplace_back(ts, path);
  }, (kMaxFramesForKeypoints + 2) * 2, 1);
  if (paths_a.empty() || paths_b.empty()) {
    LOG_WARNING("click_calib: no image paths for " + sensor_a.topic + " or " + sensor_b.topic);
    return std::nullopt;
  }
  std::sort(paths_a.begin(), paths_a.end());
  std::sort(paths_b.begin(), paths_b.end());

  // --- SIFT 提取匹配点对（多帧聚合）---
  std::vector<std::pair<double, double>> keypoints_a, keypoints_b;  // (u,v) per point

  cv::Ptr<cv::SIFT> sift = cv::SIFT::create(200);
  int frames_used = 0;
  for (const auto& [ts_a, path_a] : paths_a) {
    if (frames_used >= kMaxFramesForKeypoints) break;
    auto it = std::lower_bound(paths_b.begin(), paths_b.end(), ts_a - kSyncThresholdSec,
        [](const std::pair<double, std::string>& x, double t) { return x.first < t; });
    if (it == paths_b.end() || std::abs(it->first - ts_a) > kSyncThresholdSec) continue;

    cv::Mat img_a = cv::imread(path_a);
    cv::Mat img_b = cv::imread(it->second);
    if (img_a.empty() || img_b.empty()) continue;

    cv::Mat gray_a, gray_b;
    cv::cvtColor(img_a, gray_a, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_b, gray_b, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> kp_a, kp_b;
    cv::Mat desc_a, desc_b;
    sift->detectAndCompute(gray_a, cv::noArray(), kp_a, desc_a);
    sift->detectAndCompute(gray_b, cv::noArray(), kp_b, desc_b);
    if (desc_a.empty() || desc_b.empty()) continue;

    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(desc_a, desc_b, matches);
    for (const auto& m : matches) {
      const auto& pa = kp_a[m.queryIdx].pt;
      const auto& pb = kp_b[m.trainIdx].pt;
      keypoints_a.emplace_back(static_cast<double>(pa.x), static_cast<double>(pa.y));
      keypoints_b.emplace_back(static_cast<double>(pb.x), static_cast<double>(pb.y));
    }
    ++frames_used;
  }

  if (keypoints_a.size() < static_cast<size_t>(kMinKeypoints)) {
    LOG_WARNING("click_calib: insufficient keypoints (" + std::to_string(keypoints_a.size()) + " < " + std::to_string(kMinKeypoints) + ")");
    return std::nullopt;
  }

  std::string tmpdir = (fs::temp_directory_path() / "unicalib_click_calib").string();
  try { fs::create_directories(tmpdir); } catch (...) {
    LOG_ERROR("click_calib: failed to create " + tmpdir);
    return std::nullopt;
  }

  std::string calib_path = tmpdir + "/calib_init.yaml";
  std::string kp_path = tmpdir + "/keypoints.yaml";
  std::string output_path = tmpdir + "/result.yaml";

  // --- 写 calib_init.yaml ---
  YAML::Emitter calib_out;
  calib_out << YAML::BeginMap;
  calib_out << YAML::Key << "cameras" << YAML::Value << YAML::BeginSeq;

  for (const auto* it : {&it_a->second, &it_b->second}) {
    const std::string& sid = (it == &it_a->second) ? sensor_a.sensor_id : sensor_b.sensor_id;
    calib_out << YAML::BeginMap;
    calib_out << YAML::Key << "camera_id" << YAML::Value << sid;
    if (it->camera_pinhole) {
      const auto& K = it->camera_pinhole->K;
      const auto& d = it->camera_pinhole->dist_coeffs;
      calib_out << YAML::Key << "intrinsic_type" << YAML::Value << "pinhole";
      calib_out << YAML::Key << "fx" << YAML::Value << K(0, 0);
      calib_out << YAML::Key << "fy" << YAML::Value << K(1, 1);
      calib_out << YAML::Key << "cx" << YAML::Value << K(0, 2);
      calib_out << YAML::Key << "cy" << YAML::Value << K(1, 2);
      calib_out << YAML::Key << "k1" << YAML::Value << (d.size() > 0 ? d[0] : 0.0);
      calib_out << YAML::Key << "k2" << YAML::Value << (d.size() > 1 ? d[1] : 0.0);
      calib_out << YAML::Key << "p1" << YAML::Value << (d.size() > 2 ? d[2] : 0.0);
      calib_out << YAML::Key << "p2" << YAML::Value << (d.size() > 3 ? d[3] : 0.0);
    } else if (it->camera_fisheye) {
      calib_out << YAML::Key << "intrinsic_type" << YAML::Value << it->camera_fisheye->model_type;
      for (const auto& [k, v] : it->camera_fisheye->params)
        calib_out << YAML::Key << k << YAML::Value << v;
    }
    calib_out << YAML::EndMap;
  }
  calib_out << YAML::EndSeq;

  if (initial) {
    calib_out << YAML::Key << "initial_extrinsic" << YAML::Value << YAML::BeginMap;
    calib_out << YAML::Key << "from" << YAML::Value << initial->pair.first;
    calib_out << YAML::Key << "to" << YAML::Value << initial->pair.second;
    calib_out << YAML::Key << "rotation" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i)
      calib_out << YAML::Flow << YAML::BeginSeq << initial->rotation(i,0) << initial->rotation(i,1) << initial->rotation(i,2) << YAML::EndSeq;
    calib_out << YAML::EndSeq;
    calib_out << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq
              << initial->translation(0) << initial->translation(1) << initial->translation(2) << YAML::EndSeq;
    calib_out << YAML::EndMap;
  }
  calib_out << YAML::EndMap;

  std::ofstream f_calib(calib_path);
  if (!f_calib) { LOG_ERROR("click_calib: cannot write " + calib_path); return std::nullopt; }
  f_calib << calib_out.c_str();

  // --- 写 keypoints.yaml: camera_a / camera_b 为 [[u,v],...] ---
  YAML::Emitter kp_out;
  kp_out << YAML::BeginMap;
  kp_out << YAML::Key << "camera_a" << YAML::Value << YAML::BeginSeq;
  for (const auto& p : keypoints_a)
    kp_out << YAML::Flow << YAML::BeginSeq << p.first << p.second << YAML::EndSeq;
  kp_out << YAML::EndSeq;
  kp_out << YAML::Key << "camera_b" << YAML::Value << YAML::BeginSeq;
  for (const auto& p : keypoints_b)
    kp_out << YAML::Flow << YAML::BeginSeq << p.first << p.second << YAML::EndSeq;
  kp_out << YAML::EndSeq << YAML::EndMap;

  std::ofstream f_kp(kp_path);
  if (!f_kp) { LOG_ERROR("click_calib: cannot write " + kp_path); return std::nullopt; }
  f_kp << kp_out.c_str();

  // --- 调用 python source/optimize.py ---
  std::string script = tools_config.click_calib + "/source/optimize.py";
  if (!fs::exists(script)) {
    LOG_WARNING("click_calib: optimize.py not found at " + script);
    return std::nullopt;
  }
  std::string python = find_python3();
  if (python.empty()) { LOG_WARNING("click_calib: no python3"); return std::nullopt; }

  std::unordered_map<std::string, std::string> env;
  env["PYTHONPATH"] = tools_config.click_calib + "/source";

  ProcessResult res = run_process(
    { python, script, "--calib", calib_path, "--keypoints", kp_path, "--output", output_path },
    env, timeout_seconds, "");

  if (res.exit_code != 0 || res.timed_out) {
    LOG_WARNING("click_calib optimize exit_code=" + std::to_string(res.exit_code));
    return std::nullopt;
  }

  if (!fs::exists(output_path)) {
    LOG_WARNING("click_calib: result not found " + output_path);
    return std::nullopt;
  }

  YAML::Node data;
  try {
    data = YAML::LoadFile(output_path);
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("click_calib result parse: ") + e.what());
    return std::nullopt;
  }

  CalibResult cr;
  cr.pair = {sensor_a.sensor_id, sensor_b.sensor_id};
  cr.rotation = Eigen::Matrix3d::Identity();
  cr.translation = Eigen::Vector3d::Zero();
  if (data["rotation"]) {
    const auto& Rnode = data["rotation"];
    for (int i = 0; i < 3 && i < static_cast<int>(Rnode.size()); ++i)
      for (int j = 0; j < 3 && j < static_cast<int>(Rnode[i].size()); ++j)
        cr.rotation(i, j) = Rnode[i][j].as<double>();
  }
  if (data["translation"]) {
    const auto& tnode = data["translation"];
    for (size_t i = 0; i < 3 && i < tnode.size(); ++i)
      cr.translation(i) = tnode[i].as<double>();
  }
  if (data["mde"]) cr.reprojection_error = data["mde"].as<double>();
  cr.confidence = 0.8;
  cr.method_used = "click_calib_BA";
  LOG_INFO("click_calib BA result " + cr.pair.first + "->" + cr.pair.second + " mde=" + std::to_string(cr.reprojection_error));
  return cr;
#endif
}

}  // namespace unicalib
