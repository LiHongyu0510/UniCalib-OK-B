/**
 * UniCalib C++ — 系统主控实现 (配置解析 + 分阶段占位 + 异常处理与日志)
 */
#include "unicalib/system.hpp"
#include "unicalib/sensor_config.hpp"
#include "unicalib/allan_variance.hpp"
#include "unicalib/imu_intrinsic.hpp"
#include "unicalib/camera_fisheye.hpp"
#include "unicalib/reprojection.hpp"
#include "unicalib/reprojection.hpp"
#include "unicalib/report_generator.hpp"
#include "unicalib/exceptions.hpp"
#include "unicalib/logger.hpp"
#include "unicalib/data_validator.hpp"
#include "unicalib/calibration_quality.hpp"
#include "unicalib/dm_calib_wrapper.hpp"
#include "unicalib/learn_to_calib_wrapper.hpp"
#include "unicalib/mias_lcec_wrapper.hpp"
#include "unicalib/feature_matching.hpp"
#include "unicalib/ikalibr_wrapper.hpp"
#include "unicalib/click_calib_wrapper.hpp"
#include "unicalib/process_runner.hpp"
#include "unicalib/external_tools_config.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>

#if defined(UNICALIB_USE_YAML) && UNICALIB_USE_YAML
#include <yaml-cpp/yaml.h>
#define HAS_YAML 1
#else
#define HAS_YAML 0
#endif

namespace unicalib {

namespace {

#if HAS_YAML
void load_config_yaml(const std::string& config_path, std::string& output_dir, std::vector<SensorConfig>& sensors) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const std::exception& e) {
    throw ConfigException(ErrorCode::CONFIG_PARSE_ERROR, "load_config", std::string("YAML parse error: ") + e.what());
  }
  auto sys = root["system"];
  if (sys && sys["output_dir"]) output_dir = sys["output_dir"].as<std::string>();
  auto sens = root["sensors"];
  if (!sens || !sens.IsSequence()) return;
  for (size_t i = 0; i < sens.size(); ++i) {
    SensorConfig s;
    s.sensor_id = sens[i]["sensor_id"].as<std::string>("");
    s.sensor_type = sensor_type_from_string(sens[i]["sensor_type"].as<std::string>("camera_pinhole"));
    s.topic = sens[i]["topic"].as<std::string>("");
    s.frame_id = sens[i]["frame_id"].as<std::string>("");
    if (sens[i]["rate"]) s.rate = sens[i]["rate"].as<double>();
    if (sens[i]["resolution"] && sens[i]["resolution"].IsSequence() && sens[i]["resolution"].size() >= 2)
      s.resolution = {sens[i]["resolution"][0].as<int>(), sens[i]["resolution"][1].as<int>()};
    sensors.push_back(s);
  }
}

void load_tools_config_yaml(const std::string& config_path, ExternalToolsConfig& tools) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (...) {
    return;
  }
  auto tp = root["third_party"];
  if (!tp) return;
  if (tp["dm_calib"]) tools.dm_calib = tp["dm_calib"].as<std::string>("");
  if (tp["learn_to_calibrate"]) tools.learn_to_calibrate = tp["learn_to_calibrate"].as<std::string>("");
  if (tp["mias_lcec"]) tools.mias_lcec = tp["mias_lcec"].as<std::string>("");
  if (tp["ikalibr"]) tools.ikalibr = tp["ikalibr"].as<std::string>("");
  if (tp["click_calib"]) tools.click_calib = tp["click_calib"].as<std::string>("");
  if (tp["transformer_imu"]) tools.transformer_imu = tp["transformer_imu"].as<std::string>("");
}
#endif

void ensure_output_dir(const std::string& output_dir) {
  std::error_code ec;
  if (!std::filesystem::create_directories(output_dir, ec) && !std::filesystem::exists(output_dir))
    throw IOException(ErrorCode::IO_WRITE_FAILED, output_dir, "Failed to create output directory: " + ec.message());
}

}  // namespace

UniCalibSystem::UniCalibSystem(const std::string& config_path) : config_path_(config_path) {
  Logger::instance().init("unicalib", LogLevel::INFO);
  load_config();
  parse_config();
}

void UniCalibSystem::load_config() {
  output_dir_ = "./calib_results";
#if HAS_YAML
  try {
    if (std::filesystem::exists(config_path_))
      load_config_yaml(config_path_, output_dir_, sensors_list_);
    else
      LOG_WARNING("Config file not found: " + config_path_ + ", using defaults.");
  } catch (const ConfigException& e) {
    LOG_EXCEPTION(e);
    throw;
  } catch (const std::exception& e) {
    LOG_WARNING(std::string("Config load failed: ") + e.what() + ", using defaults.");
  }
#else
  (void)config_path_;
  LOG_WARNING("yaml-cpp not found, using default config.");
#endif
  try {
    ensure_output_dir(output_dir_);
  } catch (const IOException& e) {
    LOG_EXCEPTION(e);
    throw;
  }
  load_tools_config();
}

void UniCalibSystem::load_tools_config() {
#if HAS_YAML
  if (std::filesystem::exists(config_path_))
    load_tools_config_yaml(config_path_, tools_config_);
#endif
  tools_config_.apply_env_defaults();
  if (tools_config_.integration_enabled())
    LOG_INFO("External tools integration enabled (third_party / env).");
}

void UniCalibSystem::parse_config() {
  calib_pairs_ = auto_infer_calib_pairs(sensors_list_);
}

void UniCalibSystem::run_full_pipeline(const std::string& data_path) {
  LOG_INFO("Starting full calibration pipeline, data_path=" + data_path);
  pipeline_data_path_ = data_path;
  DataManager data_mgr(data_path);
  try {
    data_mgr.open();
  } catch (const std::exception& e) {
    LOG_EXCEPTION(e);
    throw;
  }
  try {
    stage_intrinsic(data_mgr);
    stage_coarse_extrinsic(data_mgr);
    stage_fine_extrinsic(data_mgr);
    ValidationReport report;
    stage_validation(data_mgr, report);
    report.overall_pass = report.metrics.empty() || std::all_of(
      report.metrics.begin(), report.metrics.end(),
      [](const auto& p) { return p.second.extra.count("pass") && p.second.extra.at("pass") >= 0.5; });
    if (report.summary.empty())
      report.summary = report.overall_pass ? "PASS" : "FAIL (validation)";
    save_results(report);
  } catch (const UniCalibException& e) {
    LOG_EXCEPTION(e);
    data_mgr.close();
    throw;
  } catch (const std::exception& e) {
    LOG_EXCEPTION(e);
    data_mgr.close();
    throw;
  }
  data_mgr.close();
  LOG_INFO("Pipeline finished successfully.");
}

void UniCalibSystem::run_stage(const std::string& stage, const std::string& data_path) {
  DataManager data_mgr(data_path);
  data_mgr.open();
  try {
    if (stage == "intrinsic") stage_intrinsic(data_mgr);
    else if (stage == "coarse") stage_coarse_extrinsic(data_mgr);
    else if (stage == "fine") stage_fine_extrinsic(data_mgr);
    else if (stage == "validate") {
      ValidationReport r;
      stage_validation(data_mgr, r);
      r.overall_pass = r.metrics.empty() || std::all_of(r.metrics.begin(), r.metrics.end(),
        [](const auto& p) { return p.second.extra.count("pass") && p.second.extra.at("pass") >= 0.5; });
      if (r.summary.empty()) r.summary = r.overall_pass ? "PASS" : "FAIL";
      save_results(r);
    }
  } catch (...) {
    data_mgr.close();
    throw;
  }
  data_mgr.close();
}

void UniCalibSystem::stage_intrinsic(DataManager& data_mgr) {
  intrinsic_results_.clear();
  DataValidator validator;
  CalibrationQualityChecker quality_checker;
  IMUIntrinsicCalibrator imu_calib;
  CameraFisheyeCalibrator fisheye_calib;
  for (const auto& s : sensors_list_) {
    IntrinsicResultHolder h;
    if (s.sensor_type == SensorType::IMU) {
      try {
        PERF_TIMER("IMU intrinsic " + s.sensor_id);
        h.imu = imu_calib.calibrate(data_mgr, s, &validator, &quality_checker);
        if (h.imu)
          LOG_INFO("IMU intrinsic calibrated: " + s.sensor_id + " method=" + h.imu->method);
      } catch (const UniCalibException& e) {
        LOG_EXCEPTION(e);
        throw;
      } catch (const std::exception& e) {
        LOG_ERROR("IMU calibrate failed for " + s.sensor_id + ": " + e.what());
        throw CalibrationException(ErrorCode::CALIBRATION_FAILED, "calibrate",
                                   std::string("sensor ") + s.sensor_id + ": " + e.what());
      }
    } else if (s.sensor_type == SensorType::CAMERA_PINHOLE) {
      try {
        PERF_TIMER("Pinhole intrinsic (DM-Calib) " + s.sensor_id);
        auto opt = run_dm_calib(data_mgr, s, tools_config_, 300);
        if (opt) {
          h.camera_pinhole = *opt;
          LOG_INFO("Pinhole intrinsic calibrated: " + s.sensor_id + " method=" + opt->method);
        }
      } catch (const std::exception& e) {
        LOG_WARNING("DM-Calib / pinhole failed for " + s.sensor_id + ": " + e.what());
      }
    } else if (s.sensor_type == SensorType::CAMERA_FISHEYE) {
      try {
        PERF_TIMER("Fisheye intrinsic " + s.sensor_id);
        h.camera_fisheye = fisheye_calib.calibrate(data_mgr, s);
      } catch (const std::exception& e) {
        LOG_WARNING("Fisheye calibrate failed for " + s.sensor_id + ": " + e.what());
      }
    }
    intrinsic_results_[s.sensor_id] = h;
  }
}

void UniCalibSystem::stage_coarse_extrinsic(DataManager& data_mgr) {
  extrinsic_results_.clear();
  auto find_sensor = [this](const std::string& id) -> const SensorConfig* {
    for (const auto& s : sensors_list_)
      if (s.sensor_id == id) return &s;
    return nullptr;
  };

  for (const auto& p : calib_pairs_) {
    std::string key = p.sensor_a + ":" + p.sensor_b;
    CalibResult r;
    r.pair = {p.sensor_a, p.sensor_b};
    r.rotation = Eigen::Matrix3d::Identity();
    r.translation = Eigen::Vector3d::Zero();
    r.method_used = p.method_coarse + "_placeholder";

    if (p.method_coarse == "l2calib_rl_init" && tools_config_.learn_to_calibrate.size() > 0) {
      const SensorConfig* sa = find_sensor(p.sensor_a);
      const SensorConfig* sb = find_sensor(p.sensor_b);
      if (sa && sb && sa->is_imu() && sb->is_lidar()) {
        auto opt = run_learn_to_calib(data_mgr, pipeline_data_path_, *sa, *sb, tools_config_, 600);
        if (opt) {
          r = *opt;
          LOG_INFO("Coarse extrinsic (learn-to-calibrate): " + key);
        }
      } else if (sa && sb && sa->is_lidar() && sb->is_imu()) {
        auto opt = run_learn_to_calib(data_mgr, pipeline_data_path_, *sb, *sa, tools_config_, 600);
        if (opt) {
          r = *opt;
          r.pair = {p.sensor_a, p.sensor_b};
          r.rotation = opt->rotation.transpose();
          r.translation = -opt->rotation.transpose() * opt->translation;
          LOG_INFO("Coarse extrinsic (learn-to-calibrate): " + key);
        }
      }
    }

    if (p.method_coarse == "feature_matching") {
      const SensorConfig* sa = find_sensor(p.sensor_a);
      const SensorConfig* sb = find_sensor(p.sensor_b);
      if (sa && sb && sa->is_camera() && sb->is_camera()) {
        auto opt = run_feature_matching_coarse(data_mgr, *sa, *sb, intrinsic_results_, 30, 20);
        if (opt) { 
          r = *opt; 
          LOG_INFO("Coarse extrinsic (feature_matching): " + key); 
        } else {
          LOG_WARNING("Feature matching failed for " + key + ", using identity as fallback");
          // 仍然保留identity，但不标记为placeholder
          r.method_used = "feature_matching_fallback";
        }
      }
    }

    if (p.method_coarse == "mias_lcec_coarse" && !tools_config_.mias_lcec.empty()) {
      const SensorConfig* sa = find_sensor(p.sensor_a);
      const SensorConfig* sb = find_sensor(p.sensor_b);
      if (sa && sb && sa->is_lidar() && sb->is_camera()) {
        auto opt = run_mias_lcec_coarse(data_mgr, pipeline_data_path_, *sa, *sb,
                                        intrinsic_results_, tools_config_, 600);
        if (opt) { r = *opt; LOG_INFO("Coarse extrinsic (MIAS-LCEC): " + key); }
      } else if (sa && sb && sa->is_camera() && sb->is_lidar()) {
        auto opt = run_mias_lcec_coarse(data_mgr, pipeline_data_path_, *sb, *sa,
                                        intrinsic_results_, tools_config_, 600);
        if (opt) {
          r = *opt;
          r.pair = {p.sensor_a, p.sensor_b};
          r.rotation = opt->rotation.transpose();
          r.translation = -opt->rotation.transpose() * opt->translation;
          LOG_INFO("Coarse extrinsic (MIAS-LCEC): " + key);
        }
      }
    }

    extrinsic_results_[key] = r;
  }
}

void UniCalibSystem::stage_fine_extrinsic(DataManager& data_mgr) {
  auto find_sensor = [this](const std::string& id) -> const SensorConfig* {
    for (const auto& s : sensors_list_)
      if (s.sensor_id == id) return &s;
    return nullptr;
  };

  // iKalibr 联合精化：若有任一对使用 ikalibr_bspline，则调用一次，再合并结果
  bool any_ikalibr = false;
  for (const auto& p : calib_pairs_)
    if (p.method_fine == "ikalibr_bspline") { any_ikalibr = true; break; }
  if (any_ikalibr && !tools_config_.ikalibr.empty()) {
    auto refined = run_ikalibr_joint(
      data_mgr, pipeline_data_path_, sensors_list_,
      intrinsic_results_, extrinsic_results_, tools_config_, 1800);
    if (refined) {
      for (const auto& [k, r] : *refined)
        extrinsic_results_[k] = r;
      LOG_INFO("Fine extrinsic (iKalibr joint): merged " + std::to_string(refined->size()) + " pairs.");
    }
  }

  for (const auto& p : calib_pairs_) {
    std::string key = p.sensor_a + ":" + p.sensor_b;
    auto it = extrinsic_results_.find(key);
    if (it == extrinsic_results_.end()) continue;

    if (p.method_fine == "mias_lcec_fine" && !tools_config_.mias_lcec.empty()) {
      const SensorConfig* sa = find_sensor(p.sensor_a);
      const SensorConfig* sb = find_sensor(p.sensor_b);
      if (sa && sb && sa->is_lidar() && sb->is_camera()) {
        auto opt = run_mias_lcec_fine(data_mgr, pipeline_data_path_, *sa, *sb,
                                     intrinsic_results_, it->second, tools_config_, 600);
        if (opt) {
          extrinsic_results_[key] = *opt;
          LOG_INFO("Fine extrinsic (MIAS-LCEC): " + key);
        }
      } else if (sa && sb && sa->is_camera() && sb->is_lidar()) {
        auto opt = run_mias_lcec_fine(data_mgr, pipeline_data_path_, *sb, *sa,
                                     intrinsic_results_, it->second, tools_config_, 600);
        if (opt) {
          CalibResult rev;
          rev.pair = {p.sensor_a, p.sensor_b};
          rev.rotation = opt->rotation.transpose();
          rev.translation = -opt->rotation.transpose() * opt->translation;
          rev.time_offset = opt->time_offset;
          rev.reprojection_error = opt->reprojection_error;
          rev.method_used = opt->method_used;
          extrinsic_results_[key] = rev;
          LOG_INFO("Fine extrinsic (MIAS-LCEC): " + key);
        }
      }
    }

    if (p.method_fine == "click_calib_ba" && !tools_config_.click_calib.empty()) {
      const SensorConfig* sa = find_sensor(p.sensor_a);
      const SensorConfig* sb = find_sensor(p.sensor_b);
      if (sa && sb && sa->is_camera() && sb->is_camera()) {
        // 检查是否有粗外参作为初值
        std::optional<CalibResult> initial = it->second;
        bool use_initial = (initial.has_value() && 
                           initial->method_used != "feature_matching_placeholder");
        
        if (!use_initial) {
          LOG_WARNING("No valid coarse extrinsic for " + key + ", click_calib will start from identity");
          // 创建identity作为初始值
          CalibResult id_init;
          id_init.pair = {p.sensor_a, p.sensor_b};
          id_init.rotation = Eigen::Matrix3d::Identity();
          id_init.translation = Eigen::Vector3d::Zero();
          id_init.method_used = "identity_init";
          initial = id_init;
        }
        
        auto opt = run_click_calib_ba(data_mgr, *sa, *sb, intrinsic_results_, initial, tools_config_, 300);
        if (opt) {
          extrinsic_results_[key] = *opt;
          LOG_INFO("Fine extrinsic (click_calib BA): " + key);
        } else {
          LOG_WARNING("click_calib BA failed for " + key + ", keeping coarse result");
          // 保留粗外参结果
          if (initial) {
            extrinsic_results_[key] = *initial;
          }
        }
      }
    }
  }
}

void UniCalibSystem::stage_validation(DataManager& data_mgr, ValidationReport& report) {
  ReprojectionValidator validator;
  for (const auto& p : calib_pairs_) {
    const SensorConfig* sa = nullptr;
    const SensorConfig* sb = nullptr;
    for (const auto& s : sensors_list_) {
      if (s.sensor_id == p.sensor_a) sa = &s;
      if (s.sensor_id == p.sensor_b) sb = &s;
    }
    if (!sa || !sb) continue;
    bool lidar_cam = (sa->is_lidar() && sb->is_camera()) || (sa->is_camera() && sb->is_lidar());
    if (!lidar_cam) continue;
    std::string lidar_id = sa->is_lidar() ? sa->sensor_id : sb->sensor_id;
    std::string cam_id = sa->is_camera() ? sa->sensor_id : sb->sensor_id;
    std::string key = p.sensor_a + ":" + p.sensor_b;
    auto it = extrinsic_results_.find(key);
    if (it == extrinsic_results_.end()) continue;
    auto metrics = validator.validate(data_mgr, lidar_id, cam_id, sensors_list_, intrinsic_results_, it->second);
    report.add_metric(key, metrics);
  }
}

void UniCalibSystem::save_results(const ValidationReport& validation) {
  ReportGenerator reporter(output_dir_);
  reporter.print_summary(intrinsic_results_, extrinsic_results_, validation);
  reporter.generate_html(intrinsic_results_, extrinsic_results_, validation);

#if HAS_YAML
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& [sid, h] : intrinsic_results_) {
      if (h.imu) {
        out << YAML::Key << sid << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "type" << YAML::Value << "imu";
        out << YAML::Key << "gyro_noise" << YAML::Value << h.imu->gyro_noise;
        out << YAML::Key << "gyro_bias_instability" << YAML::Value << h.imu->gyro_bias_instability;
        out << YAML::Key << "accel_noise" << YAML::Value << h.imu->accel_noise;
        out << YAML::Key << "accel_bias_instability" << YAML::Value << h.imu->accel_bias_instability;
        out << YAML::Key << "method" << YAML::Value << h.imu->method;
        out << YAML::EndMap;
      } else if (h.camera_pinhole) {
        out << YAML::Key << sid << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "type" << YAML::Value << "camera_pinhole";
        out << YAML::Key << "image_size" << YAML::Value << YAML::Flow
            << YAML::BeginSeq << h.camera_pinhole->image_size.first
            << h.camera_pinhole->image_size.second << YAML::EndSeq;
        out << YAML::Key << "reprojection_error" << YAML::Value << h.camera_pinhole->reprojection_error;
        out << YAML::Key << "method" << YAML::Value << h.camera_pinhole->method;
        out << YAML::Key << "K" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) out << h.camera_pinhole->K(i, j);
        out << YAML::EndSeq;
        out << YAML::Key << "dist_coeffs" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (double d : h.camera_pinhole->dist_coeffs) out << d;
        out << YAML::EndSeq;
        out << YAML::EndMap;
      } else if (h.camera_fisheye) {
        out << YAML::Key << sid << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "type" << YAML::Value << "camera_fisheye";
        out << YAML::Key << "model_type" << YAML::Value << h.camera_fisheye->model_type;
        out << YAML::Key << "image_size" << YAML::Value << YAML::Flow
            << YAML::BeginSeq << h.camera_fisheye->image_size.first
            << h.camera_fisheye->image_size.second << YAML::EndSeq;
        out << YAML::Key << "reprojection_error" << YAML::Value << h.camera_fisheye->reprojection_error;
        out << YAML::Key << "method" << YAML::Value << h.camera_fisheye->method;
        out << YAML::Key << "params" << YAML::Value << YAML::BeginMap;
        for (const auto& [k, v] : h.camera_fisheye->params)
          out << YAML::Key << k << YAML::Value << v;
        out << YAML::EndMap;
        out << YAML::EndMap;
      }
    }
    out << YAML::EndMap;
    std::ofstream f(output_dir_ + "/intrinsics.yaml");
    if (!f || !(f << out.c_str()))
      LOG_ERROR("Failed to write " + output_dir_ + "/intrinsics.yaml");
    else
      LOG_INFO("Wrote intrinsics.yaml");
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("Save intrinsics.yaml: ") + e.what());
  }
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& [key, r] : extrinsic_results_) {
      std::string k = r.pair.first + "_to_" + r.pair.second;
      out << YAML::Key << k << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "pair" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << r.pair.first << r.pair.second << YAML::EndSeq;
      out << YAML::Key << "rotation" << YAML::Value << YAML::BeginSeq;
      for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) out << r.rotation(i, j);
      out << YAML::EndSeq;
      out << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << r.translation(0) << r.translation(1) << r.translation(2) << YAML::EndSeq;
      out << YAML::Key << "time_offset" << YAML::Value << r.time_offset;
      out << YAML::Key << "method_used" << YAML::Value << r.method_used;
      out << YAML::EndMap;
    }
    out << YAML::EndMap;
    std::ofstream f(output_dir_ + "/extrinsics.yaml");
    if (!f || !(f << out.c_str()))
      LOG_ERROR("Failed to write extrinsics.yaml");
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("Save extrinsics.yaml: ") + e.what());
  }
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "overall_pass" << YAML::Value << validation.overall_pass;
    out << YAML::Key << "summary" << YAML::Value << validation.summary;
    out << YAML::EndMap;
    std::ofstream f(output_dir_ + "/validation_report.yaml");
    if (!f || !(f << out.c_str()))
      LOG_ERROR("Failed to write validation_report.yaml");
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("Save validation_report.yaml: ") + e.what());
  }
#endif
  LOG_INFO("Results saved to " + output_dir_);
}

}  // namespace unicalib
