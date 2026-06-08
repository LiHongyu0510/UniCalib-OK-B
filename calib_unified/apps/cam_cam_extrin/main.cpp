/**
 * UniCalib — Camera-Camera 外参标定应用
 *
 * 两阶段流程:
 *   Stage 1 粗标定: 特征匹配 + 本质矩阵初始化
 *   Stage 2 精标定: Bundle Adjustment (无目标) / 棋盘格立体标定
 *   手动校准:       --manual 触发 click-calib 风格双目对应点精化
 *
 * 用法:
 *   unicalib_cam_cam --config <config.yaml>
 *   unicalib_cam_cam --config <config.yaml> [--data-dir <dir>] [--ros2-bag <path>]
 *   unicalib_cam_cam --config <config.yaml> --method ba|stereo|essential
 *   unicalib_cam_cam --config <config.yaml> --manual
 *
 * 数据源与 LiDAR-Camera 一致：从统一 YAML 的 ros2 / data / sensors 段读取 bag 与相机话题
 * （PipelineConfig.use_ros2_bag + ros2_bag_file + camera_topics）。
 */
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/io/yaml_io.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
using namespace ns_unicalib;

static std::string resolve_data_path(const std::string& base, const std::string& path) {
    if (path.empty()) return "";
    std::string work = path;
    const char prefix[] = "/path/to/";
    if (!base.empty() && work.size() > sizeof(prefix) - 1 &&
        work.compare(0, sizeof(prefix) - 1, prefix) == 0) {
        work = work.substr(sizeof(prefix) - 1);
    }
    if (base.empty()) return path;
    fs::path p(work);
    if (p.is_absolute()) return path;
    return (fs::path(base) / p).lexically_normal().string();
}

static void resolve_path_map(const std::string& base, std::map<std::string, std::string>& paths) {
    if (base.empty()) return;
    for (auto& [k, v] : paths) {
        if (!v.empty()) v = resolve_data_path(base, v);
    }
}

static void resolve_pipeline_data_paths(PipelineConfig& pipe_cfg, const std::string& base) {
    if (base.empty()) return;
    resolve_path_map(base, pipe_cfg.camera_images_dirs);
    for (auto& [k, v] : pipe_cfg.camera_intrinsic_files) {
        if (v.empty()) continue;
        fs::path p(v);
        if (p.is_absolute()) continue;
        // 保留已存在于 cwd 的 results/ 等路径；否则相对 data-dir 解析
        if (fs::exists(v)) continue;
        v = resolve_data_path(base, v);
    }
}

static void load_cam_cam_pairs_from_cfg(const YAML::Node& root, PipelineConfig& pipe_cfg) {
    if (!root["cam_cam"]) return;
    const auto& cc = root["cam_cam"];
    auto add_pairs = [&](const YAML::Node& seq) {
        if (!seq || !seq.IsSequence()) return;
        for (const auto& p : seq) {
            if (p.IsSequence() && p.size() >= 2)
                pipe_cfg.cam_cam_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
    };
    if (cc["pairs"])
        add_pairs(cc["pairs"]);
    else if (cc["camera_align"])
        add_pairs(cc["camera_align"]);
    if (!pipe_cfg.cam_cam_pairs.empty())
        UNICALIB_INFO("[Cam-Cam] 已从配置读取 {} 对标定对", pipe_cfg.cam_cam_pairs.size());
}

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — Camera-Camera 外参标定                   ║
 ║   两阶段: 特征匹配初始化 → BA精化(无目标优先)          ║
 ║   手动校准: --manual 触发 click-calib 对应点精化       ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_cam_cam [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --data-dir <dir>        数据根目录（解析 ros2_bag_file 相对路径；默认环境变量 CALIB_DATA_DIR）\n"
        "  --ros2-bag <path>       覆盖配置中的 ROS2 bag 路径（并启用 bag 模式）\n"
        "  --method <m>            精标定方法: ba(默认)|stereo|essential\n"
        "  --manual                精标定后启用手动校准\n"
        "  --no-viz                禁用可视化界面 (默认启用)\n"
        "  --no-targetfree         禁用无目标优先\n"
        "  --log-level <l>         trace|debug|info|warn|error\n"
        "  --output-dir <dir>      输出目录\n"
        "  --help/-h               显示帮助\n\n"
        "配置文件: 与联合标定相同，支持 ros2.use_ros2_bag、ros2.ros2_bag_file、\n"
        "  ros2.camera_topics / ros2.camera_topic、sensors[].topic、cam_cam.pairs 等。\n"
        "  另支持扁平键: cam0_images_dir / cam1_images_dir（文件模式）。\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    std::string config_file, method_str = "ba";
    std::string output_dir = "./results";
    std::string log_level  = "info";
    std::string data_dir;
    std::string ros2_bag_cli;
    bool do_manual   = false;
    bool no_viz      = false;
    bool prefer_tf   = true;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--manual")                 do_manual = true;
        else if (a == "--no-viz")                 no_viz = true;
        else if (a == "--no-targetfree")          prefer_tf = false;
        else if (a == "--method" && i+1 < argc)   method_str = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--data-dir" && i+1 < argc)   data_dir = argv[++i];
        else if (a == "--ros2-bag" && i+1 < argc)   ros2_bag_cli = argv[++i];
        else if (a == "--log-level" && i+1 < argc) log_level = argv[++i];
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    UNICALIB_MAIN_TRY_BEGIN

    YAML::Node cfg;
    try { cfg = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
        UNICALIB_ERROR("配置文件加载失败: {}", e.what()); return 1;
    }

    if (cfg["output_dir"]) output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["method"]) method_str = cfg["method"].as<std::string>();
    if (cfg["prefer_targetfree"]) prefer_tf = cfg["prefer_targetfree"].as<bool>();
    double manual_thresh = cfg["manual_rms_threshold"] ?
        cfg["manual_rms_threshold"].as<double>() : 1.5;

    std::string logs_dir = resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/cam_cam_" + log_timestamp_filename() + ".log";
    Logger::init("Cam-Cam",
                 log_file,
                 log_level == "debug" ? spdlog::level::debug :
                 log_level == "trace" ? spdlog::level::trace :
                 log_level == "warn"  ? spdlog::level::warn  :
                                        spdlog::level::info);

    SystemConfig sys_cfg;
    try {
        sys_cfg = YamlIO::load_system_config(config_file);
        if (!sys_cfg.output_dir.empty()) output_dir = sys_cfg.output_dir;
    } catch (const UniCalibException& e) {
        UNICALIB_ERROR("系统配置加载失败: {}", e.toString()); return 1;
    }
    if (cfg["output_dir"]) output_dir = cfg["output_dir"].as<std::string>();

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置: {}", config_file);
    UNICALIB_INFO("精标定方法: {} | 无目标优先: {}", method_str, prefer_tf);
    UNICALIB_INFO("手动校准: {} (阈值={:.2f}px)", do_manual, manual_thresh);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    CamCamCalibrator::Config calib_cfg;
    if      (method_str == "ba")        calib_cfg.method = CamCamCalibrator::Method::BUNDLE_ADJUSTMENT;
    else if (method_str == "stereo")    calib_cfg.method = CamCamCalibrator::Method::CHESSBOARD_STEREO;
    else if (method_str == "essential") calib_cfg.method = CamCamCalibrator::Method::ESSENTIAL_MATRIX;
    else {
        UNICALIB_WARN("未知方法 '{}', 使用 ba", method_str);
        calib_cfg.method = CamCamCalibrator::Method::BUNDLE_ADJUSTMENT;
    }

    if (cfg["num_features"])  calib_cfg.num_features = cfg["num_features"].as<int>();
    if (cfg["match_ratio"])   calib_cfg.match_ratio  = cfg["match_ratio"].as<double>();
    if (cfg["ba_max_iter"])   calib_cfg.ba_max_iter  = cfg["ba_max_iter"].as<int>();
    if (cfg["max_rms_px"])    calib_cfg.max_rms_px   = cfg["max_rms_px"].as<double>();
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");

    if (cfg["feature_type"]) {
        auto ft = cfg["feature_type"].as<std::string>();
        if (ft == "sift") calib_cfg.feature_type = CamCamCalibrator::Config::FeatureType::SIFT;
        else              calib_cfg.feature_type = CamCamCalibrator::Config::FeatureType::ORB;
    }

    CamCamCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& step, double prog) {
        if (prog >= 0)
            UNICALIB_INFO("  [进度] {}: {:.1f}%", step, prog * 100.0);
    });

    PipelineConfig pipe_cfg;
    pipe_cfg.tasks             = CalibTaskType::CAM_CAM_EXTRIN;
    pipe_cfg.prefer_targetfree = prefer_tf;
    pipe_cfg.allow_manual_fallback = do_manual;
    if (cfg["allow_manual_fallback"] && cfg["allow_manual_fallback"].as<bool>())
        pipe_cfg.allow_manual_fallback = true;
    pipe_cfg.enable_viz              = !no_viz;
    pipe_cfg.cam_cam_rms_threshold   = manual_thresh;
    if (cfg["cam_cam_rms_threshold"])
        pipe_cfg.cam_cam_rms_threshold = cfg["cam_cam_rms_threshold"].as<double>();
    if (cfg["use_cam_cam_initial_from_params"])
        pipe_cfg.use_cam_cam_initial_from_params = cfg["use_cam_cam_initial_from_params"].as<bool>();
    if (cfg["use_cam_cam_skip_coarse_when_initial"])
        pipe_cfg.use_cam_cam_skip_coarse_when_initial = cfg["use_cam_cam_skip_coarse_when_initial"].as<bool>();
    if (cfg["use_pangolin_manual_panel"])
        pipe_cfg.use_pangolin_manual_panel = cfg["use_pangolin_manual_panel"].as<bool>();
    pipe_cfg.enable_coarse_cam_cam   = false;
    pipe_cfg.output_dir        = output_dir;
    pipe_cfg.log_level         = log_level;

    load_cam_cam_pairs_from_cfg(cfg, pipe_cfg);
    if (cfg["cam_cam"]) {
        const auto& cc = cfg["cam_cam"];
        if (cc["initial_extrinsics"] && cc["initial_extrinsics"].IsMap()) {
            for (const auto& kv : cc["initial_extrinsics"]) {
                const std::string key = kv.first.as<std::string>();
                if (!kv.second || !kv.second.IsMap()) continue;
                std::vector<double> v = parse_4x4_matrix_from_yaml(kv.second);
                if (v.size() >= 16u)
                    pipe_cfg.cam_cam_initial_extrinsic_inline[key] = std::move(v);
                else
                    UNICALIB_WARN("[Cam-Cam] initial_extrinsics['{}'] 无效，已忽略", key);
            }
            if (!pipe_cfg.cam_cam_initial_extrinsic_inline.empty())
                UNICALIB_INFO("[Cam-Cam] cam_cam.initial_extrinsics 已加载 {} 组",
                              pipe_cfg.cam_cam_initial_extrinsic_inline.size());
        }
    }

    std::string base_data_dir = data_dir.empty() ? (std::getenv("CALIB_DATA_DIR") ? std::getenv("CALIB_DATA_DIR") : "") : data_dir;

    const YAML::Node ros2_node = cfg["ros2"];
    if (ros2_node) {
        if (ros2_node["use_ros2_bag"])
            pipe_cfg.use_ros2_bag = ros2_node["use_ros2_bag"].as<bool>();
        if (ros2_node["use_ros2_topics"])
            pipe_cfg.use_ros2_topics = ros2_node["use_ros2_topics"].as<bool>();
        if (ros2_node["ros2_bag_file"])
            pipe_cfg.ros2_bag_file = ros2_node["ros2_bag_file"].as<std::string>();
        if (ros2_node["max_wait_time"])
            pipe_cfg.ros2_max_wait_time = ros2_node["max_wait_time"].as<double>();
        if (ros2_node["sample_interval"])
            pipe_cfg.ros2_sample_interval = ros2_node["sample_interval"].as<double>();
        if (ros2_node["max_frames"])
            pipe_cfg.ros2_max_frames = ros2_node["max_frames"].as<size_t>();
        if (ros2_node["lidar_topic"])
            pipe_cfg.lidar_ros2_topic = ros2_node["lidar_topic"].as<std::string>();
        if (ros2_node["camera_topic"]) {
            const auto& ct = ros2_node["camera_topic"];
            if (ct.IsSequence()) {
                std::vector<std::string> cam_sensor_ids;
                for (const auto& s : sys_cfg.sensors) {
                    if (s.type == SensorType::CAMERA) cam_sensor_ids.push_back(s.sensor_id);
                }
                for (size_t i = 0; i < ct.size(); ++i) {
                    std::string topic = ct[i].as<std::string>();
                    std::string id = (i < cam_sensor_ids.size()) ? cam_sensor_ids[i] : ("cam_" + std::to_string(i));
                    pipe_cfg.camera_topics[id] = topic;
                    if (i == 0) {
                        pipe_cfg.camera_ros2_topic = topic;
                        pipe_cfg.camera_id = id;
                    }
                }
            } else {
                pipe_cfg.camera_ros2_topic = ct.as<std::string>();
            }
        }
        if (ros2_node["camera_topics"] && ros2_node["camera_topics"].IsMap()) {
            for (auto it = ros2_node["camera_topics"].begin(); it != ros2_node["camera_topics"].end(); ++it)
                pipe_cfg.camera_topics[it->first.as<std::string>()] = it->second.as<std::string>();
        }
        if (ros2_node["imu_topic"])
            pipe_cfg.imu_ros2_topic = ros2_node["imu_topic"].as<std::string>();
        if (ros2_node["strict_topic_match"])
            pipe_cfg.ros2_strict_topic_match = ros2_node["strict_topic_match"].as<bool>();
    }

    // NEW_FORMAT：启用时优先于 ROS2 bag
    const YAML::Node new_format_node = cfg["new_format"];
    if (new_format_node && new_format_node["enable"] && new_format_node["enable"].as<bool>()) {
        pipe_cfg.use_new_format = true;
        if (new_format_node["root_dir"]) {
            std::string root_dir = new_format_node["root_dir"].as<std::string>();
            if (!base_data_dir.empty() && !fs::path(root_dir).is_absolute())
                root_dir = resolve_data_path(base_data_dir, root_dir);
            pipe_cfg.new_format_root_dir = root_dir;
        }
        if (new_format_node["timestamp_unit"])
            pipe_cfg.new_format_timestamp_unit = new_format_node["timestamp_unit"].as<std::string>();
        if (new_format_node["oem7_imu_rate_hz"])
            pipe_cfg.new_format_oem7_imu_rate_hz = new_format_node["oem7_imu_rate_hz"].as<double>();
        if (new_format_node["oem7_time_base"])
            pipe_cfg.new_format_oem7_time_base = new_format_node["oem7_time_base"].as<std::string>();
        if (new_format_node["oem7_gps_utc_leap_sec"])
            pipe_cfg.new_format_oem7_gps_utc_leap_sec = new_format_node["oem7_gps_utc_leap_sec"].as<int>();
        if (new_format_node["oem7_time_offset_sec"])
            pipe_cfg.new_format_oem7_time_offset_sec = new_format_node["oem7_time_offset_sec"].as<double>();
        if (new_format_node["lidar_index_files"] && new_format_node["lidar_index_files"].IsMap()) {
            for (const auto& kv : new_format_node["lidar_index_files"])
                pipe_cfg.new_format_lidar_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
        }
        if (new_format_node["camera_index_files"] && new_format_node["camera_index_files"].IsMap()) {
            for (const auto& kv : new_format_node["camera_index_files"])
                pipe_cfg.new_format_camera_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
        }
        if (new_format_node["imu_index_files"] && new_format_node["imu_index_files"].IsMap()) {
            for (const auto& kv : new_format_node["imu_index_files"])
                pipe_cfg.new_format_imu_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
        }
        pipe_cfg.use_ros2_bag = false;
        pipe_cfg.use_ros2_topics = false;
    }

    for (const auto& s : sys_cfg.sensors) {
        if (s.type == SensorType::CAMERA && !s.topic.empty()) {
            if (pipe_cfg.camera_topics.empty()) {
                pipe_cfg.camera_topics[s.sensor_id] = s.topic;
                if (pipe_cfg.camera_ros2_topic.empty()) {
                    pipe_cfg.camera_ros2_topic = s.topic;
                    pipe_cfg.camera_id = s.sensor_id;
                }
            }
        }
    }

    if (pipe_cfg.ros2_bag_file.empty() && cfg["data"] && cfg["data"]["bag_file"])
        pipe_cfg.ros2_bag_file = cfg["data"]["bag_file"].as<std::string>();

    if (!pipe_cfg.use_new_format && !ros2_bag_cli.empty()) {
        pipe_cfg.use_ros2_bag = true;
        pipe_cfg.ros2_bag_file = ros2_bag_cli;
    }

    if (pipe_cfg.use_ros2_bag && !pipe_cfg.ros2_bag_file.empty() && !base_data_dir.empty()) {
        fs::path p(pipe_cfg.ros2_bag_file);
        if (!p.is_absolute())
            pipe_cfg.ros2_bag_file = resolve_data_path(base_data_dir, pipe_cfg.ros2_bag_file);
    }

    std::string results_camera_intrinsic = "camera_intrinsic";
    if (cfg["results"] && cfg["results"]["camera_intrinsic"])
        results_camera_intrinsic = cfg["results"]["camera_intrinsic"].as<std::string>();
    pipe_cfg.results_camera_intrinsic = results_camera_intrinsic;
    pipe_cfg.camera_images_dirs = sys_cfg.camera_images_dirs;
    if (cfg["data"] && cfg["data"]["camera"]) {
        for (const auto& it : cfg["data"]["camera"]) {
            if (!it.second.IsMap()) continue;
            std::string cam_id = it.first.as<std::string>();
            if (it.second["intrinsic_yaml"]) {
                pipe_cfg.camera_intrinsic_files[cam_id] =
                    it.second["intrinsic_yaml"].as<std::string>();
                UNICALIB_INFO("[Cam-Cam] 内参 data.camera.{}.intrinsic_yaml: {}",
                              cam_id, pipe_cfg.camera_intrinsic_files[cam_id]);
            } else {
                std::string default_path = output_dir + "/" + results_camera_intrinsic + "/camera_intrinsic_" + cam_id + ".yaml";
                pipe_cfg.camera_intrinsic_files[cam_id] = default_path;
            }
        }
    }

    resolve_pipeline_data_paths(pipe_cfg, base_data_dir);
    for (const auto& [cam_id, intrin_path] : pipe_cfg.camera_intrinsic_files) {
        UNICALIB_DEBUG("[Cam-Cam] 内参解析后 {} -> {} (exists={})",
                       cam_id, intrin_path, fs::exists(intrin_path) ? "yes" : "no");
    }

    if (pipe_cfg.use_new_format) {
        UNICALIB_INFO("[Cam-Cam] 数据源: NEW_FORMAT root={} unit={}",
                      pipe_cfg.new_format_root_dir.empty() ? "(未设置)" : pipe_cfg.new_format_root_dir,
                      pipe_cfg.new_format_timestamp_unit);
    } else if (pipe_cfg.use_ros2_bag && !pipe_cfg.ros2_bag_file.empty()) {
        UNICALIB_INFO("[Cam-Cam] 数据源: ROS2 bag  path={}", pipe_cfg.ros2_bag_file);
        if (!pipe_cfg.camera_topics.empty()) {
            for (const auto& [id, top] : pipe_cfg.camera_topics)
                UNICALIB_INFO("[Cam-Cam]   相机 {} -> {}", id, top);
        }
    } else {
        UNICALIB_INFO("[Cam-Cam] 数据源: 非 ROS2 bag（未同时满足 use_ros2_bag 与有效 bag 路径）");
    }

    CalibPipeline pipeline(pipe_cfg);

    UNICALIB_INFO("▶ Stage 1 (粗标定): 特征点匹配 + 本质矩阵");
    if (pipe_cfg.use_new_format)
        UNICALIB_INFO("  图像: 从 NEW_FORMAT 索引加载");
    else if (pipe_cfg.use_ros2_bag && !pipe_cfg.ros2_bag_file.empty())
        UNICALIB_INFO("  图像: 从 ROS2 bag 按 camera_topics / sensors 加载");
    else {
        UNICALIB_INFO("  相机0 图像: {}",
                      cfg["cam0_images_dir"] ? cfg["cam0_images_dir"].as<std::string>() : "(未设置)");
        UNICALIB_INFO("  相机1 图像: {}",
                      cfg["cam1_images_dir"] ? cfg["cam1_images_dir"].as<std::string>() : "(未设置)");
    }

    UNICALIB_INFO("▶ Stage 2 (精标定): {} (无目标={})", method_str, prefer_tf);

    if (do_manual) {
        UNICALIB_INFO("▶ Stage 3 (手动校准): click-calib 双目对应点精化");
        UNICALIB_INFO("  阈值: {:.2f}px 超过则提示手动校准", manual_thresh);
        UNICALIB_INFO("  使用 ManualClickRefiner::refine_cam_cam() 进行精化");
    }

    auto report = pipeline.run();
    report.print_summary();

    UNICALIB_INFO("Cam-Cam 标定完成 | 结果: {}", output_dir);
    UNICALIB_MAIN_TRY_END(0)
}
