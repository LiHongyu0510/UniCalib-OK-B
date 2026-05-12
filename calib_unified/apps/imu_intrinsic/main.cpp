/**
 * UniCalib — IMU 内参标定应用
 * 使用: unicalib_imu_intrinsic --config <config.yaml>
 *       unicalib_imu_intrinsic --data_dir <dir> --sensor_id <id>
 *
 * 输出:
 *   - 标定结果 YAML/JSON
 *   - Allan 偏差图 PNG
 *   - HTML 报告
 */

#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/common/calib_param.h"
#include "unicalib/common/accuracy_logger.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/viz/report_gen.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/io/ros2_data_source.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <iomanip>
#include <cstdlib>

namespace fs = std::filesystem;

// 解析数据路径：相对路径与 base 拼接；绝对路径原样返回
static std::string resolve_data_path(const std::string& base, const std::string& path) {
    if (path.empty()) return "";
    if (base.empty()) return path;
    fs::path p(path);
    if (p.is_absolute()) return path;
    return (fs::path(base) / p).lexically_normal().string();
}

// 从 YAML node 读取 4x4 矩阵 (OpenCV 风格: rows, cols, dt, data[])
static Eigen::Matrix4d read_matrix_4x4_from_yaml(const YAML::Node& node) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    if (!node) return mat;
    
    // 检查是否为 OpenCV 风格格式 (rows, cols, dt, data)
    if (node["rows"] && node["cols"] && node["data"]) {
        int rows = node["rows"].as<int>();
        int cols = node["cols"].as<int>();
        if (rows != 4 || cols != 4) {
            UNICALIB_WARN("T_Body_Imu 矩阵维度应为 4x4，实际 {}x{}，将使用单位矩阵", rows, cols);
            return mat;
        }
        const auto& data = node["data"];
        if (data.IsSequence() && data.size() >= 16) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    mat(i, j) = data[i * 4 + j].as<double>();
                }
            }
        }
    } else if (node.IsSequence() && node.size() >= 16) {
        // 直接为 16 元素序列
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                mat(i, j) = node[i * 4 + j].as<double>();
            }
        }
    }
    return mat;
}

// ===================================================================
// 配置解析
// ===================================================================
struct AppConfig {
    std::string sensor_id = "imu_0";
    std::string data_file;          // CSV 文件: timestamp,gx,gy,gz,ax,ay,az
    std::string data_dir;           // 或目录(含 imu.csv)；ROS2 bag 时用于解析 ros2_bag_file 相对路径
    std::string output_dir = "./results";
    std::string results_imu_intrinsic;  // results.imu_intrinsic 子目录名；非空时结果写入 output_dir/<该子目录>/imu_intrinsic_<id>.yaml
    ns_unicalib::IMUIntrinsicCalibrator::Config calib_cfg;
    // ROS2 bag 模式（与 unicalib_example.yaml 中 ros2 段一致）
    bool use_ros2_bag = false;
    bool use_new_format = false;
    std::string new_format_root_dir;
    std::string new_format_timestamp_unit = "s";
    double new_format_oem7_imu_rate_hz = 200.0;
    std::string new_format_oem7_time_base = "gps";
    int new_format_oem7_gps_utc_leap_sec = 18;
    double new_format_oem7_time_offset_sec = 0.0;
    std::map<std::string, std::string> new_format_lidar_index_files;
    std::map<std::string, std::string> new_format_camera_index_files;
    std::map<std::string, std::string> new_format_imu_index_files;
    std::string ros2_bag_file;
    std::string imu_ros2_topic;     // 来自 ros2.imu_topic 或 sensors 中第一个 IMU 的 topic
    bool ros2_strict_topic_match = true;  // 配置话题在 bag 中无数据则失败，不自动回退
    // 六面法静态段不足(<6)时，使用 Transformer-IMU-Calibrator 作为备选（参考最新 IMU 标定研究）
    std::string transformer_imu_path;  // third_party.transformer_imu 或环境变量 UNICALIB_TRANSFORMER_IMU
    std::string transformer_imu_model_weights;  // 可选，默认 model/TIC_13.pth
    bool use_transformer_fallback_when_insufficient_static = true;
    bool transformer_imu_mean_only = false;      // 单 IMU 时传 --mean-only，避免 6× 复制降低精度
    // IMU 与车体外参 T_Body_Imu（默认单位矩阵，IMU 与车体系重合）
    Eigen::Matrix4d T_Body_Imu = Eigen::Matrix4d::Identity();
};

AppConfig parse_config(int argc, char** argv) {
    AppConfig cfg;

    // 简单参数解析
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            // 从 YAML 文件加载（支持统一配置 unicalib_example.yaml 或独立 IMU 配置）
            YAML::Node node = YAML::LoadFile(argv[++i]);
            // 统一配置: data + sensors，取第一个 IMU
            if (node["data"] && node["sensors"]) {
                for (const auto& s : node["sensors"]) {
                    if (s["type"] && s["type"].as<std::string>() == "imu") {
                        cfg.sensor_id = s["id"].as<std::string>();
                        if (node["data"]["imu"] && node["data"]["imu"][cfg.sensor_id]) {
                            cfg.data_file = node["data"]["imu"][cfg.sensor_id].as<std::string>();
                        }
                        if (cfg.imu_ros2_topic.empty() && s["topic"])
                            cfg.imu_ros2_topic = s["topic"].as<std::string>();
                        break;
                    }
                }
                const YAML::Node ros2_node = node["ros2"];
                if (ros2_node) {
                    if (ros2_node["use_ros2_bag"]) cfg.use_ros2_bag = ros2_node["use_ros2_bag"].as<bool>();
                    if (ros2_node["ros2_bag_file"]) cfg.ros2_bag_file = ros2_node["ros2_bag_file"].as<std::string>();
                    if (ros2_node["imu_topic"]) cfg.imu_ros2_topic = ros2_node["imu_topic"].as<std::string>();
                    if (ros2_node["strict_topic_match"]) cfg.ros2_strict_topic_match = ros2_node["strict_topic_match"].as<bool>();
                }
                const YAML::Node new_format_node = node["new_format"];
                if (new_format_node && new_format_node["enable"] && new_format_node["enable"].as<bool>()) {
                    cfg.use_new_format = true;
                    if (new_format_node["root_dir"]) cfg.new_format_root_dir = new_format_node["root_dir"].as<std::string>();
                    if (new_format_node["timestamp_unit"]) cfg.new_format_timestamp_unit = new_format_node["timestamp_unit"].as<std::string>();
                    if (new_format_node["oem7_imu_rate_hz"])
                        cfg.new_format_oem7_imu_rate_hz = new_format_node["oem7_imu_rate_hz"].as<double>();
                    if (new_format_node["oem7_time_base"])
                        cfg.new_format_oem7_time_base = new_format_node["oem7_time_base"].as<std::string>();
                    if (new_format_node["oem7_gps_utc_leap_sec"])
                        cfg.new_format_oem7_gps_utc_leap_sec = new_format_node["oem7_gps_utc_leap_sec"].as<int>();
                    if (new_format_node["oem7_time_offset_sec"])
                        cfg.new_format_oem7_time_offset_sec = new_format_node["oem7_time_offset_sec"].as<double>();
                    if (new_format_node["lidar_index_files"] && new_format_node["lidar_index_files"].IsMap()) {
                        for (const auto& kv : new_format_node["lidar_index_files"])
                            cfg.new_format_lidar_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
                    }
                    if (new_format_node["camera_index_files"] && new_format_node["camera_index_files"].IsMap()) {
                        for (const auto& kv : new_format_node["camera_index_files"])
                            cfg.new_format_camera_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
                    }
                    if (new_format_node["imu_index_files"] && new_format_node["imu_index_files"].IsMap()) {
                        for (const auto& kv : new_format_node["imu_index_files"])
                            cfg.new_format_imu_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
                    }
                    cfg.use_ros2_bag = false;
                    cfg.data_file.clear();
                }
                if (cfg.use_ros2_bag) {
                    cfg.data_file.clear();  // 使用 bag 时不用 data.imu 的 CSV 路径
                }
                if (cfg.use_ros2_bag && cfg.imu_ros2_topic.empty()) {
                    for (const auto& s : node["sensors"]) {
                        if (s["type"] && s["type"].as<std::string>() == "imu" && s["topic"]) {
                            cfg.imu_ros2_topic = s["topic"].as<std::string>();
                            break;
                        }
                    }
                }
                if (node["output_dir"]) cfg.output_dir = node["output_dir"].as<std::string>();
                if (node["results"] && node["results"]["imu_intrinsic"])
                    cfg.results_imu_intrinsic = node["results"]["imu_intrinsic"].as<std::string>();
                if (node["imu_intrinsic"]) {
                    const auto& ii = node["imu_intrinsic"];
                    if (ii["static_gyro_thresh"])   cfg.calib_cfg.static_gyro_threshold = ii["static_gyro_thresh"].as<double>();
                    if (ii["static_detect_window"]) cfg.calib_cfg.static_detect_window    = ii["static_detect_window"].as<double>();
                    if (ii["min_static_frames"])   cfg.calib_cfg.min_static_frames      = ii["min_static_frames"].as<int>();
                    if (ii["use_transformer_fallback_when_insufficient_static"])
                        cfg.use_transformer_fallback_when_insufficient_static = ii["use_transformer_fallback_when_insufficient_static"].as<bool>(true);
                    if (ii["transformer_imu_mean_only"])
                        cfg.transformer_imu_mean_only = ii["transformer_imu_mean_only"].as<bool>(false);
                    // 读取 IMU 与车体外参 T_Body_Imu
                    if (ii["T_Body_Imu"]) {
                        cfg.T_Body_Imu = read_matrix_4x4_from_yaml(ii["T_Body_Imu"]);
                    }
                }
                if (node["third_party"]) {
                    const auto& tp = node["third_party"];
                    if (tp["transformer_imu"]) cfg.transformer_imu_path = tp["transformer_imu"].as<std::string>("");
                    if (tp["transformer_imu_model_weights"]) cfg.transformer_imu_model_weights = tp["transformer_imu_model_weights"].as<std::string>("");
                }
            } else {
                if (node["sensor_id"])  cfg.sensor_id  = node["sensor_id"].as<std::string>();
                if (node["data_file"])  cfg.data_file  = node["data_file"].as<std::string>();
                if (node["data_dir"])   cfg.data_dir   = node["data_dir"].as<std::string>();
                if (node["output_dir"]) cfg.output_dir = node["output_dir"].as<std::string>();
                if (node["calib"]) {
                    auto cc = node["calib"];
                    if (cc["static_gyro_thresh"]) cfg.calib_cfg.static_gyro_threshold = cc["static_gyro_thresh"].as<double>();
                    if (cc["static_detect_window"]) cfg.calib_cfg.static_detect_window = cc["static_detect_window"].as<double>();
                    if (cc["min_static_frames"])  cfg.calib_cfg.min_static_frames = cc["min_static_frames"].as<int>();
                }
            }
        } else if (arg == "--sensor_id" && i + 1 < argc) {
            cfg.sensor_id = argv[++i];
        } else if (arg == "--data_file" && i + 1 < argc) {
            cfg.data_file = argv[++i];
        } else if ((arg == "--data_dir" || arg == "--data-dir") && i + 1 < argc) {
            cfg.data_dir = argv[++i];
        } else if (arg == "--output_dir" && i + 1 < argc) {
            cfg.output_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: unicalib_imu_intrinsic [options]\n"
                      << "  --config <yaml>      配置文件 (支持统一配置中 ros2.use_ros2_bag)\n"
                      << "  --sensor_id <id>     传感器ID (default: imu_0)\n"
                      << "  --data_file <csv>    IMU数据CSV文件\n"
                      << "  --data_dir <dir>     数据目录(含 imu.csv 或 ROS2 bag 相对路径根)\n"
                      << "  --data-dir <dir>     同上 (与脚本 --data-dir 一致)\n"
                      << "  --output_dir <dir>   输出目录\n"
                      << "\nCSV 格式: timestamp[s],gx,gy,gz[rad/s],ax,ay,az[m/s^2]\n"
                      << "ROS2 bag: 在 config 中设置 ros2.use_ros2_bag: true 与 ros2.ros2_bag_file，并传入 --data-dir。\n"
                      << "六面法静态段不足时，可配置 third_party.transformer_imu 使用 Transformer-IMU-Calibrator 备选。\n";
            exit(0);
        }
    }

    if (cfg.data_file.empty() && !cfg.data_dir.empty()) {
        cfg.data_file = cfg.data_dir + "/imu.csv";
    }

    // 环境变量备选：Transformer-IMU-Calibrator 路径
    if (cfg.transformer_imu_path.empty()) {
        const char* env_ti = std::getenv("UNICALIB_TRANSFORMER_IMU");
        if (env_ti && env_ti[0]) cfg.transformer_imu_path = env_ti;
    }

    return cfg;
}

// ===================================================================
// 加载 IMU CSV 数据
// ===================================================================
ns_unicalib::IMURawData load_imu_csv(const std::string& csv_path) {
    ns_unicalib::IMURawData data;

    std::ifstream f(csv_path);
    if (!f.is_open()) {
        UNICALIB_ERROR("Cannot open IMU data file: {}", csv_path);
        return data;
    }

    std::string line;
    // 跳过注释和表头
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.find("timestamp") != std::string::npos ||
            line.find("time") != std::string::npos) continue;

        std::istringstream ss(line);
        double ts, gx, gy, gz, ax, ay, az;
        char comma;
        if (ss >> ts >> comma >> gx >> comma >> gy >> comma >> gz
               >> comma >> ax >> comma >> ay >> comma >> az) {
            ns_unicalib::IMURawFrame frame;
            frame.timestamp = ts;
            frame.gyro = {gx, gy, gz};
            frame.accel = {ax, ay, az};
            data.push_back(frame);
        }
    }

    UNICALIB_INFO("Loaded {} IMU frames from {}", data.size(), csv_path);
    return data;
}

// ===================================================================
// 主函数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << R"(
 ╔══════════════════════════════════════════════╗
 ║    UniCalib — IMU 内参标定 (Allan + 六面法)  ║
 ╚══════════════════════════════════════════════╝
)" << std::endl;

    UNICALIB_MAIN_TRY_BEGIN

    // 解析配置
    auto cfg = parse_config(argc, argv);

    if (cfg.data_dir.empty()) {
        const char* env_data = std::getenv("CALIB_DATA_DIR");
        if (env_data && env_data[0]) cfg.data_dir = env_data;
    }

    if (!cfg.use_new_format && !cfg.use_ros2_bag && cfg.data_file.empty()) {
        ns_unicalib::Logger::init("UniCalib-IMU-Intrinsic");
        UNICALIB_ERROR("No data source: set new_format.enable, or ros2.use_ros2_bag + ros2.ros2_bag_file, or data.imu.<id> / --data_file for CSV");
        return 1;
    }

    // 创建输出目录，运行日志写入 output_dir/logs
    fs::create_directories(cfg.output_dir);
    std::string logs_dir = ns_unicalib::resolve_logs_dir(cfg.output_dir);
    std::string log_file = logs_dir + "/imu_intrinsic_" + ns_unicalib::log_timestamp_filename() + ".log";
    ns_unicalib::Logger::init("UniCalib-IMU-Intrinsic", log_file, spdlog::level::info);
    UNICALIB_INFO("日志文件: {}", log_file);

    ns_unicalib::IMURawData imu_data;
    if (cfg.use_new_format) {
        ns_unicalib::UnifiedDataLoader::Config load_cfg;
        load_cfg.source_type = ns_unicalib::UnifiedDataLoader::SourceType::NEW_FORMAT;
        load_cfg.new_format_root_dir = cfg.new_format_root_dir;
        load_cfg.new_format_lidar_index_files = cfg.new_format_lidar_index_files;
        load_cfg.new_format_camera_index_files = cfg.new_format_camera_index_files;
        load_cfg.new_format_imu_index_files = cfg.new_format_imu_index_files;
        load_cfg.new_format_timestamp_unit = cfg.new_format_timestamp_unit;
        load_cfg.new_format_oem7_imu_rate_hz = cfg.new_format_oem7_imu_rate_hz;
        load_cfg.new_format_oem7_time_base = cfg.new_format_oem7_time_base;
        load_cfg.new_format_oem7_gps_utc_leap_sec = cfg.new_format_oem7_gps_utc_leap_sec;
        load_cfg.new_format_oem7_time_offset_sec = cfg.new_format_oem7_time_offset_sec;
        load_cfg.max_frames = 0;
        ns_unicalib::UnifiedDataLoader loader(load_cfg);
        if (!loader.load()) {
            UNICALIB_ERROR("从 NEW_FORMAT 加载失败: {}", loader.get_status_message());
            return 1;
        }
        imu_data = loader.to_imu_raw_data(cfg.sensor_id);
        UNICALIB_INFO("Loading IMU data from NEW_FORMAT: root={} sensor_id={} frames={}",
                      cfg.new_format_root_dir, cfg.sensor_id, imu_data.size());
    } else if (cfg.use_ros2_bag) {
#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
        std::string bag_path = cfg.ros2_bag_file;
        if (!bag_path.empty() && !cfg.data_dir.empty()) {
            fs::path p(bag_path);
            if (!p.is_absolute())
                bag_path = resolve_data_path(cfg.data_dir, cfg.ros2_bag_file);
        }
        if (bag_path.empty()) {
            UNICALIB_ERROR("ROS2 bag 模式需要配置 ros2.ros2_bag_file 并传入 --data-dir 或 CALIB_DATA_DIR");
            return 1;
        }
        if (!fs::exists(bag_path)) {
            UNICALIB_ERROR("ROS2 bag 路径不存在: {}", bag_path);
            return 1;
        }
        std::string imu_topic = cfg.imu_ros2_topic.empty() ? "/imu/data" : cfg.imu_ros2_topic;
        ns_unicalib::RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = bag_path;
        ros_cfg.imu_topics[cfg.sensor_id] = imu_topic;
        ros_cfg.imu_ros2_topic = imu_topic;
        ros_cfg.strict_topic_match = cfg.ros2_strict_topic_match;
        auto ros_source = ns_unicalib::create_ros_data_source(ros_cfg);
        if (!ros_source || !ros_source->load()) {
            UNICALIB_ERROR("从 ROS2 bag 加载失败: {}", ros_source ? ros_source->get_status_message() : "create failed");
            return 1;
        }
        auto ros_frames = ros_source->get_imu_frames(cfg.sensor_id);
        for (const auto& fr : ros_frames) {
            ns_unicalib::IMURawFrame raw;
            raw.timestamp = fr.timestamp;
            raw.gyro  = Eigen::Vector3d(fr.gyro[0], fr.gyro[1], fr.gyro[2]);
            raw.accel = Eigen::Vector3d(fr.accel[0], fr.accel[1], fr.accel[2]);
            imu_data.push_back(raw);
        }
        UNICALIB_INFO("Loading IMU data from ROS2 bag: {} (topic: {}, {} frames)", bag_path, imu_topic, imu_data.size());
#else
        UNICALIB_ERROR("ROS2 bag 需要编译时启用 ROS2 (UNICALIB_WITH_ROS2)。请使用 CSV 或使用带 ROS2 的构建。");
        return 1;
#endif
    } else {
        UNICALIB_INFO("Loading IMU data from: {}", cfg.data_file);
        imu_data = load_imu_csv(cfg.data_file);
    }

    if (imu_data.size() < 1000) {
        UNICALIB_ERROR("Insufficient IMU data: {} frames (need ≥ 1000)", imu_data.size());
        return 1;
    }

    double duration = imu_data.back().timestamp - imu_data.front().timestamp;
    UNICALIB_INFO("Duration: {:.1f} s | Frames: {}", duration, imu_data.size());

    // 标定
    ns_unicalib::IMUIntrinsicCalibrator calibrator(cfg.calib_cfg);

    calibrator.set_progress_callback([](const std::string& stage, double prog) {
        static std::string last_stage;
        if (stage != last_stage) {
            std::cout << "\r  [" << stage << "] " << std::flush;
            last_stage = stage;
        }
        int bar_width = 30;
        int filled = static_cast<int>(prog * bar_width);
        std::cout << "\r  [" << stage << "] [";
        for (int i = 0; i < bar_width; ++i)
            std::cout << (i < filled ? "█" : "░");
        std::cout << "] " << std::fixed << std::setprecision(0)
                  << prog * 100 << "%" << std::flush;
    });

    auto t_start = std::chrono::high_resolution_clock::now();
    ns_unicalib::IMUIntrinsics intrin = calibrator.calibrate(imu_data);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << std::endl;

    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    UNICALIB_INFO("Calibration took {:.1f} s", elapsed);

    // 六面法静态段不足(<6)时：优先 Python 脚本（若配置），否则自动用 C++ 原生零偏估计（无 Python/无模型）
    auto segments = calibrator.detect_static_segments(imu_data);
    if (segments.size() < 6 && cfg.use_transformer_fallback_when_insufficient_static) {
        std::vector<ns_unicalib::IMUFrame> imu_frames;
        imu_frames.reserve(imu_data.size());
        for (const auto& fr : imu_data) {
            ns_unicalib::IMUFrame f;
            f.timestamp = fr.timestamp;
            f.gyro  = fr.gyro;
            f.accel = fr.accel;
            imu_frames.push_back(f);
        }
        if (cfg.transformer_imu_path.empty()) {
            UNICALIB_INFO("[UniCalib-IMU-Intrinsic] Transformer-IMU 路径未配置 (third_party.transformer_imu / UNICALIB_TRANSFORMER_IMU)，将使用 C++ 原生零偏估计");
        } else {
            UNICALIB_INFO("[UniCalib-IMU-Intrinsic] Transformer-IMU 路径: {}", cfg.transformer_imu_path);
        }
        ns_unicalib::TransformerIMUAdapter::Config ti_cfg;
        ti_cfg.repo_dir = cfg.transformer_imu_path;
        ti_cfg.work_dir = cfg.output_dir + "/transformer_imu_work";
        if (!cfg.transformer_imu_model_weights.empty())
            ti_cfg.model_weights = cfg.transformer_imu_model_weights;
        ti_cfg.use_mean_only = cfg.transformer_imu_mean_only;
        ns_unicalib::TransformerIMUAdapter adapter(ti_cfg);
        auto ti_result = adapter.estimate(imu_frames, cfg.output_dir + "/transformer_imu_out");
        if (ti_result.success) {
            intrin.bias_gyro = ti_result.coarse_intrin.bias_gyro;
            intrin.bias_acce = ti_result.coarse_intrin.bias_acce;
            double ba_norm = intrin.bias_acce.norm();
            UNICALIB_INFO("六面法静态段不足 ({} < 6)，零偏来源: {} (备选路径)", segments.size(), ti_result.model_name);
            UNICALIB_INFO("  bias_gyro=[{:.6f}, {:.6f}, {:.6f}] (陀螺零偏为均值/模型粗估，可作初值)", intrin.bias_gyro.x(), intrin.bias_gyro.y(), intrin.bias_gyro.z());
            UNICALIB_INFO("  bias_acce=[{:.6f}, {:.6f}, {:.6f}] ‖ba‖={:.4f} m/s²", intrin.bias_acce.x(), intrin.bias_acce.y(), intrin.bias_acce.z(), ba_norm);
            if (ba_norm > 15.0) {
                UNICALIB_WARN("加速度计零偏 ‖ba‖={:.2f} 远大于 g≈9.81，多为重力方向/坐标系假设不一致，下游慎用；建议启用 transformer_imu_mean_only + 脚本 --gravity-auto 或补采六面法数据", ba_norm);
            }
        } else {
            UNICALIB_WARN("零偏备选未成功: {}", ti_result.error_msg.empty() ? "unknown" : ti_result.error_msg);
        }
    }

    // 保存 Allan 偏差图
    std::string allan_plot_path = cfg.output_dir + "/allan_gyro.png";
    auto allan_result = calibrator.analyze_allan(imu_data);
    calibrator.save_allan_plot(allan_result, allan_plot_path);

    // 保存标定结果（使用 Emitter 保证 T_Body_Imu.data 为带中括号的 4x4 矩阵形式）
    std::string result_subdir = cfg.output_dir;
    if (!cfg.results_imu_intrinsic.empty()) {
        result_subdir = cfg.output_dir + "/" + cfg.results_imu_intrinsic;
        fs::create_directories(result_subdir);
    }
    std::string yaml_path = result_subdir + "/imu_intrinsic_" + cfg.sensor_id + ".yaml";
    {
        std::ofstream yaml_out(yaml_path);
        if (!yaml_out.is_open()) {
            UNICALIB_ERROR("无法写入结果文件: {}", yaml_path);
        } else {
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "sensor_id" << YAML::Value << cfg.sensor_id;
            out << YAML::Key << "timestamp" << YAML::Value
                << std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
            out << YAML::Key << "noise_gyro" << YAML::Value << intrin.noise_gyro;
            out << YAML::Key << "bias_instab_gyro" << YAML::Value << intrin.bias_instab_gyro;
            out << YAML::Key << "noise_accel" << YAML::Value << intrin.noise_acce;
            out << YAML::Key << "bias_instab_accel" << YAML::Value << intrin.bias_instab_acce;
            out << YAML::Key << "bias_gyro" << YAML::Value << YAML::Flow
                << std::vector<double>{intrin.bias_gyro[0], intrin.bias_gyro[1], intrin.bias_gyro[2]};
            out << YAML::Key << "bias_accel" << YAML::Value << YAML::Flow
                << std::vector<double>{intrin.bias_acce[0], intrin.bias_acce[1], intrin.bias_acce[2]};
            // T_Body_Imu: 4x4 矩阵，data 以带中括号的 flow 序列写出（与 imu_v100.yaml 格式一致）
            out << YAML::Key << "T_Body_Imu" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "rows" << YAML::Value << 4;
            out << YAML::Key << "cols" << YAML::Value << 4;
            out << YAML::Key << "dt" << YAML::Value << "d";
            out << YAML::Key << "data" << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    out << cfg.T_Body_Imu(i, j);
                }
            }
            out << YAML::EndSeq << YAML::EndMap;
            out << YAML::EndMap;
            std::string content = out.c_str();
            // 在 T_Body_Imu 前插入中文注释
            {
                const std::string anchor = "\nT_Body_Imu:";
                std::string::size_type insert_pos = content.find(anchor);
                if (insert_pos != std::string::npos) {
                    const char* comment =
                        "# T_Body_Imu: 车体系到IMU系的4x4刚体变换矩阵 (Body -> IMU)\n"
                        "# 含义: 描述IMU在车体坐标系下的位姿；将车体坐标系下的点 p_body 变换到IMU系: p_imu = T_Body_Imu * p_body\n"
                        "# 默认单位矩阵表示IMU与车体坐标系重合（原点与轴向一致）\n"
                        "# 使用: 下游SLAM/INS中需将IMU角速度、加速度转到车体时，用旋转部分 R(3x3)：omega_body = R^T * omega_imu，a_body = R^T * a_imu\n"
                        "# 若从配置/标定得到的是 T_Imu_Body（IMU->Body），则 T_Body_Imu = inv(T_Imu_Body)\n";
                    content.insert(insert_pos + 1, comment);
                }
            }
            // 将 data 从单行 flow 改为 4x4 矩阵形式（与 imu_v100.yaml 一致）
            const std::string data_key = "data: [";
            std::string::size_type pos = content.find(data_key);
            if (pos != std::string::npos) {
                std::string::size_type end = content.find("]", pos + data_key.size());
                if (end != std::string::npos) {
                    std::ostringstream data_ss;
                    data_ss << "[ ";
                    for (int i = 0; i < 4; ++i) {
                        for (int j = 0; j < 4; ++j) {
                            if (i * 4 + j > 0 && j != 0) data_ss << ", ";
                            data_ss << std::fixed << std::setprecision(1)
                                    << cfg.T_Body_Imu(i, j);
                        }
                        if (i < 3) data_ss << ",\n          ";
                    }
                    data_ss << " ]";
                    content.replace(pos + 6, end - pos - 6 + 1, data_ss.str());
                }
            }
            yaml_out << content << "\n";
            UNICALIB_INFO("Results saved to: {}", yaml_path);
        }
    }

    // 精度记录到 CSV（IMU 无单一 residual_rms，记录噪声与零偏不稳）
    double elapsed_ms = elapsed * 1000.0;
    std::ostringstream ng, bg, na, ba;
    ng << std::scientific << intrin.noise_gyro;
    bg << std::scientific << intrin.bias_instab_gyro;
    na << std::scientific << intrin.noise_acce;
    ba << std::scientific << intrin.bias_instab_acce;
    std::map<std::string, std::string> extra;
    extra["noise_gyro"] = ng.str();
    extra["bias_instab_gyro"] = bg.str();
    extra["noise_accel"] = na.str();
    extra["bias_instab_accel"] = ba.str();
    ns_unicalib::append_calib_accuracy(cfg.output_dir, ns_unicalib::CalibAccuracyTask::IMU_INTRINSIC,
        true, 0.0, elapsed_ms, extra);

    // 打印终端摘要
    std::cout << "\n";
    std::cout << "┌────────────────────────────────────────────────────┐\n";
    std::cout << "│              IMU 内参标定结果                      │\n";
    std::cout << "├──────────────────────┬─────────────────────────────┤\n";
    std::cout << "│ 传感器 ID            │ " << std::setw(27) << cfg.sensor_id << " │\n";
    std::cout << "│ 数据帧数             │ " << std::setw(27) << imu_data.size() << " │\n";
    std::cout << "│ 数据时长 (s)         │ " << std::setw(27) << std::fixed
              << std::setprecision(1) << duration << " │\n";
    std::cout << "├──────────────────────┼─────────────────────────────┤\n";
    std::cout << "│ 陀螺噪声 (rad/s/√Hz) │ " << std::setw(27) << std::scientific
              << std::setprecision(4) << intrin.noise_gyro << " │\n";
    std::cout << "│ 陀螺零偏不稳 (rad/s) │ " << std::setw(27) << intrin.bias_instab_gyro << " │\n";
    std::cout << "│ 加速计噪声 (m/s²/√Hz)│ " << std::setw(27) << intrin.noise_acce << " │\n";
    std::cout << "│ 加速计零偏不稳 (m/s²)│ " << std::setw(27) << intrin.bias_instab_acce << " │\n";
    double ba_norm = intrin.bias_acce.norm();
    std::cout << "│ 加速计零偏 ‖ba‖(m/s²)│ " << std::setw(27) << std::scientific << std::setprecision(4) << ba_norm << " │\n";
    std::cout << "└──────────────────────┴─────────────────────────────┘\n";
    if (ba_norm > 15.0) {
        std::cout << "⚠ ‖ba‖ 过大，请检查零偏来源与重力假设（见上方日志）\n";
    }
    std::cout << "\n结果保存至: " << cfg.output_dir << "\n\n";

    UNICALIB_MAIN_TRY_END(0)
}
