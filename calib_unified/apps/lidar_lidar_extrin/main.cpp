/**
 * UniCalib — LiDAR-LiDAR 外参标定应用
 *
 * 两阶段: 粗标定 (NDT/多帧合并) → 精标定 (GICP/NDT)
 * 用法:
 *   unicalib_lidar_lidar --config <config.yaml>
 *   unicalib_lidar_lidar --config <config.yaml> --data-dir /path/to/data
 */
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/extrinsic/lidar_scan_pair.h"
#include "unicalib/extrinsic/lidar_lidar_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/viz/calib_visualizer.h"
#include "unicalib/io/yaml_io.h"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <map>
#include <memory>
#include <optional>

#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
#include "unicalib/io/ros2_data_source.h"
#endif

namespace fs = std::filesystem;
using namespace ns_unicalib;

#define YG(node, key, default_val) \
    ((node)[key] ? (node)[key].as<std::decay_t<decltype(default_val)>>() : (default_val))

// 将 4x4 YAML（行优先）解析为 SE3，供 lidar_lidar.initial_extrinsic 使用
static Eigen::Matrix3d project_to_so3(const Eigen::Matrix3d& M) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) R.col(2) *= -1;
    return R;
}

static std::optional<Sophus::SE3d> parse_se3_from_yaml_4x4(const YAML::Node& node) {
    if (!node || !node["rows"] || !node["cols"] || !node["data"]) return std::nullopt;
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    if (rows != 4 || cols != 4 || !node["data"].IsSequence() || node["data"].size() < 16)
        return std::nullopt;
    const auto& data = node["data"];
    Eigen::Matrix4d T;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T(i, j) = data[i * 4 + j].as<double>();
    Eigen::Matrix3d R = project_to_so3(T.block<3, 3>(0, 0));
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    return Sophus::SE3d(Sophus::SO3d(R), t);
}

static std::string resolve_data_path(const std::string& base, const std::string& path) {
    if (path.empty()) return "";
    std::string work = path;
    const char prefix[] = "/path/to/";
    if (!base.empty() && work.size() > sizeof(prefix) - 1 &&
        work.compare(0, sizeof(prefix) - 1, prefix) == 0)
        work = work.substr(sizeof(prefix) - 1);
    if (base.empty()) return path;
    fs::path p(work);
    if (p.is_absolute()) return path;
    return (fs::path(base) / p).lexically_normal().string();
}

// 配置里常写 Docker/另一台机器上的绝对路径；本机不存在时，若提供了 --data-dir，则尝试把
// 「.../data/<相对后缀>」映射到「<data-dir>/<相对后缀>」。
static std::optional<std::string> data_suffix_after_data_component(const std::string& abs_path) {
    const std::string needle = "/data/";
    size_t pos = abs_path.find(needle);
    if (pos == std::string::npos) return std::nullopt;
    std::string tail = abs_path.substr(pos + needle.size());
    if (tail.empty()) return std::nullopt;
    return tail;
}

static std::string resolve_lidar_dir_for_load(const std::string& data_dir, const std::string& yaml_path) {
    UNICALIB_INFO("[LiDAR-LiDAR][Path] 解析 data.lidar 路径: yaml_path='{}' data_dir='{}'",
                  yaml_path, data_dir);
    std::string primary = resolve_data_path(data_dir, yaml_path);
    UNICALIB_INFO("[LiDAR-LiDAR][Path] primary='{}' exists={} is_dir={}",
                  primary, fs::exists(primary), fs::is_directory(primary));
    if (fs::exists(primary) && fs::is_directory(primary)) {
        UNICALIB_INFO("[LiDAR-LiDAR][Path] 使用 primary 路径: {}", primary);
        return primary;
    }
    if (!data_dir.empty() && fs::path(yaml_path).is_absolute()) {
        if (auto tail = data_suffix_after_data_component(yaml_path)) {
            fs::path alt = fs::path(data_dir) / *tail;
            std::string alt_s = alt.lexically_normal().string();
            UNICALIB_INFO("[LiDAR-LiDAR][Path] 尝试 /data/ 后缀映射: tail='{}' alt='{}' exists={} is_dir={}",
                          *tail, alt_s, fs::exists(alt_s), fs::is_directory(alt_s));
            if (fs::exists(alt_s) && fs::is_directory(alt_s)) {
                UNICALIB_WARN("data.lidar 路径在本机不存在，已改用 --data-dir 映射: {} -> {}",
                              primary, alt_s);
                return alt_s;
            }
        }
    }
    UNICALIB_WARN("[LiDAR-LiDAR][Path] 未找到可用目录，返回 primary: {}", primary);
    return primary;
}

// data.lidar.<id> 可为 PCD 目录，或传感器描述 YAML（含 pointcloud_topic，与 data/.../lidar_horz.yaml 一致）
static std::optional<std::string> read_pointcloud_topic_from_lidar_data_entry(const std::string& data_dir,
                                                                              const std::string& entry) {
    if (entry.empty()) return std::nullopt;
    const auto try_file = [](const std::string& p) -> std::optional<std::string> {
        if (!fs::exists(p) || !fs::is_regular_file(p)) return std::nullopt;
        try {
            YAML::Node n = YAML::LoadFile(p);
            if (!n["pointcloud_topic"]) return std::nullopt;
            std::string t = n["pointcloud_topic"].as<std::string>();
            if (!t.empty()) return t;
        } catch (const std::exception& e) {
            UNICALIB_DEBUG("[LiDAR-LiDAR][SensorYaml] 读取失败 '{}': {}", p, e.what());
        }
        return std::nullopt;
    };
    std::string full = resolve_data_path(data_dir, entry);
    if (auto t = try_file(full)) return t;
    if (auto t = try_file(full + ".yaml")) return t;
    if (auto t = try_file(full + ".yml")) return t;
    return std::nullopt;
}

// 若 entry 解析为普通文件（如传感器 yaml），则不当作 PCD 目录；无扩展名时尝试 .yaml/.yml（与 read_pointcloud_topic 一致）
static std::string get_lidar_pcd_dir_for_entry(const std::string& data_dir, const std::string& entry) {
    if (entry.empty()) return "";
    std::string full = resolve_data_path(data_dir, entry);
    auto is_regular_file = [](const std::string& p) { return fs::exists(p) && fs::is_regular_file(p); };
    if (is_regular_file(full) || is_regular_file(full + ".yaml") || is_regular_file(full + ".yml"))
        return "";
    return resolve_lidar_dir_for_load(data_dir, entry);
}

static bool load_lidar_scans_from_dir(const std::string& dir,
                                      std::vector<LiDARScan>& scans,
                                      size_t max_frames) {
    UNICALIB_INFO("[LiDAR-LiDAR][Load] 开始加载目录: '{}' max_frames={}", dir, max_frames);
    if (!fs::exists(dir) || !fs::is_directory(dir)) {
        UNICALIB_WARN("[LiDAR-LiDAR][Load] 目录不存在或非目录: '{}' exists={} is_dir={}",
                      dir, fs::exists(dir), fs::is_directory(dir));
        return false;
    }
    std::vector<fs::path> pcd_files;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() == ".pcd") pcd_files.push_back(e.path());
    }
    std::sort(pcd_files.begin(), pcd_files.end());
    UNICALIB_INFO("[LiDAR-LiDAR][Load] 扫描完成: pcd_count={}", pcd_files.size());
    if (pcd_files.empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR][Load] 目录中无 .pcd 文件: '{}'", dir);
        return false;
    }
    size_t step = std::max(size_t(1), pcd_files.size() / max_frames);
    UNICALIB_INFO("[LiDAR-LiDAR][Load] 采样步长 step={}", step);
    for (size_t i = 0; i < pcd_files.size() && scans.size() < max_frames; i += step) {
        LiDARScan scan;
        scan.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        int rc = pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files[i].string(), *scan.cloud);
        if (rc == 0) {
            std::string fname = pcd_files[i].stem().string();
            try { scan.timestamp = std::stod(fname); }
            catch (...) { scan.timestamp = static_cast<double>(scans.size()) * 0.1; }
            scans.push_back(std::move(scan));
            UNICALIB_INFO("[LiDAR-LiDAR][Load] 加载成功: file='{}' points={} scans_now={}",
                          pcd_files[i].string(), scans.back().cloud ? scans.back().cloud->size() : 0, scans.size());
        } else {
            UNICALIB_WARN("[LiDAR-LiDAR][Load] 加载失败: file='{}' rc={}", pcd_files[i].string(), rc);
        }
    }
    UNICALIB_INFO("[LiDAR-LiDAR][Load] 加载结束: 有效帧数={}", scans.size());
    return !scans.empty();
}

#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
static std::vector<LiDARScan> ros_scans_to_lidar_scans(const std::vector<LiDARScanRos>& in) {
    std::vector<LiDARScan> out;
    out.reserve(in.size());
    for (const auto& r : in) {
        LiDARScan s;
        s.timestamp = r.timestamp;
        s.cloud = r.cloud;
        out.push_back(std::move(s));
    }
    return out;
}

// 仅从 bag 加载当前标定对需要的两路 LiDAR，不把 ros2.camera_topics 等带入配置，避免 strict 校验被无关传感器拖死。
static bool try_load_lidar_pair_from_ros2_bag(
    const YAML::Node& cfg,
    const std::string& data_dir,
    const std::string& ref_id,
    const std::string& target_id,
    const std::optional<std::string>& topic_ref_override,
    const std::optional<std::string>& topic_tgt_override,
    const std::map<std::string, std::string>& lidar_topics_from_sensors,
    int calib_max_frames,
    std::shared_ptr<Ros2BagDataSource>& bag_cache,
    std::string& cache_fingerprint,
    std::vector<LiDARScan>& scans_ref,
    std::vector<LiDARScan>& scans_target) {

    scans_ref.clear();
    scans_target.clear();

    YAML::Node ros2 = cfg["ros2"];
    if (!ros2 || !YG(ros2, "use_ros2_bag", false)) {
        UNICALIB_WARN("[LiDAR-LiDAR][Ros2] 未启用 ros2.use_ros2_bag，无法从 bag 回退");
        return false;
    }

    std::string bag_rel;
    if (ros2["ros2_bag_file"])
        bag_rel = ros2["ros2_bag_file"].as<std::string>();
    YAML::Node data = cfg["data"];
    if (bag_rel.empty() && data && data["bag_file"])
        bag_rel = data["bag_file"].as<std::string>();
    if (bag_rel.empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR][Ros2] 未配置 ros2.ros2_bag_file（或 data.bag_file）");
        return false;
    }

    std::string bag_path = resolve_data_path(data_dir, bag_rel);
    if (!fs::exists(bag_path)) {
        UNICALIB_WARN("[LiDAR-LiDAR][Ros2] bag 路径不存在: {}", bag_path);
        return false;
    }

    std::map<std::string, std::string> topics_full;
    if (ros2["lidar_topics"] && ros2["lidar_topics"].IsMap()) {
        for (auto it = ros2["lidar_topics"].begin(); it != ros2["lidar_topics"].end(); ++it)
            topics_full[it->first.as<std::string>()] = it->second.as<std::string>();
    }
    // 与 joint_calib / lidar_camera 一致：sensors[].id + topic 补充映射（不覆盖 ros2.lidar_topics 已有键）
    for (const auto& [id, topic] : lidar_topics_from_sensors) {
        if (topic.empty()) continue;
        if (topics_full.find(id) == topics_full.end()) {
            topics_full[id] = topic;
            UNICALIB_INFO("[LiDAR-LiDAR][Ros2] 话题来自 sensors[]: {} -> {}", id, topic);
        }
    }

    auto get_topic = [&](const std::string& id) -> std::string {
        auto it = topics_full.find(id);
        return it == topics_full.end() ? std::string() : it->second;
    };
    std::string topic_ref = topic_ref_override.value_or(std::string());
    if (topic_ref.empty()) topic_ref = get_topic(ref_id);
    std::string topic_tgt = topic_tgt_override.value_or(std::string());
    if (topic_tgt.empty()) topic_tgt = get_topic(target_id);
    if (topic_ref.empty() || topic_tgt.empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR][Ros2] 缺少 PointCloud2 话题: 请保证 sensors[].id 与 lidar_lidar.pairs 一致并填写 topic，"
                      "或在 data.lidar.<id> 使用传感器 YAML（pointcloud_topic），或在 ros2.lidar_topics 中配置 '{}' 与 '{}'",
                      ref_id, target_id);
        return false;
    }
    if (topic_ref_override)
        UNICALIB_INFO("[LiDAR-LiDAR][Ros2] ref 话题来自传感器 YAML: {}", topic_ref);
    if (topic_tgt_override)
        UNICALIB_INFO("[LiDAR-LiDAR][Ros2] target 话题来自传感器 YAML: {}", topic_tgt);
    if (topic_ref == topic_tgt) {
        UNICALIB_ERROR("[LiDAR-LiDAR][Ros2] ref/target 指向同一话题 '{}', 无法做 LiDAR-LiDAR 标定", topic_ref);
        return false;
    }

    RosDataSourceConfig ros_cfg;
    ros_cfg.bag_file = bag_path;
    ros_cfg.lidar_topics[ref_id] = topic_ref;
    ros_cfg.lidar_topics[target_id] = topic_tgt;
    ros_cfg.strict_topic_match = YG(ros2, "strict_topic_match", true);
    // ros2.max_frames 常为大值（如 IMU-LiDAR 用 800）；此处必须受 lidar_lidar.max_frames 约束，否则双路 800 帧
    // 全量进内存易导致 OOM / PCL GICP 在刚打印 [fine] 0% 后崩溃。
    int mf_ros = YG(ros2, "max_frames", calib_max_frames);
    if (mf_ros <= 0) mf_ros = calib_max_frames;
    const int cap = calib_max_frames > 0 ? calib_max_frames : mf_ros;
    int mf = std::min(mf_ros, cap);
    if (mf <= 0) mf = std::max(1, mf_ros);
    ros_cfg.max_frames = static_cast<size_t>(std::max(1, mf));
    if (mf_ros > mf)
        UNICALIB_INFO("[LiDAR-LiDAR][Ros2] bag 帧数已由 ros2.max_frames={} 限制为 lidar_lidar.max_frames={} -> {}",
                      mf_ros, cap, ros_cfg.max_frames);

    std::string fingerprint = bag_path + "|" + ref_id + "=" + topic_ref + "|" + target_id + "=" + topic_tgt +
                              "|strict=" + std::string(ros_cfg.strict_topic_match ? "1" : "0") +
                              "|maxf=" + std::to_string(ros_cfg.max_frames);

    if (!bag_cache || cache_fingerprint != fingerprint) {
        UNICALIB_INFO("[LiDAR-LiDAR][Ros2] 打开 bag: {} (ref {}->{}, target {}->{})",
                      bag_path, ref_id, topic_ref, target_id, topic_tgt);
        bag_cache = std::make_shared<Ros2BagDataSource>(ros_cfg);
        if (!bag_cache->load()) {
            UNICALIB_WARN("[LiDAR-LiDAR][Ros2] bag 加载失败: {}", bag_cache->get_status_message());
            bag_cache.reset();
            cache_fingerprint.clear();
            return false;
        }
        cache_fingerprint = fingerprint;
    }

    auto vref = ros_scans_to_lidar_scans(bag_cache->get_lidar_scans(ref_id));
    auto vtgt = ros_scans_to_lidar_scans(bag_cache->get_lidar_scans(target_id));
    if (vref.empty() || vtgt.empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR][Ros2] 从 bag 取帧为空: ref_frames={} target_frames={}", vref.size(), vtgt.size());
        return false;
    }

    scans_ref = std::move(vref);
    scans_target = std::move(vtgt);
    UNICALIB_INFO("[LiDAR-LiDAR][Ros2] 已从 bag 加载: ref {} 帧, target {} 帧", scans_ref.size(), scans_target.size());
    return true;
}
#endif

static void save_extrinsic_yaml(const ExtrinsicSE3& ext, const std::string& path) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "calibration_type" << YAML::Value << "lidar_lidar_extrinsic";
    out << YAML::Key << "ref_sensor" << YAML::Value << ext.ref_sensor_id;
    out << YAML::Key << "target_sensor" << YAML::Value << ext.target_sensor_id;
    Eigen::Quaterniond q = ext.SO3_TargetInRef.unit_quaternion();
    out << YAML::Key << "quaternion" << YAML::Value << YAML::Flow
        << YAML::BeginSeq << q.w() << q.x() << q.y() << q.z() << YAML::EndSeq;
    out << YAML::Key << "translation_m" << YAML::Value << YAML::Flow
        << YAML::BeginSeq << ext.POS_TargetInRef[0] << ext.POS_TargetInRef[1] << ext.POS_TargetInRef[2] << YAML::EndSeq;
    Eigen::Vector3d euler = ext.euler_deg();
    out << YAML::Key << "euler_zyx_deg" << YAML::Value << YAML::Flow
        << YAML::BeginSeq << euler[0] << euler[1] << euler[2] << YAML::EndSeq;

    // 添加 4x4 变换矩阵
    Eigen::Matrix4d mat = ext.SE3_TargetInRef().matrix();
    out << YAML::Key << "transformation_matrix" << YAML::BeginSeq;
    for (int i = 0; i < 4; ++i) {
        out << YAML::Flow << YAML::BeginSeq
            << mat(i, 0) << mat(i, 1) << mat(i, 2) << mat(i, 3) << YAML::EndSeq;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "time_offset_s" << YAML::Value << ext.time_offset_s;
    out << YAML::Key << "residual_rms" << YAML::Value << ext.residual_rms;
    out << YAML::Key << "converged" << YAML::Value << ext.is_converged;
    out << YAML::EndMap;
    std::ofstream f(path);
    if (f.is_open()) f << out.c_str();
}

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — LiDAR-LiDAR 外参标定                     ║
 ║   两阶段: NDT粗标定 → GICP/NDT精标定                   ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_lidar_lidar [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --data-dir <dir>        数据根目录 (与 config data.lidar 相对路径拼接)\n"
        "  --output-dir <dir>      输出目录 (默认: ./results)\n"
        "  --no-viz                禁用可视化界面 (默认启用)\n"
        "  --manual                自动标定完成后启用手动调整可视化 (双点云 6-DOF)\n"
        "  --log-level <l>         trace|debug|info|warn|error\n"
        "  --help/-h               显示帮助\n\n"
        "配置: lidar_lidar.pairs [[ref_id, target_id]]；话题: sensors[].topic（id 须与 pairs 一致）、或 data.lidar 传感器 YAML、或 ros2.lidar_topics；"
        "data.lidar.<id> 可为 PCD 目录\n";
}

int main(int argc, char** argv) {
    print_banner();

    std::string config_file, data_dir, output_dir = "./results", log_level = "info";
    bool no_viz = false;
    bool manual = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i + 1 < argc) config_file = argv[++i];
        else if (a == "--data-dir" && i + 1 < argc) data_dir = argv[++i];
        else if (a == "--output-dir" && i + 1 < argc) output_dir = argv[++i];
        else if (a == "--no-viz") no_viz = true;
        else if (a == "--manual") manual = true;
        else if (a == "--log-level" && i + 1 < argc) log_level = argv[++i];
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }
    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config\n";
        print_help();
        return 1;
    }
    if (data_dir.empty() && std::getenv("CALIB_DATA_DIR")) data_dir = std::getenv("CALIB_DATA_DIR");
    std::cout << "[LiDAR-LiDAR][Args] config=" << config_file
              << " data_dir=" << (data_dir.empty() ? "(empty)" : data_dir)
              << " output_dir=" << output_dir
              << " no_viz=" << (no_viz ? "true" : "false")
              << " manual=" << (manual ? "true" : "false")
              << " log_level=" << log_level << std::endl;

    UNICALIB_MAIN_TRY_BEGIN

    std::string logs_dir = ns_unicalib::resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/lidar_lidar_" + ns_unicalib::log_timestamp_filename() + ".log";
    spdlog::level::level_enum lv = spdlog::level::info;
    if (log_level == "trace") lv = spdlog::level::trace;
    else if (log_level == "debug") lv = spdlog::level::debug;
    else if (log_level == "warn") lv = spdlog::level::warn;
    else if (log_level == "error") lv = spdlog::level::err;
    Logger::init("LiDAR-LiDAR", log_file, lv);
    UNICALIB_INFO("[LiDAR-LiDAR][Init] 日志文件: {}", log_file);

    YAML::Node cfg;
    try { cfg = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
        UNICALIB_ERROR("配置加载失败: {}", e.what());
        return 1;
    }
    if (cfg["output_dir"]) output_dir = cfg["output_dir"].as<std::string>();
    UNICALIB_INFO("[LiDAR-LiDAR][Init] 最终 output_dir='{}' data_dir='{}' manual={} no_viz={}",
                  output_dir, data_dir, manual, no_viz);

    // lidar_lidar 段
    YAML::Node ll = cfg["lidar_lidar"];
    if (!ll) {
        UNICALIB_ERROR("配置中缺少 lidar_lidar 段");
        return 1;
    }
    std::vector<std::pair<std::string, std::string>> pairs;
    if (ll["pairs"] && ll["pairs"].IsSequence()) {
        for (const auto& p : ll["pairs"]) {
            if (p.IsSequence() && p.size() >= 2)
                pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
    }
    if (pairs.empty() && ll["lidar_list"] && ll["lidar_list"].IsSequence() && ll["lidar_list"].size() >= 2) {
        const YAML::Node list = ll["lidar_list"];
        pairs.emplace_back(list[0].as<std::string>(), list[1].as<std::string>());
    }
    if (pairs.empty()) {
        UNICALIB_ERROR("lidar_lidar.pairs 或 lidar_list 未配置有效标定对");
        return 1;
    }
    UNICALIB_INFO("[LiDAR-LiDAR][Init] 标定对数量: {}", pairs.size());

    std::map<std::string, std::string> lidar_topics_from_sensors;
    try {
        SystemConfig sys_cfg = YamlIO::load_system_config(config_file);
        for (const auto& s : sys_cfg.sensors) {
            if (s.type == SensorType::LiDAR && !s.topic.empty())
                lidar_topics_from_sensors[s.sensor_id] = s.topic;
        }
        UNICALIB_INFO("[LiDAR-LiDAR][Init] 从 sensors[] 读取 LiDAR 话题 {} 条（与 lidar_lidar.pairs 的 id 对应即可）",
                      lidar_topics_from_sensors.size());
    } catch (const std::exception& e) {
        UNICALIB_DEBUG("[LiDAR-LiDAR][Init] load_system_config 未用或失败: {}", e.what());
    }

    LiDARLiDARConfig calib_cfg;
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");
    if (ll["voxel_size"]) calib_cfg.voxel_size = ll["voxel_size"].as<double>();
    if (ll["min_overlap_ratio"]) calib_cfg.min_overlap_ratio = ll["min_overlap_ratio"].as<double>();
    if (ll["max_frames"]) calib_cfg.max_frames = ll["max_frames"].as<int>();
    if (ll["coarse"] && ll["coarse"].IsMap()) {
        const auto& c = ll["coarse"];
        calib_cfg.use_fpfh_teaser_coarse = YG(c, "use_fpfh_teaser", true);
        if (c["fpfh_radius"]) calib_cfg.fpfh_radius = c["fpfh_radius"].as<double>();
        if (c["teaser_noise_bound"]) calib_cfg.teaser_noise_bound = c["teaser_noise_bound"].as<double>();
        calib_cfg.use_gmm_registration = YG(c, "use_gmm", false);
        if (c["gmm_max_iter"]) calib_cfg.gmm_max_iterations = c["gmm_max_iter"].as<int>();
    }
    if (ll["fine"] && ll["fine"].IsMap()) {
        const auto& f = ll["fine"];
        if (f["gicp_resolution"]) calib_cfg.gicp_max_corr_dist = f["gicp_resolution"].as<double>();
        if (f["gicp_max_iter"]) calib_cfg.gicp_max_iterations = f["gicp_max_iter"].as<int>();
        calib_cfg.use_multi_frame_fine = YG(f, "use_multi_frame_fine", true);
        if (f["fine_fusion_max_frames"]) calib_cfg.fine_fusion_max_frames = f["fine_fusion_max_frames"].as<int>();
        if (f["fine_fusion_method"]) calib_cfg.fine_fusion_method = f["fine_fusion_method"].as<std::string>();
        if (f["spline_dt_s"]) calib_cfg.spline_dt_s = f["spline_dt_s"].as<double>();
        if (f["spline_order"]) calib_cfg.spline_order = f["spline_order"].as<int>();
        if (f["optimize_time_offset"]) calib_cfg.optimize_time_offset = f["optimize_time_offset"].as<bool>();
    }
    if (ll["use_ndt"]) calib_cfg.use_ndt = ll["use_ndt"].as<bool>();

    std::optional<Sophus::SE3d> lidar_lidar_init_from_config;
    if (ll["initial_extrinsic"])
        lidar_lidar_init_from_config = parse_se3_from_yaml_4x4(ll["initial_extrinsic"]);
    if (lidar_lidar_init_from_config.has_value()) {
        UNICALIB_INFO("lidar_lidar.initial_extrinsic 已加载，将跳过粗标定并以该初值进入精标定/手动微调");
        if (pairs.size() > 1)
            UNICALIB_WARN("pairs 含多对雷达：当前对所有对共用同一 initial_extrinsic，若不对请改为仅一对或扩展配置");
    } else if (ll["initial_extrinsic"])
        UNICALIB_WARN("lidar_lidar.initial_extrinsic 存在但解析失败，将使用自动粗标定");

    // 数据路径: data.lidar.<id> 可为 PCD 目录，或传感器 YAML（pointcloud_topic，见 data/.../lidar_horz.yaml）
    YAML::Node data = cfg["data"];
    auto get_lidar_pcd_dir = [&](const std::string& id) -> std::string {
        if (!data["lidar"] || !data["lidar"][id]) return "";
        return get_lidar_pcd_dir_for_entry(data_dir, data["lidar"][id].as<std::string>());
    };
    auto get_topic_from_lidar_yaml = [&](const std::string& id) -> std::optional<std::string> {
        if (!data["lidar"] || !data["lidar"][id]) return std::nullopt;
        return read_pointcloud_topic_from_lidar_data_entry(data_dir, data["lidar"][id].as<std::string>());
    };

    fs::create_directories(output_dir);
    std::string result_subdir = output_dir + "/lidar_lidar_extrinsic";
    fs::create_directories(result_subdir);

    LiDARLiDARCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& stage, double p) {
        UNICALIB_INFO("  [{}] {:.0f}%", stage, p * 100.0);
    });

#if UNICALIB_WITH_PANGOLIN
    if (!no_viz) {
        try {
            auto viz = CalibVisualizer::Create();
            viz->start_display("UniCalib LiDAR-LiDAR", 1280, 720);
            calibrator.set_visualizer(viz);
            calibrator.enable_realtime_viz(true);
            UNICALIB_INFO("可视化已启用（默认）");
        } catch (const std::exception& e) {
            UNICALIB_WARN("可视化启动失败（继续无界面运行）: {}", e.what());
        }
    } else {
        UNICALIB_INFO("[LiDAR-LiDAR][Viz] --no-viz 已设置，不创建自动可视化窗口");
    }
#endif

#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
    std::shared_ptr<Ros2BagDataSource> ros_bag_cache;
    std::string ros_bag_cache_fingerprint;
#endif

    int done = 0;
    for (const auto& [ref_id, target_id] : pairs) {
        UNICALIB_INFO("━━━ 标定对: {} -> {} ━━━", ref_id, target_id);
        std::string ref_path = get_lidar_pcd_dir(ref_id);
        std::string tgt_path = get_lidar_pcd_dir(target_id);
        std::optional<std::string> ref_topic_ov = get_topic_from_lidar_yaml(ref_id);
        std::optional<std::string> tgt_topic_ov = get_topic_from_lidar_yaml(target_id);
        UNICALIB_INFO("[LiDAR-LiDAR][Pair] PCD 目录: ref_id='{}' -> '{}' ; target_id='{}' -> '{}'",
                      ref_id, ref_path.empty() ? "(无)" : ref_path, target_id, tgt_path.empty() ? "(无)" : tgt_path);
        if (ref_topic_ov) UNICALIB_INFO("[LiDAR-LiDAR][Pair] ref 传感器 YAML 中 pointcloud_topic: {}", *ref_topic_ov);
        if (tgt_topic_ov) UNICALIB_INFO("[LiDAR-LiDAR][Pair] target 传感器 YAML 中 pointcloud_topic: {}", *tgt_topic_ov);
        std::vector<LiDARScan> scans_ref, scans_target;
        bool pcd_ok = false;
        if (!ref_path.empty() && !tgt_path.empty()) {
            pcd_ok = load_lidar_scans_from_dir(ref_path, scans_ref, static_cast<size_t>(calib_cfg.max_frames)) &&
                     load_lidar_scans_from_dir(tgt_path, scans_target, static_cast<size_t>(calib_cfg.max_frames));
        } else {
            UNICALIB_INFO("[LiDAR-LiDAR][Pair] data.lidar 未配置或路径为空，跳过 PCD 目录加载");
        }
        if (!pcd_ok) {
            UNICALIB_WARN("  PCD 目录不可用或未配置，尝试 ROS2 bag 回退（需 ros2.use_ros2_bag；话题来自传感器 YAML 的 "
                          "pointcloud_topic 或 ros2.lidar_topics）");
#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
            if (!try_load_lidar_pair_from_ros2_bag(cfg, data_dir, ref_id, target_id, ref_topic_ov, tgt_topic_ov,
                                                   lidar_topics_from_sensors, calib_cfg.max_frames, ros_bag_cache,
                                                   ros_bag_cache_fingerprint, scans_ref, scans_target)) {
                UNICALIB_WARN("  跳过: 无法从目录或 bag 加载点云 ref_id={} target_id={}", ref_id, target_id);
                if (data_dir.empty())
                    UNICALIB_WARN("  提示: 使用 bag 时同样需要 --data-dir 解析 ros2.ros2_bag_file 相对路径");
                else
                    UNICALIB_WARN("  提示: 提供含 .pcd 的目录，或在 data.lidar 使用含 pointcloud_topic 的传感器 YAML，"
                                  "或在 ros2.lidar_topics 中配置两路 PointCloud2 话题");
                continue;
            }
#else
            UNICALIB_WARN("  跳过: 无法从目录加载点云（本构建未启用 UNICALIB_WITH_ROS2，无法从 bag 回退）");
            if (data_dir.empty())
                UNICALIB_WARN("  提示: 若 YAML 中为其它环境绝对路径，请使用 --data-dir <本机数据根>（或与 "
                              "配置中 /data/ 之后路径一致）");
            else
                UNICALIB_WARN("  提示: 确认目录存在且含 .pcd；相对路径相对于 --data-dir");
            continue;
#endif
        }
        UNICALIB_INFO("  加载点云: ref {} 帧, target {} 帧", scans_ref.size(), scans_target.size());
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] main: calling calibrate_two_stage pair {} -> {}", ref_id, target_id);
        Logger::flush();

        auto result = calibrator.calibrate_two_stage(scans_ref, scans_target, ref_id, target_id,
                                                     lidar_lidar_init_from_config);
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] main: calibrate_two_stage returned best={}",
                         result.best() ? "yes" : "no");
        Logger::flush();
        const ExtrinsicSE3* best = result.best();
        if (!best) {
            UNICALIB_WARN("  标定失败: {}", result.failure_reason.empty() ? "无结果" : result.failure_reason);
            continue;
        }
        ExtrinsicSE3 final_ext = *best;
        if (manual) {
            UNICALIB_INFO("[LiDAR-LiDAR][Manual] manual=true，准备进入手动微调");
#if UNICALIB_WITH_PANGOLIN
            // 解决窗口冲突：进入手动模式前关闭自动模式的可视化窗口
            if (calibrator.get_visualizer()) {
                UNICALIB_INFO("[LiDAR-LiDAR][Manual] 关闭自动可视化窗口，避免 Pangolin 上下文冲突");
                calibrator.get_visualizer()->close_display();
            } else {
                UNICALIB_INFO("[LiDAR-LiDAR][Manual] 无自动可视化窗口对象，跳过 close_display");
            }
#endif
            double auto_fitness = 0.0;
            if (const auto* q = result.best_quality()) auto_fitness = q->fitness_score;
            UNICALIB_INFO("[LiDAR-LiDAR][Manual] auto_fitness={:.6f}", auto_fitness);
            ManualCalibSession::SessionConfig sess_cfg;
            sess_cfg.enable_interactive_gui = true;
            ManualCalibSession session(sess_cfg);
            UNICALIB_INFO("  ===== 手动标定操作说明 =====");
            UNICALIB_INFO("  左栏: 填 roll/pitch/yaw(deg) 与 tx/ty/tz(m) 后点 Apply RPY+T，再 +/- 或键盘微调");
            UNICALIB_INFO("  按键: q/a w/s e/d 旋转, r/f t/g y/h 平移, u=撤销, Enter=接受, Esc=取消");
            const auto [ir_man, it_man] = time_aligned_lidar_scan_indices_for_manual(scans_ref, scans_target);
            const LiDARScan& sref_man = scans_ref[ir_man];
            const LiDARScan& stgt_man = scans_target[it_man];
            UNICALIB_INFO("[LiDAR-LiDAR][Manual] 可视化使用时间对齐帧: ref[{}] t={:.6f}, target[{}] t={:.6f}, |Δt|={:.6f}s, points={}/{}",
                          ir_man, sref_man.timestamp, it_man, stgt_man.timestamp,
                          std::fabs(sref_man.timestamp - stgt_man.timestamp),
                          sref_man.cloud ? sref_man.cloud->size() : 0,
                          stgt_man.cloud ? stgt_man.cloud->size() : 0);
            final_ext = session.run_lidar_lidar(final_ext, auto_fitness, &sref_man, &stgt_man);
            UNICALIB_INFO("[LiDAR-LiDAR][Manual] 手动微调结束，rpy_deg={} xyz_m={}",
                          final_ext.euler_deg().transpose(), final_ext.translation().transpose());
        }
        std::string out_key = ref_id + "_to_" + target_id + ".yaml";
        std::string out_path = result_subdir + "/" + out_key;
        save_extrinsic_yaml(final_ext, out_path);
        UNICALIB_INFO("  结果已保存: {}", out_path);
        done++;
    }

    UNICALIB_INFO("LiDAR-LiDAR 标定完成: {} 对", done);
    UNICALIB_MAIN_TRY_END(0)
}
