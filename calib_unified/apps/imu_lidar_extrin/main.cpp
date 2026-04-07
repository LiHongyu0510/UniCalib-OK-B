/**
 * UniCalib — IMU-LiDAR 外参标定应用
 *
 * 两阶段流程:
 *   Stage 1 粗标定: L2Calib RL强化学习 (--coarse --bag)
 *   Stage 2 精标定: B样条连续时间优化 (无目标, 始终)
 *   手动校准:       --manual 触发旋转可视化验证 + 增量调整
 *
 * 用法:
 *   unicalib_imu_lidar --config <config.yaml>
 *   unicalib_imu_lidar --config <config.yaml> --coarse --bag <bag_file>
 *   unicalib_imu_lidar --config <config.yaml> --manual
 */
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <cstdlib>
#include <set>

namespace fs = std::filesystem;
using namespace ns_unicalib;

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — IMU-LiDAR 外参标定                       ║
 ║   两阶段: L2Calib RL粗估 → B样条连续时间精化           ║
 ║   手动校准: --manual 触发旋转可视化验证                 ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_imu_lidar [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --coarse                启用 L2Calib AI 粗标定\n"
        "  --bag <file>            ROS bag 文件 (L2Calib 需要)\n"
        "  --ai-root <dir>         AI 工程根目录\n"
        "  --manual                精标定后启用手动校准\n"
        "  --log-level <l>         trace|debug|info|warn|error\n"
        "  --output-dir <dir>      输出目录\n"
        "  --help/-h               显示帮助\n\n"
        "配置文件字段:\n"
        "  imu_data_file:          IMU 数据 (CSV/YAML)\n"
        "  lidar_data_dir:         LiDAR PCD 目录\n"
        "  imu_id:                 IMU 传感器 ID (默认 imu_0)\n"
        "  lidar_id:               LiDAR 传感器 ID (默认 lidar_0)\n"
        "  ndt_resolution:         NDT 分辨率 [m] (默认 1.0)\n"
        "  spline_dt_s:            B样条结间距 [s] (默认 0.1)\n"
        "  optimize_time_offset:   是否优化时间偏移 (默认 true)\n"
        "  manual_rot_threshold:   触发手动校准的旋转误差 [deg] (默认 0.5)\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    std::string config_file, bag_file;
    std::string ai_root    = "../";
    std::string output_dir = "./results";
    std::string log_level  = "info";
    bool do_coarse = false;
    bool do_manual = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--coarse")                 do_coarse = true;
        else if (a == "--manual")                 do_manual = true;
        else if (a == "--bag" && i+1 < argc)      bag_file = argv[++i];
        else if (a == "--ai-root" && i+1 < argc)  ai_root = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--log-level" && i+1 < argc) log_level = argv[++i];
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    UNICALIB_MAIN_TRY_BEGIN

    std::string logs_dir = resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/imu_lidar_" + log_timestamp_filename() + ".log";
    Logger::init("IMU-LiDAR",
                 log_file,
                 log_level == "debug" ? spdlog::level::debug :
                 log_level == "trace" ? spdlog::level::trace :
                 log_level == "warn"  ? spdlog::level::warn  :
                                        spdlog::level::info);

    YAML::Node cfg;
    try { cfg = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
        UNICALIB_ERROR("配置加载失败: {}", e.what()); return 1;
    }

    if (cfg["output_dir"]) output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["bag_file"] && bag_file.empty()) bag_file = cfg["bag_file"].as<std::string>();

    double manual_thresh = cfg["manual_rot_threshold"] ?
        cfg["manual_rot_threshold"].as<double>() : 0.5;

    std::string imu_id   = cfg["imu_id"]   ? cfg["imu_id"].as<std::string>()   : "imu_0";
    std::string lidar_id = cfg["lidar_id"] ? cfg["lidar_id"].as<std::string>() : "lidar_0";

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置: {} | IMU: {} | LiDAR: {}", config_file, imu_id, lidar_id);
    UNICALIB_INFO("AI粗标定: {} | 手动校准: {} (阈值={:.2f}deg)",
                  do_coarse, do_manual, manual_thresh);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─── Stage 1: L2Calib 粗标定 ──────────────────────────────────────
    std::optional<Sophus::SE3d> coarse_init;
    if (do_coarse) {
        UNICALIB_INFO("▶ Stage 1: AI 粗标定 (L2Calib SE(3)-流形RL)");

        if (bag_file.empty()) {
            UNICALIB_WARN("  --coarse 需要 --bag <bag_file> 或 config.yaml::bag_file");
        } else {
            AICoarseCalibManager::Config ai_cfg;
            ai_cfg.ai_root = ai_root;
            if (cfg["python_exe"])
                ai_cfg.python_exe = cfg["python_exe"].as<std::string>();
            if (cfg["l2calib_epochs"])
                ai_cfg.l2calib.num_epochs = cfg["l2calib_epochs"].as<int>();
            if (cfg["imu_topic"])
                ai_cfg.l2calib.imu_topic = cfg["imu_topic"].as<std::string>();
            if (cfg["lidar_type"])
                ai_cfg.l2calib.lidar_type = cfg["lidar_type"].as<std::string>();

            AICoarseCalibManager ai_mgr(ai_cfg);
            ai_mgr.print_availability();

            auto result = ai_mgr.coarse_imu_lidar(bag_file, imu_id, lidar_id);
            if (result.has_value()) {
                coarse_init = result->SE3_TargetInRef();
                UNICALIB_INFO("  ✓ L2Calib 粗估: t=[{:.3f},{:.3f},{:.3f}]m",
                              coarse_init->translation().x(),
                              coarse_init->translation().y(),
                              coarse_init->translation().z());
            } else {
                UNICALIB_WARN("  ✗ L2Calib 粗标定失败, 使用手眼标定初始化");
            }
        }
    }

    // ─── Stage 2: B样条精标定 ─────────────────────────────────────────
    UNICALIB_INFO("▶ Stage 2: B样条连续时间精标定");

    IMULiDARCalibrator::Config calib_cfg;
    if (cfg["ndt_resolution"])    calib_cfg.ndt_resolution    = cfg["ndt_resolution"].as<double>();
    if (cfg["spline_dt_s"])       calib_cfg.spline_dt_s       = cfg["spline_dt_s"].as<double>();
    if (cfg["ceres_max_iter"])    calib_cfg.ceres_max_iter     = cfg["ceres_max_iter"].as<int>();
    if (cfg["optimize_time_offset"]) calib_cfg.optimize_time_offset = cfg["optimize_time_offset"].as<bool>();
    if (cfg["min_motion_rot_deg"]) calib_cfg.min_motion_rot_deg = cfg["min_motion_rot_deg"].as<double>();
    if (cfg["rot_pair_dt_target_s"]) calib_cfg.rot_pair_dt_target_s = cfg["rot_pair_dt_target_s"].as<double>();
    if (cfg["min_motion_rot_deg_short"]) calib_cfg.min_motion_rot_deg_short = cfg["min_motion_rot_deg_short"].as<double>();
    if (cfg["rot_pair_short_step_s"]) calib_cfg.rot_pair_short_step_s = cfg["rot_pair_short_step_s"].as<double>();
    // 从 imu_lidar 子节点读取（与 joint 配置一致）
    if (cfg["imu_lidar"]) {
        const auto& il = cfg["imu_lidar"];
        if (il["rot_pair_dt_target_s"]) calib_cfg.rot_pair_dt_target_s = il["rot_pair_dt_target_s"].as<double>();
        if (il["min_motion_rot_deg_short"]) calib_cfg.min_motion_rot_deg_short = il["min_motion_rot_deg_short"].as<double>();
        if (il["rot_pair_short_step_s"]) calib_cfg.rot_pair_short_step_s = il["rot_pair_short_step_s"].as<double>();
        if (il["handeye_fix_180_ambiguity"]) calib_cfg.handeye_fix_180_ambiguity = il["handeye_fix_180_ambiguity"].as<bool>();
        if (il["handeye_180_decision"]) calib_cfg.handeye_180_decision = il["handeye_180_decision"].as<std::string>();
        if (il["handeye_prefer_identity_when_ambiguous"]) calib_cfg.handeye_prefer_identity_when_ambiguous = il["handeye_prefer_identity_when_ambiguous"].as<bool>();
        if (il["handeye_180_residual_margin_deg"]) calib_cfg.handeye_180_residual_margin_deg = il["handeye_180_residual_margin_deg"].as<double>();
        if (il["handeye_ratio_min"]) calib_cfg.handeye_ratio_min = il["handeye_ratio_min"].as<double>();
        if (il["handeye_ratio_max"]) calib_cfg.handeye_ratio_max = il["handeye_ratio_max"].as<double>();
        if (il["handeye_outlier_reject_quantile"]) calib_cfg.handeye_outlier_reject_quantile = il["handeye_outlier_reject_quantile"].as<double>();
        if (il["handeye_outlier_max_iter"]) calib_cfg.handeye_outlier_max_iter = il["handeye_outlier_max_iter"].as<int>();
        if (il["handeye_outlier_min_pairs"]) calib_cfg.handeye_outlier_min_pairs = il["handeye_outlier_min_pairs"].as<int>();
        if (il["handeye_ransac_enable"]) calib_cfg.handeye_ransac_enable = il["handeye_ransac_enable"].as<bool>();
        if (il["handeye_ransac_inlier_thresh_deg"]) calib_cfg.handeye_ransac_inlier_thresh_deg = il["handeye_ransac_inlier_thresh_deg"].as<double>();
        if (il["handeye_ransac_max_iter"]) calib_cfg.handeye_ransac_max_iter = il["handeye_ransac_max_iter"].as<int>();
        if (il["use_planar_prior"]) calib_cfg.use_planar_prior = il["use_planar_prior"].as<bool>();
        if (il["min_roll_motion_deg"]) calib_cfg.min_roll_motion_deg = il["min_roll_motion_deg"].as<double>();
        if (il["min_pitch_motion_deg"]) calib_cfg.min_pitch_motion_deg = il["min_pitch_motion_deg"].as<double>();
        if (il["min_yaw_motion_deg"]) calib_cfg.min_yaw_motion_deg = il["min_yaw_motion_deg"].as<double>();
        if (il["max_z_trans_ratio"]) calib_cfg.max_z_trans_ratio = il["max_z_trans_ratio"].as<double>();
        if (il["enable_planar_warning"]) calib_cfg.enable_planar_warning = il["enable_planar_warning"].as<bool>();
    }
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");

    IMULiDARCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& stage, double prog) {
        UNICALIB_INFO("  [进度] {}: {:.1f}%", stage, prog * 100.0);
    });

    UNICALIB_INFO("  IMU数据: {}",
                  cfg["imu_data_file"] ? cfg["imu_data_file"].as<std::string>() : "(未设置)");
    UNICALIB_INFO("  LiDAR目录: {}",
                  cfg["lidar_data_dir"] ? cfg["lidar_data_dir"].as<std::string>() : "(未设置)");
    UNICALIB_INFO("  B样条间距: {}s | 时间偏移优化: {}",
                  calib_cfg.spline_dt_s, calib_cfg.optimize_time_offset);

    // ─── Stage 3: 手动校准 ────────────────────────────────────────────
    if (do_manual) {
        UNICALIB_INFO("▶ Stage 3 (手动校准): 旋转可视化验证 + 增量调整");
        UNICALIB_INFO("  阈值: {:.2f}deg | 使用 ManualCalibSession::run_imu_lidar()",
                      manual_thresh);
        UNICALIB_INFO("  可视化: IMU积分轨迹 vs LiDAR里程计对比");
    }

    // ─── 构造 PipelineConfig（与统一配置/ROS2 配置保持一致）────────────
    PipelineConfig pipe_cfg;
    pipe_cfg.tasks                 = CalibTaskType::IMU_LIDAR_EXTRIN;
    pipe_cfg.enable_coarse_imu_lidar = do_coarse;
    pipe_cfg.allow_manual_fallback   = do_manual;
    pipe_cfg.imu_lidar_rot_threshold = manual_thresh;
    pipe_cfg.output_dir              = output_dir;
    pipe_cfg.log_level               = log_level;

    // 1) 读取 ROS2 段，填充 use_ros2_bag / ros2_bag_file 等
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
        if (ros2_node["camera_topic"])
            pipe_cfg.camera_ros2_topic = ros2_node["camera_topic"].as<std::string>();
        if (ros2_node["imu_topic"])
            pipe_cfg.imu_ros2_topic = ros2_node["imu_topic"].as<std::string>();
        if (ros2_node["strict_topic_match"])
            pipe_cfg.ros2_strict_topic_match = ros2_node["strict_topic_match"].as<bool>();
    }

    // data.bag_file 作为 ros2_bag_file 的后备（与统一配置保持一致）
    if (pipe_cfg.ros2_bag_file.empty() && cfg["data"] && cfg["data"]["bag_file"]) {
        pipe_cfg.ros2_bag_file = cfg["data"]["bag_file"].as<std::string>();
    }

    // 解析基础数据目录：CALIB_DATA_DIR 环境变量（由一键脚本设置）
    std::string base_data_dir;
    if (const char* env_p = std::getenv("CALIB_DATA_DIR")) {
        base_data_dir = env_p;
    }

    // 如果使用 ROS2 bag，且路径为相对路径，则相对于 base_data_dir 解析为绝对路径
    if (pipe_cfg.use_ros2_bag && !pipe_cfg.ros2_bag_file.empty() && !base_data_dir.empty()) {
        fs::path p(pipe_cfg.ros2_bag_file);
        if (!p.is_absolute()) {
            pipe_cfg.ros2_bag_file = (fs::path(base_data_dir) / p).string();
        }
    }

    // 2) 从统一配置的 sensors 段填充 IMU/LiDAR 话题映射
    if (cfg["sensors"]) {
        for (const auto& s : cfg["sensors"]) {
            if (!s["id"] || !s["type"])
                continue;
            std::string sid   = s["id"].as<std::string>();
            std::string stype = s["type"].as<std::string>();
            std::string topic;
            if (s["topic"])
                topic = s["topic"].as<std::string>();

            if (stype == "imu") {
                if (!topic.empty()) {
                    pipe_cfg.imu_topics[sid] = topic;
                    if (pipe_cfg.imu_ros2_topic.empty())
                        pipe_cfg.imu_ros2_topic = topic;
                }
                if (pipe_cfg.imu_sensor_id.empty())
                    pipe_cfg.imu_sensor_id = sid;
            } else if (stype == "lidar") {
                if (!topic.empty()) {
                    pipe_cfg.lidar_topics[sid] = topic;
                    if (pipe_cfg.lidar_ros2_topic.empty()) {
                        pipe_cfg.lidar_ros2_topic = topic;
                        pipe_cfg.lidar_id = sid;
                    }
                }
            }
        }
    }

    // 3) IMU-LiDAR 标定对：优先使用 imu_lidar.pairs（与 joint_calib 一致）
    if (cfg["imu_lidar"] && cfg["imu_lidar"]["pairs"]) {
        for (const auto& p : cfg["imu_lidar"]["pairs"]) {
            if (!p.IsSequence() || p.size() != 2) continue;
            std::string imu  = p[0].as<std::string>();
            std::string lidar = p[1].as<std::string>();
            pipe_cfg.imu_lidar_pairs.emplace_back(imu, lidar);
        }
    }
    // 手眼 180° 歧义策略（pipeline 用；lidar_rear 反向安装时建议 prefer_180）
    if (cfg["imu_lidar"]) {
        const auto& il = cfg["imu_lidar"];
        if (il["handeye_180_decision"]) pipe_cfg.imu_lidar_handeye_180_decision = il["handeye_180_decision"].as<std::string>();
        if (il["handeye_prefer_identity_when_ambiguous"]) pipe_cfg.imu_lidar_handeye_prefer_identity_when_ambiguous = il["handeye_prefer_identity_when_ambiguous"].as<bool>();
        if (il["handeye_180_residual_margin_deg"]) pipe_cfg.imu_lidar_handeye_180_residual_margin_deg = il["handeye_180_residual_margin_deg"].as<double>();
        // 初值：标定在初值基础上精化。支持 1) 文件路径（字符串） 2) 内联 4x4（含 rows/cols/data 的对象）
        if (il["initial_extrinsics"] && il["initial_extrinsics"].IsMap()) {
            for (const auto& kv : il["initial_extrinsics"]) {
                std::string key = kv.first.as<std::string>();
                if (kv.second.IsNull()) continue;
                std::string storage_key = (key.size() > 2 && key.substr(0, 2) == "T_") ? key.substr(2) : key;
                if (kv.second.IsScalar()) {
                    pipe_cfg.imu_lidar_initial_extrinsic_files[storage_key] = kv.second.as<std::string>();
                } else if (kv.second.IsMap() && kv.second["data"].IsSequence()) {
                    const auto& data = kv.second["data"];
                    std::vector<double> v;
                    for (size_t i = 0; i < data.size() && i < 16u; ++i) v.push_back(data[i].as<double>());
                    if (v.size() >= 16u) pipe_cfg.imu_lidar_initial_extrinsic_inline[storage_key] = std::move(v);
                }
            }
        }
    }

    // 4) IMU 内参：从 results/imu_intrinsic/ 标定结果文件读取，供手眼/B样条使用（陀螺零偏等）
    std::string results_imu_intrinsic = "imu_intrinsic";
    if (cfg["results"] && cfg["results"]["imu_intrinsic"])
        results_imu_intrinsic = cfg["results"]["imu_intrinsic"].as<std::string>();
    pipe_cfg.results_imu_intrinsic = results_imu_intrinsic;
    std::string imu_intrinsic_dir = output_dir + "/" + results_imu_intrinsic;
    if (cfg["imu_intrinsic_dir"]) {
        std::string dir = cfg["imu_intrinsic_dir"].as<std::string>();
        imu_intrinsic_dir = fs::path(dir).is_absolute()
                               ? dir
                               : (fs::path(output_dir) / dir).string();
    }
    std::set<std::string> imu_ids_used;
    if (!pipe_cfg.imu_lidar_pairs.empty()) {
        for (const auto& pr : pipe_cfg.imu_lidar_pairs)
            imu_ids_used.insert(pr.first);
    } else {
        imu_ids_used.insert(pipe_cfg.imu_sensor_id.empty() ? imu_id : pipe_cfg.imu_sensor_id);
    }
    for (const std::string& sid : imu_ids_used) {
        std::string path = imu_intrinsic_dir + "/imu_intrinsic_" + sid + ".yaml";
        pipe_cfg.imu_intrinsic_files[sid] = path;
    }
    if (!pipe_cfg.imu_intrinsic_files.empty()) {
        UNICALIB_INFO("IMU 内参: 从 results 目录读取 ({} 个)，路径示例: {}",
                      pipe_cfg.imu_intrinsic_files.size(),
                      pipe_cfg.imu_intrinsic_files.begin()->second);
    }

    CalibPipeline pipeline(pipe_cfg);
    auto report = pipeline.run();
    report.print_summary();

    UNICALIB_INFO("IMU-LiDAR 标定完成 | 结果: {}", output_dir);
    UNICALIB_MAIN_TRY_END(0)
}
