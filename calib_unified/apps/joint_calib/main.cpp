/**
 * UniCalib — 联合标定应用 (支持5种标定任务灵活组合)
 *
 * 可选标定任务:
 *   --imu-intrin          IMU 内参 (Allan方差 + Transformer-IMU粗估)
 *   --cam-intrin          相机内参 (棋盘格 + DM-Calib粗估)
 *   --imu-lidar           IMU-LiDAR 外参 (B样条 + L2Calib粗估)
 *   --lidar-cam           LiDAR-Camera 外参 (边缘对齐 + MIAS-LCEC粗估)
 *   --cam-cam             Camera-Camera 外参 (BA + 特征匹配)
 *   --all                 全部标定 (默认)
 *
 * 两阶段 + 手动校准:
 *   --coarse              启用 AI 粗标定 (所有启用的任务)
 *   --manual              精标定后允许手动校准 (外参任务)
 *   --prefer-targetfree   无目标方法优先 (默认)
 *
 * 用法:
 *   unicalib_joint --config joint_config.yaml --all --coarse
 *   unicalib_joint --config joint_config.yaml --lidar-cam --cam-cam --manual
 *   unicalib_joint --config joint_config.yaml --imu-intrin --imu-lidar --coarse
 */
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include "unicalib/solver/joint_calib_solver.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/intrinsic/camera_calib.h"
#include "unicalib/io/yaml_io.h"
#include "unicalib/common/sensor_types.h"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdlib>

namespace fs = std::filesystem;
using namespace ns_unicalib;

#define YG(node, key, def) ((node)[key] ? (node)[key].as<std::decay_t<decltype(def)>>() : (def))

// 从 unicalib_example.yaml 各子段落填充 JointCalibSolver::Config，保证每个配置参数都被读取
static void fill_solver_config_from_yaml(const YAML::Node& root,
                                         JointCalibSolver::Config& out) {
    if (root["do_imu_intrinsic"])         out.do_imu_intrinsic         = root["do_imu_intrinsic"].as<bool>();
    if (root["do_camera_intrinsic"])      out.do_camera_intrinsic      = root["do_camera_intrinsic"].as<bool>();
    if (root["do_imu_lidar_extrinsic"])   out.do_imu_lidar_extrinsic   = root["do_imu_lidar_extrinsic"].as<bool>();
    if (root["do_lidar_lidar_extrinsic"]) out.do_lidar_lidar_extrinsic = root["do_lidar_lidar_extrinsic"].as<bool>();
    if (root["do_lidar_camera_extrinsic"]) out.do_lidar_camera_extrinsic = root["do_lidar_camera_extrinsic"].as<bool>();
    if (root["do_cam_cam_extrinsic"])     out.do_cam_cam_extrinsic     = root["do_cam_cam_extrinsic"].as<bool>();
    if (root["do_joint_bspline_refine"])  out.do_joint_bspline_refine   = root["do_joint_bspline_refine"].as<bool>();
    if (root["verbose"])                  out.verbose                  = root["verbose"].as<bool>();
    if (root["output_dir"])                out.output_dir               = root["output_dir"].as<std::string>();

    if (root["imu_intrinsic"]) {
        const auto& n = root["imu_intrinsic"];
        out.imu_intrin_cfg.static_gyro_threshold   = YG(n, "static_gyro_thresh",  0.05);
        out.imu_intrin_cfg.static_detect_window   = YG(n, "static_detect_window", 0.5);
        out.imu_intrin_cfg.min_static_frames      = YG(n, "min_static_frames", 50);
        out.imu_intrin_cfg.allan_num_tau_points   = YG(n, "allan_num_tau_points", 50);
    }
    if (root["camera_intrinsic"]) {
        const auto& n = root["camera_intrinsic"];
        out.cam_intrin_cfg.min_images   = YG(n, "min_images", 15);
        out.cam_intrin_cfg.max_images   = YG(n, "max_images", 100);
        out.cam_intrin_cfg.max_rms_px   = YG(n, "max_rms_px", 1.5);
        if (n["target"]) {
            std::string tt = YG(n["target"], "type", std::string("chessboard"));
            if (tt == "circles")       out.cam_intrin_cfg.target.type = TargetConfig::Type::CIRCLES_GRID;
            else if (tt == "asym_circles") out.cam_intrin_cfg.target.type = TargetConfig::Type::ASYMMETRIC_CIRCLES;
            else                        out.cam_intrin_cfg.target.type = TargetConfig::Type::CHESSBOARD;
            out.cam_intrin_cfg.target.cols = YG(n["target"], "cols", 9);
            out.cam_intrin_cfg.target.rows = YG(n["target"], "rows", 6);
            out.cam_intrin_cfg.target.square_size_m = YG(n["target"], "square_size", 0.025);
        }
        std::string model_str = YG(n, "model", std::string("pinhole"));
        out.cam_intrin_cfg.model = (model_str == "fisheye") ? CameraIntrinsics::Model::FISHEYE : CameraIntrinsics::Model::PINHOLE;
    }
    if (root["imu_lidar"]) {
        const auto& n = root["imu_lidar"];
        out.imu_lidar_cfg.ndt_resolution    = YG(n, "ndt_resolution", 1.0);
        out.imu_lidar_cfg.ndt_max_iter      = YG(n, "ndt_max_iter", 30);
        out.imu_lidar_cfg.spline_dt_s       = YG(n, "spline_dt_s", 0.1);
        out.imu_lidar_cfg.spline_order      = YG(n, "spline_order", 4);
        out.imu_lidar_cfg.optimize_time_offset = YG(n, "optimize_time_offset", true);
        out.imu_lidar_cfg.time_offset_init_s   = YG(n, "time_offset_init", 0.0);
        out.imu_lidar_cfg.time_offset_max_s   = YG(n, "time_offset_max", 0.2);
        out.imu_lidar_cfg.min_motion_rot_deg   = YG(n, "min_motion_rot_deg", 3.0);
        out.imu_lidar_cfg.rot_pair_dt_target_s = YG(n, "rot_pair_dt_target_s", 0.2);
        out.imu_lidar_cfg.min_motion_rot_deg_short = YG(n, "min_motion_rot_deg_short", 0.5);
        out.imu_lidar_cfg.rot_pair_short_step_s    = YG(n, "rot_pair_short_step_s", 0.1);
        out.imu_lidar_cfg.handeye_fix_180_ambiguity = YG(n, "handeye_fix_180_ambiguity", true);
        out.imu_lidar_cfg.handeye_180_decision = YG(n, "handeye_180_decision", std::string("prefer_identity"));
        out.imu_lidar_cfg.handeye_prefer_identity_when_ambiguous = YG(n, "handeye_prefer_identity_when_ambiguous", true);
        out.imu_lidar_cfg.handeye_180_residual_margin_deg = YG(n, "handeye_180_residual_margin_deg", 3.0);
        out.imu_lidar_cfg.handeye_outlier_reject_quantile = YG(n, "handeye_outlier_reject_quantile", 0.0);
        out.imu_lidar_cfg.handeye_outlier_max_iter = YG(n, "handeye_outlier_max_iter", 2);
        out.imu_lidar_cfg.handeye_outlier_min_pairs = YG(n, "handeye_outlier_min_pairs", 20);
        out.imu_lidar_cfg.handeye_ransac_enable = YG(n, "handeye_ransac_enable", false);
        out.imu_lidar_cfg.handeye_ransac_inlier_thresh_deg = YG(n, "handeye_ransac_inlier_thresh_deg", 3.0);
        out.imu_lidar_cfg.handeye_ransac_max_iter = YG(n, "handeye_ransac_max_iter", 200);
    }
    if (root["lidar_camera"]) {
        const auto& n = root["lidar_camera"];
        std::string method_str = YG(n, "method", std::string("target"));
        out.lidar_cam_cfg.method = (method_str == "edge") ? LiDARCameraCalibrator::Method::EDGE_ALIGNMENT :
                                   (method_str == "motion") ? LiDARCameraCalibrator::Method::MOTION_BSPLINE :
                                   LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
        out.lidar_cam_cfg.board_cols   = YG(n, "board_cols", 9);
        out.lidar_cam_cfg.board_rows   = YG(n, "board_rows", 6);
        out.lidar_cam_cfg.square_size_m = YG(n, "square_size", 0.025);
        std::string target_type_str = YG(n, "target_type", std::string("chessboard"));
        if (target_type_str == "circles_grid")      out.lidar_cam_cfg.target_type = LiDARCameraCalibrator::TargetType::CIRCLES_GRID;
        else if (target_type_str == "asym_circles") out.lidar_cam_cfg.target_type = LiDARCameraCalibrator::TargetType::ASYMMETRIC_CIRCLES;
        else                                         out.lidar_cam_cfg.target_type = LiDARCameraCalibrator::TargetType::CHESSBOARD;
        out.lidar_cam_cfg.optimize_time_offset = YG(n, "optimize_time_offset", true);
        if (n["fine"]) {
            const auto& fine = n["fine"];
            if (fine["multi_feature"]) {
                const auto& mf = fine["multi_feature"];
                if (mf["edge_weight"])   out.lidar_cam_cfg.edge_weight   = mf["edge_weight"].as<double>();
                if (mf["corner_weight"]) out.lidar_cam_cfg.corner_weight = mf["corner_weight"].as<double>();
                if (mf["intensity_weight"]) out.lidar_cam_cfg.intensity_weight = mf["intensity_weight"].as<double>();
                if (mf["corner_max_per_frame"]) out.lidar_cam_cfg.corner_max_per_frame = mf["corner_max_per_frame"].as<int>();
            }
            if (fine["robust_loss"]) {
                const auto& rl = fine["robust_loss"];
                if (rl["use"])       out.lidar_cam_cfg.use_robust_loss = rl["use"].as<bool>();
                if (rl["type"])      out.lidar_cam_cfg.robust_loss_type = rl["type"].as<std::string>();
                if (rl["threshold"]) out.lidar_cam_cfg.robust_loss_threshold = rl["threshold"].as<double>();
            }
        }
    }
    if (root["cam_cam"]) {
        const auto& n = root["cam_cam"];
        std::string method_str = YG(n, "method", std::string("chessboard"));
        out.cam_cam_cfg.method = (method_str == "essential") ? CamCamCalibrator::Method::ESSENTIAL_MATRIX :
                                 (method_str == "ba") ? CamCamCalibrator::Method::BUNDLE_ADJUSTMENT :
                                 CamCamCalibrator::Method::CHESSBOARD_STEREO;
        out.cam_cam_cfg.target.cols = YG(n, "board_cols", 9);
        out.cam_cam_cfg.target.rows = YG(n, "board_rows", 6);
        out.cam_cam_cfg.target.square_size_m = YG(n, "square_size", 0.025);
        out.cam_cam_cfg.max_rms_px = YG(n, "max_rms_px", 2.0);
        // fix_intrinsics 对应 StereoCameraCalibrator；CamCamCalibrator 用 target，这里用 ba_optimize_intrinsics 反义
        if (n["fix_intrinsics"]) out.cam_cam_cfg.ba_optimize_intrinsics = !n["fix_intrinsics"].as<bool>();
        if (n["camera_align"] && n["camera_align"].IsSequence()) {
            for (const auto& pair_node : n["camera_align"]) {
                if (pair_node.IsSequence() && pair_node.size() >= 2)
                    out.cam_cam_camera_align.emplace_back(
                        pair_node[0].as<std::string>(), pair_node[1].as<std::string>());
            }
        }
    }
    if (root["joint_bspline"]) {
        const auto& n = root["joint_bspline"];
        out.imu_lidar_cfg.spline_order = YG(n, "spline_order", 4);
        out.imu_lidar_cfg.spline_dt_s  = YG(n, "spline_dt_s", 0.05);
        out.imu_lidar_cfg.optimize_gravity = YG(n, "optimize_gravity", true);
        if (n["max_iterations"]) out.imu_lidar_cfg.ceres_max_iter = n["max_iterations"].as<int>();
        if (n["optimize_intrinsics"]) out.cam_cam_cfg.ba_optimize_intrinsics = n["optimize_intrinsics"].as<bool>();
    }
}

// ===================================================================
// 从 YAML 读取标定对配置 (pairs)
// 支持 imu_lidar.pairs / lidar_lidar.pairs / lidar_camera.pairs / cam_cam.pairs
// ===================================================================
static CalibPairsConfig load_calib_pairs_from_yaml(const YAML::Node& root) {
    CalibPairsConfig pairs;
    
    // IMU-LiDAR pairs: [[imu_id, lidar_id], ...]
    if (root["imu_lidar"] && root["imu_lidar"]["pairs"] && root["imu_lidar"]["pairs"].IsSequence()) {
        for (const auto& p : root["imu_lidar"]["pairs"]) {
            if (p.IsSequence() && p.size() >= 2)
                pairs.imu_lidar_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
        UNICALIB_INFO("配置: 读取到 {} 对 IMU-LiDAR 标定对", pairs.imu_lidar_pairs.size());
    }
    
    // LiDAR-LiDAR pairs: [[ref_id, target_id], ...]
    if (root["lidar_lidar"] && root["lidar_lidar"]["pairs"] && root["lidar_lidar"]["pairs"].IsSequence()) {
        for (const auto& p : root["lidar_lidar"]["pairs"]) {
            if (p.IsSequence() && p.size() >= 2)
                pairs.lidar_lidar_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
        UNICALIB_INFO("配置: 读取到 {} 对 LiDAR-LiDAR 标定对", pairs.lidar_lidar_pairs.size());
    }
    
    // LiDAR-Camera pairs: [[lidar_id, cam_id], ...]
    if (root["lidar_camera"] && root["lidar_camera"]["pairs"] && root["lidar_camera"]["pairs"].IsSequence()) {
        for (const auto& p : root["lidar_camera"]["pairs"]) {
            if (p.IsSequence() && p.size() >= 2)
                pairs.lidar_camera_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
        UNICALIB_INFO("配置: 读取到 {} 对 LiDAR-Camera 标定对", pairs.lidar_camera_pairs.size());
    }
    
    return pairs;
}

// Camera-Camera pairs 向后兼容：同时支持 pairs 和 camera_align 两种命名
static void load_cam_cam_pairs_compat(const YAML::Node& root, CalibPairsConfig& pairs) {
    if (!root["cam_cam"]) return;
    
    if (root["cam_cam"]["pairs"] && root["cam_cam"]["pairs"].IsSequence()) {
        for (const auto& p : root["cam_cam"]["pairs"]) {
            if (p.IsSequence() && p.size() >= 2)
                pairs.cam_cam_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
        UNICALIB_INFO("配置: 读取到 {} 对 Camera-Camera 标定对 (from pairs)", pairs.cam_cam_pairs.size());
    } else if (root["cam_cam"]["camera_align"] && root["cam_cam"]["camera_align"].IsSequence()) {
        for (const auto& p : root["cam_cam"]["camera_align"]) {
            if (p.IsSequence() && p.size() >= 2)
                pairs.cam_cam_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
        UNICALIB_INFO("配置: 读取到 {} 对 Camera-Camera 标定对 (from camera_align)", pairs.cam_cam_pairs.size());
    }
}

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════════════╗
 ║   UniCalib v2.0 — 多传感器联合标定系统                        ║
 ║                                                               ║
 ║   支持: IMU内参 / 相机内参 / IMU-LiDAR外参 /                  ║
 ║         LiDAR-Camera外参 / Camera-Camera外参                  ║
 ║                                                               ║
 ║   两阶段: AI粗标定 → 无目标精标定 → 手动校准                  ║
 ║   AI模型: DM-Calib / MIAS-LCEC / Transformer-IMU / L2Calib   ║
 ╚═══════════════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_joint [选项]\n\n"
        "配置:\n"
        "  --config/-c <file>     YAML 配置文件 (必选)\n\n"
        "标定任务选择 (可组合, 默认 --all):\n"
        "  --all                  所有任务\n"
        "  --imu-intrin           IMU 内参标定\n"
        "  --cam-intrin           相机内参标定\n"
        "  --imu-lidar            IMU-LiDAR 外参标定\n"
        "  --lidar-cam            LiDAR-Camera 外参标定\n"
        "  --cam-cam              Camera-Camera 外参标定\n\n"
        "标定模式:\n"
        "  --coarse               启用 AI 粗标定\n"
        "  --manual               启用手动校准 (外参任务)\n"
        "  --prefer-targetfree    优先无目标方法 (默认)\n"
        "  --no-targetfree        允许使用靶标方法\n\n"
        "AI 配置:\n"
        "  --ai-root <dir>        AI 工程根目录 (默认 ../)\n"
        "  --check-ai             只检查 AI 模型可用性\n\n"
        "日志/输出:\n"
        "  --log-level <l>        trace|debug|info|warn|error\n"
        "  --output-dir <dir>     输出目录 (默认 ./results)\n"
        "  --data-dir <dir>       数据根目录 (与 config 中相对路径/ros2_bag_file 拼接；也可用 CALIB_DATA_DIR)\n\n"
        "配置文件: config/unicalib_example.yaml（全工程唯一）\n\n";
}

// 解析数据路径：相对路径与 base 拼接；占位符 /path/to/xxx 视为 base+xxx；绝对路径原样返回
static std::string resolve_data_path(const std::string& base, const std::string& path) {
    if (path.empty()) return "";
    std::string work = path;
    const char prefix[] = "/path/to/";
    if (!base.empty() && work.size() >= sizeof(prefix) - 1 &&
        work.compare(0, sizeof(prefix) - 1, prefix) == 0)
        work = work.substr(sizeof(prefix) - 1);
    if (base.empty()) return path;
    fs::path p(work);
    if (p.is_absolute()) return path;
    return (fs::path(base) / p).lexically_normal().string();
}

// 打印任务选择摘要
static void print_task_summary(CalibTaskType tasks, bool do_coarse,
                                bool do_manual, bool prefer_tf) {
    auto flag = [&](CalibTaskType t) {
        return has_task(tasks, t) ? "✓" : "✗";
    };
    std::cout << "\n  标定任务:\n";
    std::cout << "    " << flag(CalibTaskType::IMU_INTRINSIC)    << " IMU 内参\n";
    std::cout << "    " << flag(CalibTaskType::CAM_INTRINSIC)    << " 相机内参\n";
    std::cout << "    " << flag(CalibTaskType::IMU_LIDAR_EXTRIN) << " IMU-LiDAR 外参\n";
    std::cout << "    " << flag(CalibTaskType::LIDAR_CAM_EXTRIN) << " LiDAR-Camera 外参\n";
    std::cout << "    " << flag(CalibTaskType::CAM_CAM_EXTRIN)   << " Camera-Camera 外参\n";
    std::cout << "\n  选项:\n";
    std::cout << "    AI粗标定:    " << (do_coarse  ? "启用" : "禁用") << "\n";
    std::cout << "    手动校准:    " << (do_manual  ? "启用" : "禁用") << "\n";
    std::cout << "    无目标优先:  " << (prefer_tf  ? "是"   : "否")   << "\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    std::string config_file;
    std::string ai_root    = "../";
    std::string output_dir = "./results";
    std::string log_level  = "info";
    std::string data_dir;  // 数据根目录，空则用 CALIB_DATA_DIR（配合 --dataset 时脚本会设置）

    bool do_coarse   = false;
    bool do_manual   = false;
    bool prefer_tf   = true;
    bool check_ai    = false;

    // 任务标志
    bool do_all        = false;
    bool do_imu_intrin = false;
    bool do_cam_intrin = false;
    bool do_imu_lidar  = false;
    bool do_lidar_lidar = false;
    bool do_lidar_cam  = false;
    bool do_cam_cam    = false;
    bool any_task      = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--all")             { do_all = true;        any_task = true; }
        else if (a == "--imu-intrin")      { do_imu_intrin = true; any_task = true; }
        else if (a == "--cam-intrin")      { do_cam_intrin = true; any_task = true; }
        else if (a == "--imu-lidar")       { do_imu_lidar  = true; any_task = true; }
        else if (a == "--lidar-lidar")     { do_lidar_lidar = true; any_task = true; }
        else if (a == "--lidar-cam")       { do_lidar_cam  = true; any_task = true; }
        else if (a == "--cam-cam")         { do_cam_cam    = true; any_task = true; }
        else if (a == "--coarse")          do_coarse = true;
        else if (a == "--manual")          do_manual = true;
        else if (a == "--prefer-targetfree") prefer_tf = true;
        else if (a == "--no-targetfree")   prefer_tf = false;
        else if (a == "--check-ai")        check_ai  = true;
        else if (a == "--ai-root" && i+1 < argc)   ai_root = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--data-dir" && i+1 < argc)   data_dir = argv[++i];
        else if (a == "--log-level" && i+1 < argc)  log_level = argv[++i];
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    // 默认: 所有任务
    if (!any_task) do_all = true;

    // 组合任务掩码
    CalibTaskType tasks = CalibTaskType::NONE;
    if (do_all) {
        tasks = CalibTaskType::ALL;
    } else {
        if (do_imu_intrin)   tasks = tasks | CalibTaskType::IMU_INTRINSIC;
        if (do_cam_intrin)   tasks = tasks | CalibTaskType::CAM_INTRINSIC;
        if (do_imu_lidar)    tasks = tasks | CalibTaskType::IMU_LIDAR_EXTRIN;
        if (do_lidar_lidar)  tasks = tasks | CalibTaskType::LIDAR_LIDAR_EXTRIN;
        if (do_lidar_cam)    tasks = tasks | CalibTaskType::LIDAR_CAM_EXTRIN;
        if (do_cam_cam)      tasks = tasks | CalibTaskType::CAM_CAM_EXTRIN;
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    int main_exit = 0;
    UNICALIB_MAIN_TRY_BEGIN

    // ─── 初始化日志（写入 logs 目录，文件名带时间戳）───────────────────
    std::string logs_dir = resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/joint_calib_" + log_timestamp_filename() + ".log";
    Logger::init("Joint-Calib",
                 log_file,
                 log_level == "debug" ? spdlog::level::debug :
                 log_level == "trace" ? spdlog::level::trace :
                 log_level == "warn"  ? spdlog::level::warn  :
                                        spdlog::level::info);

    // ─── 加载配置 ─────────────────────────────────────────────────────
    YAML::Node cfg;
    try { cfg = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
        UNICALIB_ERROR("配置加载失败: {}", e.what()); return 1;
    }

    // 系统配置（传感器列表、reference_imu、output_dir、data.bag_file）统一由 YamlIO 读取
    SystemConfig sys_cfg;
    try {
        sys_cfg = YamlIO::load_system_config(config_file);
        if (!sys_cfg.output_dir.empty()) output_dir = sys_cfg.output_dir;
    } catch (const ns_unicalib::UniCalibException& e) {
        UNICALIB_ERROR("系统配置加载失败: {}", e.toString()); return 1;
    }

    if (cfg["output_dir"])   output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["prefer_targetfree"]) prefer_tf = cfg["prefer_targetfree"].as<bool>();
    if (cfg["ai_root"])      ai_root = cfg["ai_root"].as<std::string>();

    // 全自动标定：根据 sensors 数量自动决定执行哪些任务（话题与数量均来自配置）
    bool auto_tasks = cfg["auto_tasks"] && cfg["auto_tasks"].as<bool>();
    if (auto_tasks) {
        size_t n_imu = 0, n_lidar = 0, n_cam = 0;
        for (const auto& s : sys_cfg.sensors) {
            if (s.type == SensorType::IMU)    n_imu++;
            if (s.type == SensorType::LiDAR)  n_lidar++;
            if (s.type == SensorType::CAMERA) n_cam++;
        }
        do_imu_intrin = (n_imu >= 1);
        do_cam_intrin = (n_cam >= 1);
        do_cam_cam    = (n_cam >= 2);
        do_imu_lidar   = (n_imu >= 1 && n_lidar >= 1);
        do_lidar_lidar = (n_lidar >= 2);
        do_lidar_cam   = (n_lidar >= 1 && n_cam >= 1);
        tasks = CalibTaskType::NONE;
        if (do_imu_intrin)   tasks = tasks | CalibTaskType::IMU_INTRINSIC;
        if (do_cam_intrin)   tasks = tasks | CalibTaskType::CAM_INTRINSIC;
        if (do_imu_lidar)    tasks = tasks | CalibTaskType::IMU_LIDAR_EXTRIN;
        if (do_lidar_lidar)  tasks = tasks | CalibTaskType::LIDAR_LIDAR_EXTRIN;
        if (do_lidar_cam)    tasks = tasks | CalibTaskType::LIDAR_CAM_EXTRIN;
        if (do_cam_cam)      tasks = tasks | CalibTaskType::CAM_CAM_EXTRIN;
        UNICALIB_INFO("auto_tasks=true: 传感器 IMU={} LiDAR={} 相机={} → 已自动勾选任务", n_imu, n_lidar, n_cam);
    }
    // 若 YAML 中存在 do_* 任务开关且未开 auto_tasks，则覆盖命令行（联合标定以配置文件为准）
    bool yaml_has_do_flags = cfg["do_imu_intrinsic"] || cfg["do_camera_intrinsic"] ||
                             cfg["do_imu_lidar_extrinsic"] || cfg["do_lidar_lidar_extrinsic"] ||
                             cfg["do_lidar_camera_extrinsic"] || cfg["do_cam_cam_extrinsic"];
    if (!auto_tasks && yaml_has_do_flags) {
        if (cfg["do_imu_intrinsic"])         do_imu_intrin = cfg["do_imu_intrinsic"].as<bool>();
        if (cfg["do_camera_intrinsic"])       do_cam_intrin = cfg["do_camera_intrinsic"].as<bool>();
        if (cfg["do_imu_lidar_extrinsic"])   do_imu_lidar  = cfg["do_imu_lidar_extrinsic"].as<bool>();
        if (cfg["do_lidar_lidar_extrinsic"]) do_lidar_lidar = cfg["do_lidar_lidar_extrinsic"].as<bool>();
        if (cfg["do_lidar_camera_extrinsic"]) do_lidar_cam  = cfg["do_lidar_camera_extrinsic"].as<bool>();
        if (cfg["do_cam_cam_extrinsic"])     do_cam_cam    = cfg["do_cam_cam_extrinsic"].as<bool>();
        tasks = CalibTaskType::NONE;
        if (do_imu_intrin)   tasks = tasks | CalibTaskType::IMU_INTRINSIC;
        if (do_cam_intrin)   tasks = tasks | CalibTaskType::CAM_INTRINSIC;
        if (do_imu_lidar)    tasks = tasks | CalibTaskType::IMU_LIDAR_EXTRIN;
        if (do_lidar_lidar)  tasks = tasks | CalibTaskType::LIDAR_LIDAR_EXTRIN;
        if (do_lidar_cam)    tasks = tasks | CalibTaskType::LIDAR_CAM_EXTRIN;
        if (do_cam_cam)      tasks = tasks | CalibTaskType::CAM_CAM_EXTRIN;
    }
    if (cfg["verbose"]) log_level = cfg["verbose"].as<bool>() ? "debug" : log_level;

    // 从 YAML 各子段落填充求解器配置，保证 unicalib_example.yaml 中每个参数都被读取
    JointCalibSolver::Config solver_cfg;
    fill_solver_config_from_yaml(cfg, solver_cfg);
    (void)solver_cfg;  // 供后续 pipeline 与 solver 对接时使用

    print_task_summary(tasks, do_coarse, do_manual, prefer_tf);

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置: {}", config_file);
    UNICALIB_INFO("输出: {}", output_dir);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─── 检查 AI 可用性 (--check-ai) ─────────────────────────────────
    if (check_ai || do_coarse) {
        UNICALIB_INFO("▶ 检查 AI 模型可用性...");
        AICoarseCalibManager::Config ai_cfg;
        ai_cfg.ai_root = ai_root;
        if (cfg["python_exe"]) ai_cfg.python_exe = cfg["python_exe"].as<std::string>();
        AICoarseCalibManager ai_mgr(ai_cfg);
        ai_mgr.print_availability();
        if (check_ai) return 0;
    }

    // ─── 构建流水线 ───────────────────────────────────────────────────
    PipelineConfig pipe_cfg;
    
    // 读取标定对配置
    CalibPairsConfig calib_pairs = load_calib_pairs_from_yaml(cfg);
    load_cam_cam_pairs_compat(cfg, calib_pairs);
    
    // 传递 pairs 到 pipe_cfg
    pipe_cfg.imu_lidar_pairs = calib_pairs.imu_lidar_pairs;
    if (cfg["imu_lidar"]) {
        const auto& il = cfg["imu_lidar"];
        if (il["handeye_180_decision"]) pipe_cfg.imu_lidar_handeye_180_decision = il["handeye_180_decision"].as<std::string>();
        if (il["handeye_prefer_identity_when_ambiguous"]) pipe_cfg.imu_lidar_handeye_prefer_identity_when_ambiguous = il["handeye_prefer_identity_when_ambiguous"].as<bool>();
        if (il["handeye_180_residual_margin_deg"]) pipe_cfg.imu_lidar_handeye_180_residual_margin_deg = il["handeye_180_residual_margin_deg"].as<double>();
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
    pipe_cfg.lidar_lidar_pairs = calib_pairs.lidar_lidar_pairs;
    pipe_cfg.lidar_camera_pairs = calib_pairs.lidar_camera_pairs;
    pipe_cfg.cam_cam_pairs = calib_pairs.cam_cam_pairs;
    if (cfg["lidar_camera"]) {
        const auto& n = cfg["lidar_camera"];
        pipe_cfg.board_cols   = YG(n, "board_cols", 9);
        pipe_cfg.board_rows   = YG(n, "board_rows", 6);
        pipe_cfg.square_size_m = YG(n, "square_size", 0.025);
        if (n["square_size_m"]) pipe_cfg.square_size_m = n["square_size_m"].as<double>();
        pipe_cfg.lidar_cam_target_type = YG(n, "target_type", std::string("chessboard"));
        if (n["method"]) pipe_cfg.lidar_cam_method = n["method"].as<std::string>();
        if (n["target_min_corners_per_frame"]) pipe_cfg.target_min_corners_per_frame = n["target_min_corners_per_frame"].as<int>();
        if (n["target_min_frames"]) pipe_cfg.target_min_frames = n["target_min_frames"].as<int>();
        if (n["target_use_coarse_rotation_search"]) pipe_cfg.target_use_coarse_rotation_search = n["target_use_coarse_rotation_search"].as<bool>();
        if (n["target_coarse_rotation_range_deg"]) pipe_cfg.target_coarse_rotation_range_deg = n["target_coarse_rotation_range_deg"].as<double>();
        if (n["target_coarse_rotation_step_deg"]) pipe_cfg.target_coarse_rotation_step_deg = n["target_coarse_rotation_step_deg"].as<double>();
        if (n["target_coarse_inlier_threshold_px"]) pipe_cfg.target_coarse_inlier_threshold_px = n["target_coarse_inlier_threshold_px"].as<double>();
        if (n["target_per_frame_rms_threshold_px"]) pipe_cfg.target_per_frame_rms_threshold_px = n["target_per_frame_rms_threshold_px"].as<double>();
        if (n["target_ba_max_iter"]) pipe_cfg.target_ba_max_iter = n["target_ba_max_iter"].as<int>();
        if (n["target_ba_use_robust_loss"]) pipe_cfg.target_ba_use_robust_loss = n["target_ba_use_robust_loss"].as<bool>();
        if (n["target_ba_huber_scale_px"]) pipe_cfg.target_ba_huber_scale_px = n["target_ba_huber_scale_px"].as<double>();
    }
    // 传递 reference_imu
    if (!sys_cfg.reference_imu.empty()) {
        pipe_cfg.reference_imu = sys_cfg.reference_imu;
    }
    
    // Third party / AI 配置 (dm_calib, mias_lcec 等)
    if (cfg["third_party"]) {
        const auto& tp = cfg["third_party"];
        if (tp["dm_calib_model"])       pipe_cfg.dm_calib_model = tp["dm_calib_model"].as<std::string>();
        if (tp["dm_calib_use_cpu"])     pipe_cfg.dm_calib_use_cpu = tp["dm_calib_use_cpu"].as<bool>();
        if (tp["dm_calib_timeout_sec"]) pipe_cfg.dm_calib_timeout_sec = tp["dm_calib_timeout_sec"].as<int>();
        if (tp["dm_calib_denoise_steps"]) pipe_cfg.dm_calib_denoise_steps = tp["dm_calib_denoise_steps"].as<int>();
        if (tp["dm_calib_ensemble_size"]) pipe_cfg.dm_calib_ensemble_size = tp["dm_calib_ensemble_size"].as<int>();
        if (tp["dm_calib_processing_res"]) pipe_cfg.dm_calib_processing_res = tp["dm_calib_processing_res"].as<int>();
        if (tp["dm_calib_max_images"]) pipe_cfg.dm_calib_max_images = tp["dm_calib_max_images"].as<int>();
        if (tp["mias_lcec"] && tp["mias_lcec"].IsMap()) {
            const auto& mias = tp["mias_lcec"];
            if (mias["repo_dir"])      pipe_cfg.mias_lcec_repo_dir    = mias["repo_dir"].as<std::string>();
            if (mias["calib_script"])  pipe_cfg.mias_lcec_calib_script = mias["calib_script"].as<std::string>();
            if (mias["model_path"])    pipe_cfg.mias_lcec_model_path  = mias["model_path"].as<std::string>();
            if (mias["timeout_sec"])   pipe_cfg.mias_lcec_timeout_sec = mias["timeout_sec"].as<int>();
            if (mias["work_dir"])      pipe_cfg.mias_lcec_work_dir    = mias["work_dir"].as<std::string>();
            if (mias["allow_pnp_fallback"]) pipe_cfg.mias_lcec_allow_pnp_fallback = mias["allow_pnp_fallback"].as<bool>();
            if (mias["python_exe"])    pipe_cfg.mias_lcec_python_exe  = mias["python_exe"].as<std::string>();
        }
    }
    
    pipe_cfg.tasks               = tasks;
    pipe_cfg.prefer_targetfree   = prefer_tf;
    pipe_cfg.allow_manual_fallback = do_manual;
    pipe_cfg.output_dir          = output_dir;
    pipe_cfg.log_level           = log_level;
    pipe_cfg.ai_models_root      = ai_root;

    // ─── LiDAR-Camera 粗标定(C3M) 与质量评估阈值（若存在） ───────────────
    if (cfg["lidar_camera"]) {
        const auto& lc = cfg["lidar_camera"];
        if (lc["coarse"]) {
            const auto& coarse = lc["coarse"];
            if (coarse["c3m_use_iterative_refine"])
                pipe_cfg.lidar_cam_c3m_use_iterative_refine = coarse["c3m_use_iterative_refine"].as<bool>();
            if (coarse["max_iterations"])
                pipe_cfg.lidar_cam_c3m_iter_max = coarse["max_iterations"].as<int>();
            if (coarse["convergence_threshold"])
                pipe_cfg.lidar_cam_c3m_iter_thresh = coarse["convergence_threshold"].as<double>();
            if (coarse["initial_guess"] && coarse["initial_guess"]["feature_match_threshold"])
                pipe_cfg.lidar_cam_c3m_similarity_threshold = coarse["initial_guess"]["feature_match_threshold"].as<double>();
        }
        if (lc["quality"]) {
            const auto& q = lc["quality"];
            if (q["ncc_good"])          pipe_cfg.lidar_cam_quality_ncc_good = q["ncc_good"].as<double>();
            if (q["ncc_acceptable"])    pipe_cfg.lidar_cam_quality_ncc_acceptable = q["ncc_acceptable"].as<double>();
            if (q["rms_good_px"])       pipe_cfg.lidar_cam_quality_rms_good_px = q["rms_good_px"].as<double>();
            if (q["rms_acceptable_px"]) pipe_cfg.lidar_cam_quality_rms_acceptable_px = q["rms_acceptable_px"].as<double>();
            if (q["inlier_ratio_good"]) pipe_cfg.lidar_cam_quality_inlier_ratio_good = q["inlier_ratio_good"].as<double>();
            if (q["inlier_ratio_acceptable"]) pipe_cfg.lidar_cam_quality_inlier_ratio_acceptable = q["inlier_ratio_acceptable"].as<double>();
        }
    }

    // 根据命令行 --coarse 配置各任务的粗标定开关
    pipe_cfg.enable_coarse_imu_intrin = do_coarse && has_task(tasks, CalibTaskType::IMU_INTRINSIC);
    pipe_cfg.enable_coarse_cam_intrin = do_coarse && has_task(tasks, CalibTaskType::CAM_INTRINSIC);
    pipe_cfg.enable_coarse_imu_lidar  = do_coarse && has_task(tasks, CalibTaskType::IMU_LIDAR_EXTRIN);
    pipe_cfg.enable_coarse_lidar_cam  = do_coarse && has_task(tasks, CalibTaskType::LIDAR_CAM_EXTRIN);
    pipe_cfg.enable_coarse_cam_cam    = do_coarse && has_task(tasks, CalibTaskType::CAM_CAM_EXTRIN);

    // 配置文件覆盖阈值
    if (cfg["lidar_cam_rms_threshold"])
        pipe_cfg.lidar_cam_rms_threshold = cfg["lidar_cam_rms_threshold"].as<double>();
    if (cfg["cam_cam_rms_threshold"])
        pipe_cfg.cam_cam_rms_threshold   = cfg["cam_cam_rms_threshold"].as<double>();
    if (cfg["use_cam_cam_initial_from_params"])
        pipe_cfg.use_cam_cam_initial_from_params = cfg["use_cam_cam_initial_from_params"].as<bool>();
    if (cfg["use_cam_cam_skip_coarse_when_initial"])
        pipe_cfg.use_cam_cam_skip_coarse_when_initial = cfg["use_cam_cam_skip_coarse_when_initial"].as<bool>();
    if (cfg["imu_lidar_rot_threshold"])
        pipe_cfg.imu_lidar_rot_threshold = cfg["imu_lidar_rot_threshold"].as<double>();

    // ─── 数据根目录（用于解析相对路径的 ros2_bag_file / data 路径）────────────────
    std::string base_data_dir = data_dir.empty() ? (std::getenv("CALIB_DATA_DIR") ? std::getenv("CALIB_DATA_DIR") : "") : data_dir;

    // ─── ROS2 / 传感器话题：默认从配置文件 sensors 与 ros2 段读取 ───
    const YAML::Node ros2_node = cfg["ros2"];
    if (ros2_node) {
        if (ros2_node["use_ros2_bag"])   pipe_cfg.use_ros2_bag   = ros2_node["use_ros2_bag"].as<bool>();
        if (ros2_node["use_ros2_topics"]) pipe_cfg.use_ros2_topics = ros2_node["use_ros2_topics"].as<bool>();
        if (ros2_node["ros2_bag_file"])   pipe_cfg.ros2_bag_file = ros2_node["ros2_bag_file"].as<std::string>();
        if (ros2_node["max_wait_time"])  pipe_cfg.ros2_max_wait_time = ros2_node["max_wait_time"].as<double>();
        if (ros2_node["sample_interval"]) pipe_cfg.ros2_sample_interval = ros2_node["sample_interval"].as<double>();
        if (ros2_node["max_frames"])     pipe_cfg.ros2_max_frames = ros2_node["max_frames"].as<size_t>();
        if (ros2_node["lidar_topic"])     pipe_cfg.lidar_ros2_topic  = ros2_node["lidar_topic"].as<std::string>();
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
        if (ros2_node["imu_topic"])      pipe_cfg.imu_ros2_topic    = ros2_node["imu_topic"].as<std::string>();
        if (ros2_node["strict_topic_match"]) pipe_cfg.ros2_strict_topic_match = ros2_node["strict_topic_match"].as<bool>();
    }
    if (pipe_cfg.ros2_bag_file.empty() && cfg["data"] && cfg["data"]["bag_file"])
        pipe_cfg.ros2_bag_file = cfg["data"]["bag_file"].as<std::string>();
    if (pipe_cfg.use_ros2_bag && !pipe_cfg.ros2_bag_file.empty() && !base_data_dir.empty()) {
        fs::path p(pipe_cfg.ros2_bag_file);
        if (!p.is_absolute())
            pipe_cfg.ros2_bag_file = resolve_data_path(base_data_dir, pipe_cfg.ros2_bag_file);
    }
    // NEW_FORMAT：启用后优先于 ROS2
    const YAML::Node new_format_node = cfg["new_format"];
    if (new_format_node && new_format_node["enable"] && new_format_node["enable"].as<bool>()) {
        pipe_cfg.use_new_format = true;
        if (new_format_node["root_dir"])
            pipe_cfg.new_format_root_dir = new_format_node["root_dir"].as<std::string>();
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
        UNICALIB_INFO("[Joint] 数据源: NEW_FORMAT root={} unit={}",
                      pipe_cfg.new_format_root_dir.empty() ? "(未设置)" : pipe_cfg.new_format_root_dir,
                      pipe_cfg.new_format_timestamp_unit);
        if (!pipe_cfg.new_format_root_dir.empty() && !base_data_dir.empty()) {
            fs::path pr(pipe_cfg.new_format_root_dir);
            if (!pr.is_absolute())
                pipe_cfg.new_format_root_dir = resolve_data_path(base_data_dir, pipe_cfg.new_format_root_dir);
        }
    }
    // 传感器话题与数量全可配置：从 sensors 按类型填入 lidar_topics / camera_topics / imu_topics
    // 若 ros2.camera_topic 已以数组形式填入 camera_topics，则不再用 sensors 覆盖
    for (const auto& s : sys_cfg.sensors) {
        if (s.type == SensorType::LiDAR && !s.topic.empty()) {
            pipe_cfg.lidar_topics[s.sensor_id] = s.topic;
            if (pipe_cfg.lidar_ros2_topic.empty()) {
                pipe_cfg.lidar_ros2_topic = s.topic;
                pipe_cfg.lidar_id = s.sensor_id;
            }
        }
        if (s.type == SensorType::CAMERA && !s.topic.empty()) {
            if (pipe_cfg.camera_topics.empty()) {
                pipe_cfg.camera_topics[s.sensor_id] = s.topic;
                if (pipe_cfg.camera_ros2_topic.empty()) {
                    pipe_cfg.camera_ros2_topic = s.topic;
                    pipe_cfg.camera_id = s.sensor_id;
                }
            }
        }
        if (s.type == SensorType::IMU && !s.topic.empty()) {
            pipe_cfg.imu_topics[s.sensor_id] = s.topic;
            if (pipe_cfg.imu_ros2_topic.empty()) {
                pipe_cfg.imu_ros2_topic = s.topic;
            }
        }
    }
    for (const auto& s : sys_cfg.sensors) {
        if (s.type == SensorType::LiDAR && pipe_cfg.lidar_id == "lidar_front") pipe_cfg.lidar_id = s.sensor_id;
        if (s.type == SensorType::CAMERA && pipe_cfg.camera_id == "cam_left")  pipe_cfg.camera_id = s.sensor_id;
    }
    // 多相机内参文件：从 data.camera 的 intrinsic_yaml 填入；未填则从 results/camera_intrinsic/ 读取
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
                pipe_cfg.camera_intrinsic_files[cam_id] = it.second["intrinsic_yaml"].as<std::string>();
            } else {
                std::string default_path = output_dir + "/" + results_camera_intrinsic + "/camera_intrinsic_" + cam_id + ".yaml";
                pipe_cfg.camera_intrinsic_files[cam_id] = default_path;
            }
        }
    }
    if (cfg["lidar_camera"]) {
        const auto& lc = cfg["lidar_camera"];
        if (lc["optimize_time_offset"])
            pipe_cfg.lidar_cam_optimize_time_offset = lc["optimize_time_offset"].as<bool>();
        if (lc["camera_list"] && lc["camera_list"].IsSequence()) {
            for (const auto& v : lc["camera_list"])
                pipe_cfg.lidar_camera_camera_list.push_back(v.as<std::string>());
        }
        if (lc["fine"]) {
            const auto& fine = lc["fine"];
            if (fine["multi_feature"]) {
                const auto& mf = fine["multi_feature"];
                if (mf["edge_weight"])   pipe_cfg.lidar_cam_edge_weight   = mf["edge_weight"].as<double>();
                if (mf["corner_weight"]) pipe_cfg.lidar_cam_corner_weight = mf["corner_weight"].as<double>();
                if (mf["intensity_weight"]) pipe_cfg.lidar_cam_intensity_weight = mf["intensity_weight"].as<double>();
                if (mf["corner_max_per_frame"]) pipe_cfg.lidar_cam_corner_max_per_frame = mf["corner_max_per_frame"].as<int>();
            }
            if (fine["robust_loss"]) {
                const auto& rl = fine["robust_loss"];
                if (rl["use"])       pipe_cfg.lidar_cam_use_robust_loss = rl["use"].as<bool>();
                if (rl["type"])      pipe_cfg.lidar_cam_robust_loss_type = rl["type"].as<std::string>();
                if (rl["threshold"]) pipe_cfg.lidar_cam_robust_loss_threshold = rl["threshold"].as<double>();
            }
        }
        // 粗标定初值：initial_extrinsic 或 T_cam_lidar（4×4），作为粗标定输入，粗标定结果再作精标定初值
        auto project_to_so3 = [](const Eigen::Matrix3d& M) {
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
            if (R.determinant() < 0) R.col(2) *= -1;
            return R;
        };
        auto parse_4x4_se3 = [&project_to_so3](const YAML::Node& node) -> std::optional<Sophus::SE3d> {
            if (!node || !node["rows"] || !node["cols"] || !node["data"]) return std::nullopt;
            if (node["rows"].as<int>() != 4 || node["cols"].as<int>() != 4 || !node["data"].IsSequence() || node["data"].size() < 16) return std::nullopt;
            const auto& d = node["data"];
            Eigen::Matrix4d T;
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) T(i, j) = d[i * 4 + j].as<double>();
            Eigen::Matrix3d R = project_to_so3(T.block<3,3>(0,0));
            return Sophus::SE3d(Sophus::SO3d(R), T.block<3,1>(0,3));
        };
        if (lc["initial_extrinsic"]) pipe_cfg.lidar_cam_coarse_initial = parse_4x4_se3(lc["initial_extrinsic"]);
        if (!pipe_cfg.lidar_cam_coarse_initial.has_value() && lc["T_cam_lidar"]) pipe_cfg.lidar_cam_coarse_initial = parse_4x4_se3(lc["T_cam_lidar"]);
        if (lc["use_config_extrinsic_only"])
            pipe_cfg.lidar_cam_use_config_extrinsic_only = lc["use_config_extrinsic_only"].as<bool>();
        // 与 lidar_camera_extrin 一致：按 lidar__camera 键加载初值，供多相机 fine 阶段使用
        if (lc["initial_extrinsics"] && lc["initial_extrinsics"].IsMap()) {
            auto parse_4x4_flat = [](const YAML::Node& node) -> std::vector<double> {
                if (!node || !node["rows"] || !node["cols"] || !node["data"] || !node["data"].IsSequence())
                    return {};
                if (node["rows"].as<int>() != 4 || node["cols"].as<int>() != 4) return {};
                const auto& data = node["data"];
                if (data.size() < 16) return {};
                std::vector<double> v;
                for (size_t i = 0; i < 16; ++i) v.push_back(data[i].as<double>());
                return v;
            };
            for (const auto& kv : lc["initial_extrinsics"]) {
                const std::string key = kv.first.as<std::string>();
                if (!kv.second || !kv.second.IsMap()) continue;
                std::vector<double> v = parse_4x4_flat(kv.second);
                if (v.size() >= 16u)
                    pipe_cfg.lidar_camera_initial_extrinsic_inline[key] = std::move(v);
                else
                    UNICALIB_WARN("[Joint/LiDAR-Cam] initial_extrinsics['{}'] 无效，已忽略", key);
            }
            if (!pipe_cfg.lidar_camera_initial_extrinsic_inline.empty())
                UNICALIB_INFO("[Joint] lidar_camera.initial_extrinsics 已加载 {} 组",
                              pipe_cfg.lidar_camera_initial_extrinsic_inline.size());
        }
    }
    // cam_cam_pairs 已由 load_calib_pairs_from_yaml + load_cam_cam_pairs_compat 填充，此处不再重复解析
    // 多 IMU 内参文件：未做 IMU 内参时从 results/imu_intrinsic/ 读取，供 IMU-LiDAR 等外参使用
    std::string results_imu_intrinsic = "imu_intrinsic";
    if (cfg["results"] && cfg["results"]["imu_intrinsic"])
        results_imu_intrinsic = cfg["results"]["imu_intrinsic"].as<std::string>();
    pipe_cfg.results_imu_intrinsic = results_imu_intrinsic;
    for (const auto& s : sys_cfg.sensors) {
        if (s.type == SensorType::IMU) {
            std::string path = output_dir + "/" + results_imu_intrinsic + "/imu_intrinsic_" + s.sensor_id + ".yaml";
            pipe_cfg.imu_intrinsic_files[s.sensor_id] = path;
        }
    }
    // IMU 内参 Allan / NEW_FORMAT：与 reference_imu 对齐，否则取 sensors 中第一个 IMU
    if (!pipe_cfg.reference_imu.empty())
        pipe_cfg.imu_sensor_id = pipe_cfg.reference_imu;
    else {
        for (const auto& s : sys_cfg.sensors) {
            if (s.type == SensorType::IMU) {
                pipe_cfg.imu_sensor_id = s.sensor_id;
                break;
            }
        }
    }

    // 与 cam-intrin 一致：Docker 内 NumPy 2.x 时由 run 脚本设置 UNICALIB_MIAS_LCEC_PYTHON，粗标定子进程使用该解释器
    const char* mias_py = std::getenv("UNICALIB_MIAS_LCEC_PYTHON");
    if (mias_py && mias_py[0]) pipe_cfg.mias_lcec_python_exe = mias_py;

    CalibPipeline pipeline(pipe_cfg);

    // 设置进度回调
    pipeline.set_progress_callback([](CalibStage stage,
                                       const std::string& step,
                                       double progress) {
        if (progress >= 0) {
            UNICALIB_INFO("  [{}-{}] {:.1f}%",
                          stage_name(stage), step, progress * 100.0);
        }
    });

    // ─── 运行流水线 ───────────────────────────────────────────────────
    UNICALIB_INFO("▶ 开始联合标定流水线...");
    PipelineReport report;
    try {
        report = pipeline.run();
    } catch (const ns_unicalib::UniCalibException& e) {
        std::cerr << "[UniCalib 异常] " << e.toString() << "\n";
        UNICALIB_ERROR("流水线异常: {}", e.toString());
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "[标准异常] " << e.what() << "\n";
        UNICALIB_ERROR("流水线异常: {}", e.what());
        return 1;
    }

    // ─── 打印最终摘要 ─────────────────────────────────────────────────
    std::cout << "\n";
    std::cout << "  ┌─────────────────────────────────────────────────┐\n";
    std::cout << "  │            UniCalib 联合标定完成                 │\n";
    std::cout << "  ├─────────────────────────────────────────────────┤\n";
    std::cout << "  │ Pipeline ID: " << std::left << std::setw(36)
              << report.pipeline_id << "│\n";
    std::cout << "  │ 总耗时: " << std::fixed << std::setprecision(0) << std::setw(10)
              << report.total_elapsed_ms() << " ms"
              << std::setw(29) << " " << "│\n";
    std::cout << "  │ 全部收敛: " << std::setw(38)
              << (report.all_converged() ? "是 ✓" : "否 ✗ (查看日志)") << "│\n";

    for (const auto& r : report.stage_results) {
        std::string status = r.success ? "✓" : "✗";
        if (r.needs_manual_refine()) status += " (建议手动)";
        UNICALIB_INFO("  [{}] rms={:.4f} time={:.0f}ms: {} {}",
                      stage_name(r.stage), r.residual_rms, r.elapsed_ms,
                      status, r.message);
    }

    std::cout << "  ├─────────────────────────────────────────────────┤\n";
    std::cout << "  │ 结果目录: " << std::left << std::setw(38) << output_dir << "│\n";
    std::cout << "  └─────────────────────────────────────────────────┘\n\n";

    // ─── 手动校准提示 ─────────────────────────────────────────────────
    bool any_needs_manual = false;
    for (const auto& r : report.stage_results) {
        if (r.needs_manual_refine()) { any_needs_manual = true; break; }
    }

    if (any_needs_manual) {
        UNICALIB_WARN("⚠ 部分标定精度未达标, 建议手动校准:");
        UNICALIB_WARN("  重新运行并添加 --manual 标志:");
        UNICALIB_WARN("  unicalib_joint --config {} --lidar-cam --cam-cam --manual",
                      config_file);
        UNICALIB_WARN("  或直接调用 ManualCalibSession::run_*() API");
    }

    main_exit = report.all_converged() ? 0 : 1;
    UNICALIB_MAIN_TRY_END(main_exit)
}
