/**
 * UniCalib — LiDAR-Camera 外参标定应用
 *
 * 两阶段流程:
 *   Stage 1 粗标定: MIAS-LCEC AI 模型 (可选, --coarse)
 *   Stage 2 精标定: 无目标边缘对齐 (默认) / 棋盘格 / B样条运动法
 *   手动校准:       --manual 标志触发 6-DOF 交互式调整
 *
 * 用法:
 *   unicalib_lidar_camera --config <config.yaml>
 *   unicalib_lidar_camera --config <config.yaml> --data-dir /path/to/data
 *   unicalib_lidar_camera --config <config.yaml> --coarse --ai-root /path/to/ai
 *   unicalib_lidar_camera --config <config.yaml> --manual
 *   unicalib_lidar_camera --config <config.yaml> --method edge|target|motion
 *
 * 数据路径: 支持两种配置方式
 *   1) 统一格式 (推荐): config 中 data.lidar.<id> / data.camera.<id>.images_dir，配合 CALIB_DATA_DIR 或 --data-dir 作为基准路径
 *   2) 扁平键: config 中 lidar_data_dir / camera_images_dir (可为相对上述基准路径)
 */
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/calib_stage.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/io/yaml_io.h"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <algorithm>
#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
#include "unicalib/io/ros2_data_source.h"
#endif
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>
#include <csignal>
#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace fs = std::filesystem;
using namespace ns_unicalib;

// 将 3x3 投影到最近的正交旋转，避免 Sophus::SO3d(R) 因数值误差断言失败 (SIGABRT)
static Eigen::Matrix3d project_to_so3(const Eigen::Matrix3d& M) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) R.col(2) *= -1;
    return R;
}

// 从 YAML 节点（含 rows/cols/data 的 4×4 矩阵，行优先）解析为 lidar→camera 外参 SE3
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

// initial_extrinsics Map 项：rows/cols/data 行优先 16 个数
static std::vector<double> parse_4x4_matrix_data(const YAML::Node& node) {
    if (!node || !node["rows"] || !node["cols"] || !node["data"] || !node["data"].IsSequence())
        return {};
    if (node["rows"].as<int>() != 4 || node["cols"].as<int>() != 4)
        return {};
    const auto& data = node["data"];
    if (data.size() < 16)
        return {};
    std::vector<double> v;
    v.reserve(16);
    for (size_t i = 0; i < 16; ++i)
        v.push_back(data[i].as<double>());
    return v;
}

#if !defined(_WIN32)
// 信号处理：崩溃时打出当前阶段到 stderr（仅使用 async-signal-safe 调用），便于从日志/终端定位
static void fatal_signal_handler(int sig) {
    const char* sname = (sig == SIGSEGV) ? "SIGSEGV" : (sig == SIGABRT) ? "SIGABRT" : "SIG???";
    const char* prefix = "FATAL: 收到 ";
    const char* mid = " 当前阶段=";
    const char* nl = "\n";
    for (const char* p = prefix; *p; ++p) (void)write(STDERR_FILENO, p, 1);
    for (const char* p = sname; *p; ++p) (void)write(STDERR_FILENO, p, 1);
    for (const char* p = mid; *p; ++p) (void)write(STDERR_FILENO, p, 1);
    const char* stage = ns_unicalib::g_current_calib_stage;
    if (stage) { for (; *stage; ++stage) (void)write(STDERR_FILENO, stage, 1); }
    else { const char* n = "(null)"; for (; *n; ++n) (void)write(STDERR_FILENO, n, 1); }
    for (const char* p = nl; *p; ++p) (void)write(STDERR_FILENO, p, 1);
    // 阶段含义提示（便于从日志看出崩溃点）
    if (stage) {
        const char* hint = "(若为 viz_pc_* 则崩溃在点云渲染 Bind/Draw/Unbind，请查上一行日志)\n";
        for (const char* p = hint; *p; ++p) (void)write(STDERR_FILENO, p, 1);
    }
    _exit(1);
}
#endif

// 解析数据路径：若 path 为相对路径则与 base 拼接；若为占位符 /path/to/... 则视为相对 base 的路径；否则绝对路径直接返回
static std::string resolve_data_path(const std::string& base, const std::string& path) {
    if (path.empty()) return "";
    std::string work = path;
    // 占位符绝对路径：/path/to/xxx -> 视为 base + xxx，便于 --dataset 与示例配置配合
    const char prefix[] = "/path/to/";
    if (base.size() > 0 && work.size() > sizeof(prefix) - 1 &&
        work.compare(0, sizeof(prefix) - 1, prefix) == 0) {
        work = work.substr(sizeof(prefix) - 1);
    }
    if (base.empty()) return path;  // 未改过则原样返回
    fs::path p(work);
    if (p.is_absolute()) return path;
    fs::path b(base);
    return (b / p).lexically_normal().string();
}

static void resolve_path_map(const std::string& base, std::map<std::string, std::string>& paths) {
    if (base.empty()) return;
    for (auto& [k, v] : paths) {
        if (!v.empty()) v = resolve_data_path(base, v);
    }
}

static void resolve_pipeline_data_paths(PipelineConfig& pipe_cfg, const std::string& base) {
    if (base.empty()) return;
    auto resolve_one = [&](std::string& p) {
        if (!p.empty()) p = resolve_data_path(base, p);
    };
    resolve_one(pipe_cfg.lidar_data_dir);
    resolve_one(pipe_cfg.camera_images_dir);
    resolve_path_map(base, pipe_cfg.lidar_data_paths);
    resolve_path_map(base, pipe_cfg.camera_images_dirs);
    resolve_path_map(base, pipe_cfg.camera_intrinsic_files);
    resolve_path_map(base, pipe_cfg.imu_data_paths);
    if (!pipe_cfg.lidar_id.empty()) {
        auto it = pipe_cfg.lidar_data_paths.find(pipe_cfg.lidar_id);
        if (it != pipe_cfg.lidar_data_paths.end() && !it->second.empty())
            pipe_cfg.lidar_data_dir = it->second;
    }
}

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — LiDAR-Camera 外参标定                    ║
 ║   两阶段: AI粗标定(MIAS-LCEC) → 无目标边缘精标定      ║
 ║   手动校准: --manual 触发 6-DOF 交互式调整             ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_lidar_camera [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --coarse                启用 AI 粗标定 (MIAS-LCEC)\n"
        "  --ai-root <dir>         AI 工程根目录 (默认: ../)\n"
        "  --manual                精标定后启用手动校准（点云叠加窗）\n"
        "  --pangolin-panel        手动校准时使用 Pangolin 点云叠加+6-DOF 面板\n"
        "  --method <m>            精标定方法: edge(默认)|target|motion\n"
        "  --no-viz                禁用可视化界面 (默认启用)\n"
        "  --no-targetfree         禁用无目标优先 (允许使用棋盘格)\n"
        "  --log-level <l>         日志级别: trace|debug|info|warn|error\n"
        "  --output-dir <dir>      输出目录 (默认: ./results)\n"
        "  --data-dir <dir>        数据根目录 (与 config 中相对路径拼接；也可用环境变量 CALIB_DATA_DIR)\n"
        "  --help/-h               显示此帮助\n\n"
        "配置文件: 支持统一格式 data.lidar.<id>/data.camera.<id>.images_dir 或扁平键 lidar_data_dir/camera_images_dir\n"
        "  lidar_data_dir:         PCD 文件目录\n"
        "  camera_images_dir:      图像目录\n"
        "  camera_intrinsic_file:  相机内参 YAML\n"
        "  method:                 edge|target|motion\n"
        "  board_cols/rows:        棋盘格尺寸 (target 方法)\n"
        "  square_size_m:          棋盘格格子大小 [m]\n"
        "  edge_canny_low/high:    Canny 边缘检测阈值\n"
        "  coarse_rms_threshold:   粗标定通过阈值 [px]\n"
        "  fine_rms_threshold:     精标定通过阈值 [px]\n"
        "  manual_rms_threshold:   触发手动校准的阈值 [px] (默认 2.0)\n\n";
}

int main(int argc, char** argv) {
    print_banner();

#if !defined(_WIN32)
    std::signal(SIGSEGV, fatal_signal_handler);
    std::signal(SIGABRT, fatal_signal_handler);
#endif

    // ─────────────────────────────────────────────────────────────────
    // 解析命令行
    // ─────────────────────────────────────────────────────────────────
    std::string config_file, method_str = "edge";
    std::string ai_root = "../";
    std::string output_dir = "./results";
    std::string log_level  = "info";
    std::string data_dir;   // 数据根目录，空则用环境变量 CALIB_DATA_DIR
    bool do_coarse   = false;
    bool do_manual   = false;
    bool do_pangolin_panel = false;
    bool no_viz      = false;  // 默认启用 viz，--no-viz 时关闭
    bool prefer_tf   = true;   // prefer target-free
    
    // ROS2 参数
    bool use_ros2_bag = false;
    bool use_ros2_topics = false;
    bool use_new_format = false;
    std::string ros2_bag_file;
    std::string lidar_ros2_topic;
    std::string camera_ros2_topic;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--coarse")                 do_coarse = true;
        else if (a == "--manual")                 do_manual = true;
        else if (a == "--pangolin-panel")         do_pangolin_panel = true;
        else if (a == "--no-viz")                 no_viz = true;
        else if (a == "--no-targetfree")          prefer_tf = false;
        else if (a == "--method" && i+1 < argc)   method_str = argv[++i];
        else if (a == "--ai-root" && i+1 < argc)  ai_root = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--data-dir" && i+1 < argc)  data_dir = argv[++i];
        else if (a == "--log-level" && i+1 < argc) log_level = argv[++i];
        else if (a == "--ros2-bag" && i+1 < argc) {
            use_ros2_bag = true;
            ros2_bag_file = argv[++i];
        }
        else if (a == "--ros2-topic") {
            use_ros2_topics = true;
        }
        else if (a == "--lidar-topic" && i+1 < argc) {
            lidar_ros2_topic = argv[++i];
        }
        else if (a == "--camera-topic" && i+1 < argc) {
            camera_ros2_topic = argv[++i];
        }
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    UNICALIB_MAIN_TRY_BEGIN

    // ─────────────────────────────────────────────────────────────────
    // 初始化日志（写入 logs 目录，文件名带时间戳）
    // ─────────────────────────────────────────────────────────────────
    std::string logs_dir = resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/lidar_camera_" + log_timestamp_filename() + ".log";
    Logger::init("LiDAR-Camera",
                 log_file,
                 log_level == "debug"   ? spdlog::level::debug   :
                 log_level == "trace"   ? spdlog::level::trace   :
                 log_level == "warn"    ? spdlog::level::warn    :
                 log_level == "error"   ? spdlog::level::err     :
                                          spdlog::level::info);

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置文件: {}", config_file);
    UNICALIB_INFO("输出目录: {}", output_dir);
    UNICALIB_INFO("数据根目录: {} (--data-dir 或 CALIB_DATA_DIR)", data_dir.empty() ? "(未设置)" : data_dir);
    UNICALIB_INFO("精标定方法: {} (无目标优先={})", method_str, prefer_tf);
    UNICALIB_INFO("AI粗标定: {} | 手动校准: {}", do_coarse, do_manual);
    if (!do_coarse)
        UNICALIB_INFO("  粗标定未启用原因: 未传入 --coarse；启用粗标定请加 --coarse 并配置 coarse_pcd_file/coarse_image_file（或由 run.sh 传入 --coarse）");
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─────────────────────────────────────────────────────────────────
    // 加载配置
    // ─────────────────────────────────────────────────────────────────
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_file);
    } catch (const std::exception& e) {
        UNICALIB_ERROR("配置文件加载失败: {}", e.what());
        return 1;
    }

    // 从配置文件覆盖命令行默认值
    if (cfg["method"])      method_str = cfg["method"].as<std::string>();
    if (cfg["output_dir"])  output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["prefer_targetfree"]) prefer_tf = cfg["prefer_targetfree"].as<bool>();

    double manual_rms_thresh = 2.0;
    if (cfg["manual_rms_threshold"]) manual_rms_thresh = cfg["manual_rms_threshold"].as<double>();

    // ─────────────────────────────────────────────────────────────────
    // 数据路径解析 (统一格式 data.lidar / data.camera 或扁平键 lidar_data_dir / camera_images_dir)
    // ─────────────────────────────────────────────────────────────────
    std::string base_data_dir = data_dir.empty() ? (std::getenv("CALIB_DATA_DIR") ? std::getenv("CALIB_DATA_DIR") : "") : data_dir;
    std::string lidar_dir_raw;
    std::string camera_dir_raw;

    if (cfg["lidar_data_dir"])
        lidar_dir_raw = cfg["lidar_data_dir"].as<std::string>();
    if (cfg["camera_images_dir"])
        camera_dir_raw = cfg["camera_images_dir"].as<std::string>();

    if (lidar_dir_raw.empty() || camera_dir_raw.empty()) {
        try {
            SystemConfig sys_cfg = YamlIO::load_system_config(config_file);
            if (lidar_dir_raw.empty() && !sys_cfg.lidar_data_paths.empty())
                lidar_dir_raw = sys_cfg.lidar_data_paths.begin()->second;
            if (camera_dir_raw.empty() && !sys_cfg.camera_images_dirs.empty())
                camera_dir_raw = sys_cfg.camera_images_dirs.begin()->second;
        } catch (const std::exception& e) {
            UNICALIB_DEBUG("使用扁平键读取数据路径 (load_system_config 未用): {}", e.what());
        }
    }

    std::string lidar_dir_resolved  = resolve_data_path(base_data_dir, lidar_dir_raw);
    std::string camera_dir_resolved = resolve_data_path(base_data_dir, camera_dir_raw);

    // 若目录不存在则自动创建，便于首次运行或挂载卷为空时先建目录再放入数据
    if (!lidar_dir_resolved.empty() && !fs::exists(lidar_dir_resolved)) {
        std::error_code ec;
        fs::create_directories(lidar_dir_resolved, ec);
        if (ec)
            UNICALIB_WARN("自动创建 LiDAR 目录失败: {} — {}", lidar_dir_resolved, ec.message());
        else
            UNICALIB_INFO("已自动创建 LiDAR 数据目录: {}", lidar_dir_resolved);
    }
    if (!camera_dir_resolved.empty() && !fs::exists(camera_dir_resolved)) {
        std::error_code ec;
        fs::create_directories(camera_dir_resolved, ec);
        if (ec)
            UNICALIB_WARN("自动创建相机图像目录失败: {} — {}", camera_dir_resolved, ec.message());
        else
            UNICALIB_INFO("已自动创建相机图像目录: {}", camera_dir_resolved);
    }

    // 数据路径来源（便于排查：来自扁平键还是 data.lidar/data.camera）
    bool from_flat = cfg["lidar_data_dir"] && cfg["camera_images_dir"];
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ [数据路径] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("  数据路径来源: {}",
                  from_flat ? "扁平键 (lidar_data_dir / camera_images_dir)" : "统一格式 (data.lidar.<id> / data.camera.<id>.images_dir)");
    UNICALIB_INFO("  配置原始值 — lidar:  {}",
                  lidar_dir_raw.empty() ? "(未设置)" : lidar_dir_raw);
    UNICALIB_INFO("  配置原始值 — camera: {}",
                  camera_dir_raw.empty() ? "(未设置)" : camera_dir_raw);
    UNICALIB_INFO("  base_data_dir:  {}",
                  base_data_dir.empty() ? "(未设置，可设 CALIB_DATA_DIR 或 --data-dir)" : base_data_dir);
    if (!base_data_dir.empty()) {
        UNICALIB_INFO("  base 来源: {}",
                      data_dir.empty() ? "环境变量 CALIB_DATA_DIR" : "命令行 --data-dir");
    }
    bool lidar_absolute = !lidar_dir_raw.empty() && fs::path(lidar_dir_raw).is_absolute();
    bool cam_absolute   = !camera_dir_raw.empty() && fs::path(camera_dir_raw).is_absolute();
    UNICALIB_INFO("  lidar_dir:      {}",
                  lidar_dir_resolved.empty() ? "(未设置)" : lidar_dir_resolved);
    if (!lidar_dir_resolved.empty()) {
        UNICALIB_INFO("    → {}",
                      lidar_absolute ? "配置为绝对路径，未与 base 拼接（若为 /path/to/... 占位符则不会指向实际数据）"
                                    : "相对路径，已与 base 拼接");
        bool exists = fs::exists(lidar_dir_resolved);
        UNICALIB_INFO("    → 存在: {}", exists ? "是" : "否");
        if (!exists)
            UNICALIB_WARN("    LiDAR 数据目录不存在，精标定将无法加载点云");
    }
    UNICALIB_INFO("  camera_dir:     {}",
                  camera_dir_resolved.empty() ? "(未设置)" : camera_dir_resolved);
    if (!camera_dir_resolved.empty()) {
        UNICALIB_INFO("    → {}",
                      cam_absolute ? "配置为绝对路径，未与 base 拼接（若为 /path/to/... 占位符则不会指向实际数据）"
                                  : "相对路径，已与 base 拼接");
        bool exists = fs::exists(camera_dir_resolved);
        UNICALIB_INFO("    → 存在: {}", exists ? "是" : "否");
        if (!exists)
            UNICALIB_WARN("    相机图像目录不存在，精标定将无法加载图像");
    }

    bool data_ready = !lidar_dir_resolved.empty() && !camera_dir_resolved.empty() &&
                      fs::exists(lidar_dir_resolved) && fs::exists(camera_dir_resolved);
    std::string skip_reason;
    if (!data_ready) {
        if (base_data_dir.empty())
            skip_reason = "未设置数据根目录（请设置环境变量 CALIB_DATA_DIR 或命令行 --data-dir）";
        else if (lidar_dir_raw.find("/path/to") != std::string::npos || camera_dir_raw.find("/path/to") != std::string::npos)
            skip_reason = "配置中 data.lidar / data.camera 为占位符 /path/to/... 且为绝对路径，未与数据根目录拼接；请改为相对路径（如 pcd、images）或填写实际路径";
        else if (lidar_dir_resolved.empty() || camera_dir_resolved.empty())
            skip_reason = "配置中未提供 LiDAR 或相机数据路径（请填写 data.lidar.<id> 与 data.camera.<id>.images_dir）";
        else if (!fs::exists(lidar_dir_resolved))
            skip_reason = "LiDAR 目录不存在: " + lidar_dir_resolved;
        else
            skip_reason = "相机图像目录不存在: " + camera_dir_resolved;
    }
    UNICALIB_INFO("  【数据就绪】{} — {}",
                  data_ready ? "是" : "否",
                  data_ready ? "点云与图像目录均存在，将执行精标定" : ("原因: " + skip_reason));
    UNICALIB_INFO("  说明: 使用 --dataset 时请将 data.lidar/data.camera 设为相对 base 的路径（如 pcd、images），不要使用 /path/to/... 占位符");
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─────────────────────────────────────────────────────────────────
    // 构建流水线配置
    // ─────────────────────────────────────────────────────────────────
    PipelineConfig pipe_cfg;
    pipe_cfg.tasks               = CalibTaskType::LIDAR_CAM_EXTRIN;
    pipe_cfg.enable_coarse_lidar_cam = do_coarse;
    pipe_cfg.allow_manual_fallback   = do_manual;
    pipe_cfg.use_pangolin_manual_panel = do_pangolin_panel;  // 方案 B：点云叠加+面板
    pipe_cfg.enable_viz              = !no_viz;  // 默认启动可视化界面
    pipe_cfg.prefer_targetfree       = prefer_tf;
    pipe_cfg.lidar_cam_rms_threshold = manual_rms_thresh;
    pipe_cfg.output_dir              = output_dir;
    pipe_cfg.ai_models_root          = ai_root;
    pipe_cfg.log_level               = log_level;
    if (cfg["third_party"] && cfg["third_party"]["mias_lcec"]) {
        const auto& mias = cfg["third_party"]["mias_lcec"];
        if (mias["repo_dir"])      pipe_cfg.mias_lcec_repo_dir    = mias["repo_dir"].as<std::string>();
        if (mias["calib_script"])  pipe_cfg.mias_lcec_calib_script = mias["calib_script"].as<std::string>();
        if (mias["model_path"])    pipe_cfg.mias_lcec_model_path  = mias["model_path"].as<std::string>();
        if (mias["work_dir"])      pipe_cfg.mias_lcec_work_dir    = mias["work_dir"].as<std::string>();
        if (mias["allow_pnp_fallback"]) pipe_cfg.mias_lcec_allow_pnp_fallback = mias["allow_pnp_fallback"].as<bool>();
        if (mias["python_exe"])    pipe_cfg.mias_lcec_python_exe  = mias["python_exe"].as<std::string>();
    }
    if (cfg["lidar_camera"] && cfg["lidar_camera"]["camera_list"] && cfg["lidar_camera"]["camera_list"].IsSequence()) {
        for (const auto& v : cfg["lidar_camera"]["camera_list"])
            pipe_cfg.lidar_camera_camera_list.push_back(v.as<std::string>());
    }
    if (cfg["lidar_camera"] && cfg["lidar_camera"]["pairs"] && cfg["lidar_camera"]["pairs"].IsSequence()) {
        for (const auto& p : cfg["lidar_camera"]["pairs"]) {
            if (p.IsSequence() && p.size() >= 2)
                pipe_cfg.lidar_camera_pairs.emplace_back(p[0].as<std::string>(), p[1].as<std::string>());
        }
        if (!pipe_cfg.lidar_camera_pairs.empty())
            UNICALIB_INFO("[LiDAR-Cam] 配置 pairs: {} 对", pipe_cfg.lidar_camera_pairs.size());
    }
    if (cfg["lidar_camera"]) {
        const auto& lc = cfg["lidar_camera"];
        if (lc["board_cols"])   pipe_cfg.board_cols   = lc["board_cols"].as<int>();
        if (lc["board_rows"])   pipe_cfg.board_rows   = lc["board_rows"].as<int>();
        if (lc["square_size"])  pipe_cfg.square_size_m = lc["square_size"].as<double>();
        if (lc["square_size_m"]) pipe_cfg.square_size_m = lc["square_size_m"].as<double>();
        if (lc["target_type"])  pipe_cfg.lidar_cam_target_type = lc["target_type"].as<std::string>();
        if (lc["method"])       pipe_cfg.lidar_cam_method = lc["method"].as<std::string>();
        if (lc["target_min_corners_per_frame"]) pipe_cfg.target_min_corners_per_frame = lc["target_min_corners_per_frame"].as<int>();
        if (lc["target_min_frames"]) pipe_cfg.target_min_frames = lc["target_min_frames"].as<int>();
        if (lc["target_use_coarse_rotation_search"]) pipe_cfg.target_use_coarse_rotation_search = lc["target_use_coarse_rotation_search"].as<bool>();
        if (lc["target_coarse_rotation_range_deg"]) pipe_cfg.target_coarse_rotation_range_deg = lc["target_coarse_rotation_range_deg"].as<double>();
        if (lc["target_coarse_rotation_step_deg"]) pipe_cfg.target_coarse_rotation_step_deg = lc["target_coarse_rotation_step_deg"].as<double>();
        if (lc["target_coarse_inlier_threshold_px"]) pipe_cfg.target_coarse_inlier_threshold_px = lc["target_coarse_inlier_threshold_px"].as<double>();
        if (lc["target_per_frame_rms_threshold_px"]) pipe_cfg.target_per_frame_rms_threshold_px = lc["target_per_frame_rms_threshold_px"].as<double>();
        if (lc["target_ba_max_iter"]) pipe_cfg.target_ba_max_iter = lc["target_ba_max_iter"].as<int>();
        if (lc["target_ba_use_robust_loss"]) pipe_cfg.target_ba_use_robust_loss = lc["target_ba_use_robust_loss"].as<bool>();
        if (lc["target_ba_huber_scale_px"]) pipe_cfg.target_ba_huber_scale_px = lc["target_ba_huber_scale_px"].as<double>();
    }
    if (cfg["board_cols"])   pipe_cfg.board_cols   = cfg["board_cols"].as<int>();
    if (cfg["board_rows"])   pipe_cfg.board_rows   = cfg["board_rows"].as<int>();
    if (cfg["square_size_m"]) pipe_cfg.square_size_m = cfg["square_size_m"].as<double>();
    if (cfg["target_type"])  pipe_cfg.lidar_cam_target_type = cfg["target_type"].as<std::string>();
    if (cfg["lidar_camera"] && cfg["lidar_camera"]["fine"]) {
        const auto& fine = cfg["lidar_camera"]["fine"];
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
    // ─── LiDAR-Camera 粗标定(C3M) 与质量评估阈值 ─────────────────────────
    if (cfg["lidar_camera"]) {
        const auto& lc = cfg["lidar_camera"];
        if (lc["coarse"]) {
            const auto& coarse = lc["coarse"];
            // 可选：直接给出 use_iterative_refine
            if (coarse["c3m_use_iterative_refine"])
                pipe_cfg.lidar_cam_c3m_use_iterative_refine = coarse["c3m_use_iterative_refine"].as<bool>();
            // 兼容：使用 coarse.max_iterations / coarse.convergence_threshold 作为 iterative refine 参数
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
    // 粗标定初值：若配置了 initial_extrinsic 或 T_cam_lidar，则提供给粗标定（粗标定必执行，其结果为精标定初值）
    std::optional<Sophus::SE3d> lidar_cam_initial_from_config;
    bool use_config_extrinsic_only = false;
    if (cfg["lidar_camera"]) {
        const auto& lc = cfg["lidar_camera"];
        if (lc["use_config_extrinsic_only"])
            use_config_extrinsic_only = lc["use_config_extrinsic_only"].as<bool>();
        if (lc["initial_extrinsic"])
            lidar_cam_initial_from_config = parse_se3_from_yaml_4x4(lc["initial_extrinsic"]);
        if (!lidar_cam_initial_from_config.has_value() && lc["T_cam_lidar"])
            lidar_cam_initial_from_config = parse_se3_from_yaml_4x4(lc["T_cam_lidar"]);
    }
    if (lidar_cam_initial_from_config.has_value())
        pipe_cfg.lidar_cam_coarse_initial = lidar_cam_initial_from_config;
    // 多相机初值：lidar_camera.initial_extrinsics
    // 键支持 "lidar_id__camera_id" 或 "T_lidar_id__camera_id"，值为 rows/cols/data 的 4x4
    if (cfg["lidar_camera"] && cfg["lidar_camera"]["initial_extrinsics"] &&
        cfg["lidar_camera"]["initial_extrinsics"].IsMap()) {
        for (const auto& kv : cfg["lidar_camera"]["initial_extrinsics"]) {
            const std::string key = kv.first.as<std::string>();
            if (!kv.second || !kv.second.IsMap()) continue;
            std::vector<double> v = parse_4x4_matrix_data(kv.second);
            if (v.size() >= 16u) {
                pipe_cfg.lidar_camera_initial_extrinsic_inline[key] = std::move(v);
            } else {
                UNICALIB_WARN("[LiDAR-Cam] initial_extrinsics['{}'] 不是有效 4x4，已忽略", key);
            }
        }
        if (!pipe_cfg.lidar_camera_initial_extrinsic_inline.empty()) {
            UNICALIB_INFO("[LiDAR-Cam] 已加载 {} 组按相机初值", pipe_cfg.lidar_camera_initial_extrinsic_inline.size());
        }
    }
    pipe_cfg.lidar_cam_use_config_extrinsic_only = use_config_extrinsic_only;
    if (use_config_extrinsic_only) {
        do_coarse = false;
        UNICALIB_INFO("[LiDAR-Cam] use_config_extrinsic_only=true：跳过 AI 粗标定，将使用配置 initial_extrinsic 直接进入精标定/手动微调");
    }
    if (cfg["frame_sync_threshold_s"]) pipe_cfg.frame_sync_threshold_s = cfg["frame_sync_threshold_s"].as<double>();
    if (cfg["time_offset_search_range_s"]) pipe_cfg.time_offset_search_range_s = cfg["time_offset_search_range_s"].as<double>();
    if (cfg["ncc_threshold"]) pipe_cfg.ncc_threshold = cfg["ncc_threshold"].as<double>();
    if (cfg["ncc_low_skip_threshold"])  pipe_cfg.ncc_low_skip_threshold  = cfg["ncc_low_skip_threshold"].as<double>();
    if (!config_file.empty()) {
        try {
            SystemConfig sys_cfg = YamlIO::load_system_config(config_file);
            pipe_cfg.camera_images_dirs = sys_cfg.camera_images_dirs;
        } catch (const std::exception& e) {
            UNICALIB_DEBUG("加载系统配置以填充 camera_images_dirs 失败: {}", e.what());
        }
    }
    
    // ─── 数据源配置 (文件或 ROS2) ─────────────────────────────
    // 命令行 ROS2 标志与话题
    bool use_ros2_bag = false;
    bool use_ros2_topics = false;
    std::string ros2_bag_file;
    std::string lidar_ros2_topic;
    std::string camera_ros2_topic;
    
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--ros2-bag" && i+1 < argc) {
            use_ros2_bag = true;
            ros2_bag_file = argv[++i];
        } else if (a == "--ros2-topic") {
            use_ros2_topics = true;
        } else if (a == "--lidar-topic" && i+1 < argc) {
            lidar_ros2_topic = argv[++i];
        } else if (a == "--camera-topic" && i+1 < argc) {
            camera_ros2_topic = argv[++i];
        }
    }
    
    // 未从命令行指定时，从配置文件 ros2 / data / sensors 读取
    std::string imu_ros2_topic;
    const YAML::Node new_format_node = cfg["new_format"];
    if (new_format_node && new_format_node["enable"] && new_format_node["enable"].as<bool>()) {
        use_new_format = true;
        if (new_format_node["root_dir"]) {
            std::string root_dir = new_format_node["root_dir"].as<std::string>();
            if (!base_data_dir.empty() && !fs::path(root_dir).is_absolute()) {
                root_dir = resolve_data_path(base_data_dir, root_dir);
            }
            pipe_cfg.new_format_root_dir = root_dir;
        }
        if (new_format_node["oem7_imu_rate_hz"])
            pipe_cfg.new_format_oem7_imu_rate_hz = new_format_node["oem7_imu_rate_hz"].as<double>();
        if (new_format_node["oem7_time_base"])
            pipe_cfg.new_format_oem7_time_base = new_format_node["oem7_time_base"].as<std::string>();
        if (new_format_node["oem7_gps_utc_leap_sec"])
            pipe_cfg.new_format_oem7_gps_utc_leap_sec = new_format_node["oem7_gps_utc_leap_sec"].as<int>();
        if (new_format_node["oem7_time_offset_sec"])
            pipe_cfg.new_format_oem7_time_offset_sec = new_format_node["oem7_time_offset_sec"].as<double>();
        if (new_format_node["timestamp_unit"]) {
            pipe_cfg.new_format_timestamp_unit = new_format_node["timestamp_unit"].as<std::string>();
        }
        if (new_format_node["lidar_index_files"] && new_format_node["lidar_index_files"].IsMap()) {
            for (const auto& kv : new_format_node["lidar_index_files"]) {
                pipe_cfg.new_format_lidar_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
            }
        }
        if (new_format_node["camera_index_files"] && new_format_node["camera_index_files"].IsMap()) {
            for (const auto& kv : new_format_node["camera_index_files"]) {
                pipe_cfg.new_format_camera_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
            }
        }
        if (new_format_node["imu_index_files"] && new_format_node["imu_index_files"].IsMap()) {
            for (const auto& kv : new_format_node["imu_index_files"]) {
                pipe_cfg.new_format_imu_index_files[kv.first.as<std::string>()] = kv.second.as<std::string>();
            }
        }
    }

    const YAML::Node ros2_node = cfg["ros2"];
    if (ros2_node) {
        if (!use_ros2_bag && ros2_node["use_ros2_bag"] && ros2_node["use_ros2_bag"].as<bool>()) {
            use_ros2_bag = true;
            if (ros2_bag_file.empty() && ros2_node["ros2_bag_file"])
                ros2_bag_file = ros2_node["ros2_bag_file"].as<std::string>();
        }
        if (!use_ros2_topics && ros2_node["use_ros2_topics"] && ros2_node["use_ros2_topics"].as<bool>())
            use_ros2_topics = true;
        if (lidar_ros2_topic.empty() && ros2_node["lidar_topic"])
            lidar_ros2_topic = ros2_node["lidar_topic"].as<std::string>();
        if (camera_ros2_topic.empty() && ros2_node["camera_topic"]) {
            const auto& ct = ros2_node["camera_topic"];
            if (ct.IsSequence() && ct.size() > 0) {
                for (size_t i = 0; i < ct.size(); ++i)
                    pipe_cfg.camera_topics["cam_" + std::to_string(i)] = ct[i].as<std::string>();
                camera_ros2_topic = ct[0].as<std::string>();
                pipe_cfg.camera_id = "cam_0";
            } else {
                camera_ros2_topic = ct.as<std::string>();
            }
        }
        if (ros2_node["imu_topic"])
            imu_ros2_topic = ros2_node["imu_topic"].as<std::string>();
        if (ros2_node["max_wait_time"])
            pipe_cfg.ros2_max_wait_time = ros2_node["max_wait_time"].as<double>();
        if (ros2_node["sample_interval"])
            pipe_cfg.ros2_sample_interval = ros2_node["sample_interval"].as<double>();
        if (ros2_node["max_frames"])
            pipe_cfg.ros2_max_frames = ros2_node["max_frames"].as<size_t>();
    }
    // data.bag_file 作为 ros2 bag 路径回退（与 data 段统一）
    if (ros2_bag_file.empty() && cfg["data"] && cfg["data"]["bag_file"])
        ros2_bag_file = cfg["data"]["bag_file"].as<std::string>();
    // 相对路径相对于数据根目录解析（支持 --dataset nya_02_ros2 等）
    if (use_ros2_bag && !ros2_bag_file.empty() && !base_data_dir.empty()) {
        fs::path p(ros2_bag_file);
        if (!p.is_absolute())
            ros2_bag_file = resolve_data_path(base_data_dir, ros2_bag_file);
    }
    if (lidar_ros2_topic.empty() || camera_ros2_topic.empty() || imu_ros2_topic.empty()) {
        try {
            SystemConfig sys_cfg = YamlIO::load_system_config(config_file);
            pipe_cfg.camera_images_dirs = sys_cfg.camera_images_dirs;
            for (const auto& s : sys_cfg.sensors) {
                if (s.type == SensorType::LiDAR && lidar_ros2_topic.empty() && !s.topic.empty()) {
                    lidar_ros2_topic = s.topic;
                    pipe_cfg.lidar_id = s.sensor_id;
                }
                if (s.type == SensorType::CAMERA && camera_ros2_topic.empty() && !s.topic.empty()) {
                    camera_ros2_topic = s.topic;
                    pipe_cfg.camera_id = s.sensor_id;
                }
                if (s.type == SensorType::IMU && imu_ros2_topic.empty() && !s.topic.empty())
                    imu_ros2_topic = s.topic;
            }
        } catch (const std::exception& e) {
            UNICALIB_DEBUG("从系统配置读取 ROS2 话题失败，沿用命令行或 ros2 段: {}", e.what());
        }
    }
    // pairs 优先于 sensors[] 默认的第一个 LiDAR/相机（文件模式标定对须与 initial_extrinsics 键一致）
    if (!pipe_cfg.lidar_camera_pairs.empty()) {
        const auto& [lid, cam] = pipe_cfg.lidar_camera_pairs.front();
        pipe_cfg.lidar_id = lid;
        pipe_cfg.camera_id = cam;
        UNICALIB_INFO("[LiDAR-Cam] 标定对（pairs）: lidar_id={} camera_id={}", lid, cam);
    }

    // 设置数据源配置（在线=实时话题，离线=bag 文件；话题名以 config 中 ros2/sensors 为准）
    if (use_new_format) {
        pipe_cfg.use_new_format = true;
        UNICALIB_INFO("配置: 使用 NEW_FORMAT 模式（索引 CSV 对齐）");
        UNICALIB_INFO("  root_dir: {}", pipe_cfg.new_format_root_dir.empty() ? "(未设置)" : pipe_cfg.new_format_root_dir);
    } else if (use_ros2_bag && !ros2_bag_file.empty()) {
        pipe_cfg.use_ros2_bag = true;
        pipe_cfg.ros2_bag_file = ros2_bag_file;
        pipe_cfg.lidar_ros2_topic = lidar_ros2_topic;
        pipe_cfg.camera_ros2_topic = camera_ros2_topic;
        pipe_cfg.imu_ros2_topic = imu_ros2_topic;
        
        UNICALIB_INFO("配置: 使用 ROS2 bag 模式（离线标定）");
        UNICALIB_INFO("  Bag 路径: {}", ros2_bag_file);
        UNICALIB_INFO("  LiDAR 话题: {}", pipe_cfg.lidar_ros2_topic.empty() ? "(未设置)" : pipe_cfg.lidar_ros2_topic);
        UNICALIB_INFO("  相机话题: {}", pipe_cfg.camera_ros2_topic.empty() ? "(未设置)" : pipe_cfg.camera_ros2_topic);
        UNICALIB_INFO("  IMU 话题: {}", pipe_cfg.imu_ros2_topic.empty() ? "(未设置)" : pipe_cfg.imu_ros2_topic);
    } else if (use_ros2_topics) {
        pipe_cfg.use_ros2_topics = true;
        pipe_cfg.lidar_ros2_topic = lidar_ros2_topic;
        pipe_cfg.camera_ros2_topic = camera_ros2_topic;
        pipe_cfg.imu_ros2_topic = imu_ros2_topic;
        
        UNICALIB_INFO("配置: 使用 ROS2 实时话题模式（在线标定）");
        UNICALIB_INFO("  LiDAR 话题: {}", pipe_cfg.lidar_ros2_topic.empty() ? "(未设置)" : pipe_cfg.lidar_ros2_topic);
        UNICALIB_INFO("  相机话题: {}", pipe_cfg.camera_ros2_topic.empty() ? "(未设置)" : pipe_cfg.camera_ros2_topic);
        UNICALIB_INFO("  最大等待: {:.1f}s, 最大帧数: {}", pipe_cfg.ros2_max_wait_time, pipe_cfg.ros2_max_frames);
    } else {
        try {
            SystemConfig sys_cfg_paths = YamlIO::load_system_config(config_file);
            pipe_cfg.lidar_data_paths = sys_cfg_paths.lidar_data_paths;
            if (pipe_cfg.camera_images_dirs.empty())
                pipe_cfg.camera_images_dirs = sys_cfg_paths.camera_images_dirs;
        } catch (const std::exception& e) {
            UNICALIB_DEBUG("加载 data 路径映射失败: {}", e.what());
        }
        pipe_cfg.lidar_data_dir    = lidar_dir_resolved;
        pipe_cfg.camera_images_dir = camera_dir_resolved;
        if (!pipe_cfg.lidar_id.empty()) {
            auto itl = pipe_cfg.lidar_data_paths.find(pipe_cfg.lidar_id);
            if (itl != pipe_cfg.lidar_data_paths.end() && !itl->second.empty())
                pipe_cfg.lidar_data_dir = itl->second;
        }
        if (!pipe_cfg.camera_id.empty()) {
            auto itc = pipe_cfg.camera_images_dirs.find(pipe_cfg.camera_id);
            if (itc != pipe_cfg.camera_images_dirs.end() && !itc->second.empty())
                pipe_cfg.camera_images_dir = itc->second;
        }
    }

    // 统一日志：当前数据源类型（便于 grep 与排障）
    if (pipe_cfg.use_new_format) {
        UNICALIB_INFO("[数据源] 类型=NEW_FORMAT  root_dir={}  timestamp_unit={}",
                      pipe_cfg.new_format_root_dir.empty() ? "(未设置)" : pipe_cfg.new_format_root_dir,
                      pipe_cfg.new_format_timestamp_unit);
    } else if (pipe_cfg.use_ros2_bag && !pipe_cfg.ros2_bag_file.empty()) {
        UNICALIB_INFO("[数据源] 类型=ROS2_BAG  path={}  lidar_topic={}  camera_topic={}",
                      pipe_cfg.ros2_bag_file, pipe_cfg.lidar_ros2_topic, pipe_cfg.camera_ros2_topic);
    } else if (pipe_cfg.use_ros2_topics) {
        UNICALIB_INFO("[数据源] 类型=ROS2_TOPIC  lidar_topic={}  camera_topic={}  max_wait={:.1f}s",
                      pipe_cfg.lidar_ros2_topic, pipe_cfg.camera_ros2_topic, pipe_cfg.ros2_max_wait_time);
    } else {
        UNICALIB_INFO("[数据源] 类型=FILES  lidar_dir={}  camera_dir={}",
                      pipe_cfg.lidar_data_dir.empty() ? "(未设置)" : pipe_cfg.lidar_data_dir,
                      pipe_cfg.camera_images_dir.empty() ? "(未设置)" : pipe_cfg.camera_images_dir);
    }

    // 相机内参（与 joint_calib 一致，读取 data.camera.<id>.intrinsic_yaml）：
    //   1) data.camera.<id>.intrinsic_yaml（相对 --data-dir；多路可共用 cameras 映射 YAML）
    //   2) 根键 camera_intrinsic_file 覆盖当前 camera_id
    //   3) 若存在 results/camera_intrinsic/camera_intrinsic_<id>.yaml 则兜底
    std::string results_camera_intrinsic = "camera_intrinsic";
    if (cfg["results"] && cfg["results"]["camera_intrinsic"])
        results_camera_intrinsic = cfg["results"]["camera_intrinsic"].as<std::string>();
    if (cfg["data"] && cfg["data"]["camera"]) {
        for (const auto& it : cfg["data"]["camera"]) {
            if (!it.second.IsMap()) continue;
            const std::string cam_id = it.first.as<std::string>();
            if (it.second["intrinsic_yaml"]) {
                pipe_cfg.camera_intrinsic_files[cam_id] =
                    it.second["intrinsic_yaml"].as<std::string>();
                UNICALIB_INFO("[LiDAR-Cam] 内参 data.camera.{}.intrinsic_yaml: {}",
                              cam_id, pipe_cfg.camera_intrinsic_files[cam_id]);
            } else {
                pipe_cfg.camera_intrinsic_files[cam_id] =
                    output_dir + "/" + results_camera_intrinsic + "/camera_intrinsic_" + cam_id + ".yaml";
            }
        }
    }
    if (cfg["camera_intrinsic_file"]) {
        std::string p = cfg["camera_intrinsic_file"].as<std::string>();
        if (!base_data_dir.empty() && !fs::path(p).is_absolute())
            p = resolve_data_path(base_data_dir, p);
        pipe_cfg.camera_intrinsic_file = p;
        pipe_cfg.camera_intrinsic_files[pipe_cfg.camera_id] = p;
        UNICALIB_INFO("[LiDAR-Cam] 内参覆盖 camera_intrinsic_file: {} (camera_id={})",
                      p, pipe_cfg.camera_id);
    }
    auto try_add_results_intrinsic = [&](const std::string& cam_id) {
        if (pipe_cfg.camera_intrinsic_files.count(cam_id) != 0) return;
        std::string p = output_dir + "/" + results_camera_intrinsic + "/camera_intrinsic_" + cam_id + ".yaml";
        if (fs::exists(p)) {
            pipe_cfg.camera_intrinsic_files[cam_id] = p;
            UNICALIB_INFO("[LiDAR-Cam] 内参从标定结果: {} (camera_id={})", p, cam_id);
        }
    };
    try_add_results_intrinsic(pipe_cfg.camera_id);
    for (const auto& cid : pipe_cfg.lidar_camera_camera_list)
        try_add_results_intrinsic(cid);

    // 与 cam-intrin 一致：Docker/容器内 NumPy 2.x 时由 run 脚本设置 UNICALIB_MIAS_LCEC_PYTHON（NumPy 1.x wrapper），粗标定子进程使用该解释器以便 cv2/PnP 可用
    const char* mias_py = std::getenv("UNICALIB_MIAS_LCEC_PYTHON");
    if (mias_py && mias_py[0]) pipe_cfg.mias_lcec_python_exe = mias_py;

    resolve_pipeline_data_paths(pipe_cfg, base_data_dir);

    CalibPipeline pipeline(pipe_cfg);

    // ─────────────────────────────────────────────────────────────────
    // Stage 1: AI 粗标定 (可选)；粗标定结果将作为精标定初值；若配置了 initial_extrinsic 则已传入 pipeline 供粗标定使用
    // ─────────────────────────────────────────────────────────────────
    std::optional<Sophus::SE3d> coarse_init;
    if (do_coarse) {
        UNICALIB_INFO("▶ Stage 1: AI 粗标定 (MIAS-LCEC)");
        UNICALIB_INFO("  模型路径: {}/MIAS-LCEC (粗标定使用 pretrained_overlap_transformer.pth.tar)", ai_root);

        AICoarseCalibManager::Config ai_cfg;
        ai_cfg.ai_root    = ai_root;
        ai_cfg.python_exe = cfg["python_exe"] ?
            cfg["python_exe"].as<std::string>() : "python3";
        if (cfg["third_party"] && cfg["third_party"]["mias_lcec"]) {
            const auto& mias = cfg["third_party"]["mias_lcec"];
            if (mias["calib_script"]) ai_cfg.mias_lcec.calib_script = mias["calib_script"].as<std::string>();
            if (mias["repo_dir"])    ai_cfg.mias_lcec.repo_dir    = mias["repo_dir"].as<std::string>();
            if (mias["model_path"])  ai_cfg.mias_lcec.model_path  = mias["model_path"].as<std::string>();
            if (mias["python_exe"])  ai_cfg.python_exe            = mias["python_exe"].as<std::string>();
        }
        const char* mias_py_env = std::getenv("UNICALIB_MIAS_LCEC_PYTHON");
        if (mias_py_env && mias_py_env[0]) ai_cfg.python_exe = mias_py_env;
        ai_cfg.mias_lcec.allow_pnp_fallback = pipe_cfg.mias_lcec_allow_pnp_fallback;
        // 粗标定运行时必须使用 pretrained_overlap_transformer.pth.tar：配置未指定时使用默认路径
        if (ai_cfg.mias_lcec.model_path.empty() && !ai_root.empty()) {
            fs::path default_model = fs::path(ai_root) / "MIAS-LCEC" / "model" / "pretrained_overlap_transformer.pth.tar";
            if (fs::exists(default_model))
                ai_cfg.mias_lcec.model_path = default_model.lexically_normal().string();
        } else if (!ai_cfg.mias_lcec.model_path.empty() && !ai_root.empty()) {
            fs::path mp(ai_cfg.mias_lcec.model_path);
            if (!mp.is_absolute())
                ai_cfg.mias_lcec.model_path = (fs::path(ai_root) / ai_cfg.mias_lcec.model_path).lexically_normal().string();
        }

        AICoarseCalibManager ai_mgr(ai_cfg);
        ai_mgr.print_availability();

        bool mias_available = ai_mgr.check_mias_lcec();
        UNICALIB_INFO("  [Stage 1] MIAS-LCEC 可用: {} (粗标定支持: 1) 指定 coarse_pcd_file/coarse_image_file 2) 从当前数据源取首帧，同相机内参)",
                      mias_available ? "是" : "否");

        // 内参：优先 camera_intrinsic_file，否则使用已从 results.camera_intrinsic 目录填入的 pipe_cfg.camera_intrinsic_files[camera_id]（需在取首帧前加载，便于 ROS2 首帧推断时使用）
        CameraIntrinsics cam_intrin;
        std::string intrin_path;
        if (cfg["camera_intrinsic_file"]) {
            intrin_path = cfg["camera_intrinsic_file"].as<std::string>();
            if (!base_data_dir.empty() && !fs::path(intrin_path).is_absolute())
                intrin_path = resolve_data_path(base_data_dir, intrin_path);
        } else {
            auto it = pipe_cfg.camera_intrinsic_files.find(pipe_cfg.camera_id);
            if (it != pipe_cfg.camera_intrinsic_files.end() && fs::exists(it->second))
                intrin_path = it->second;
        }
        if (!intrin_path.empty()) {
            try {
                auto intrin_node = YAML::LoadFile(intrin_path);
                YAML::Node proj_params = intrin_node["projection_parameters"];
                const YAML::Node& ref_node = (proj_params && proj_params.IsDefined()) ? proj_params : intrin_node;
                if (ref_node["fx"])  cam_intrin.fx = ref_node["fx"].as<double>();
                if (ref_node["fy"])  cam_intrin.fy = ref_node["fy"].as<double>();
                if (ref_node["cx"])  cam_intrin.cx = ref_node["cx"].as<double>();
                if (ref_node["cy"])  cam_intrin.cy = ref_node["cy"].as<double>();
                if (intrin_node["width"])  cam_intrin.width  = intrin_node["width"].as<int>();
                else if (intrin_node["image_width"]) cam_intrin.width  = intrin_node["image_width"].as<int>();
                if (intrin_node["height"]) cam_intrin.height = intrin_node["height"].as<int>();
                else if (intrin_node["image_height"]) cam_intrin.height = intrin_node["image_height"].as<int>();
                UNICALIB_INFO("  内参: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} 尺寸={}x{} (来自 {})",
                              cam_intrin.fx, cam_intrin.fy, cam_intrin.cx, cam_intrin.cy,
                              cam_intrin.width, cam_intrin.height, intrin_path);
            } catch (const std::exception& e) {
                UNICALIB_WARN("  内参文件加载失败: {} — {}", intrin_path, e.what());
            }
        }

        std::string pcd_file   = cfg["coarse_pcd_file"] ?
            cfg["coarse_pcd_file"].as<std::string>() : "";
        std::string image_file = cfg["coarse_image_file"] ?
            cfg["coarse_image_file"].as<std::string>() : "";
        if (!pcd_file.empty() && !base_data_dir.empty()) pcd_file = resolve_data_path(base_data_dir, pcd_file);
        if (!image_file.empty() && !base_data_dir.empty()) image_file = resolve_data_path(base_data_dir, image_file);
        if (!pcd_file.empty() && !image_file.empty())
            UNICALIB_INFO("  [Stage 1] 粗标定输入来源: 配置文件  pcd={}  image={}", pcd_file, image_file);

        // 未指定文件时，从当前数据源取首帧（兼容 ROS2 话题/bag 与文件目录，参考相机内参粗标定）
        if ((pcd_file.empty() || image_file.empty()) && mias_available) {
            const bool use_ros2 = pipe_cfg.use_ros2_bag || pipe_cfg.use_ros2_topics;
            if (use_ros2) {
#if defined(UNICALIB_WITH_ROS2) && UNICALIB_WITH_ROS2
                RosDataSourceConfig ros_cfg;
                ros_cfg.bag_file = pipe_cfg.ros2_bag_file;
                ros_cfg.realtime_mode = pipe_cfg.use_ros2_topics;
                ros_cfg.realtime_timeout = pipe_cfg.ros2_max_wait_time;
                ros_cfg.sample_interval = pipe_cfg.ros2_sample_interval;
                ros_cfg.max_frames = 2;
                ros_cfg.strict_topic_match = pipe_cfg.ros2_strict_topic_match;
                if (!pipe_cfg.lidar_topics.empty()) {
                    ros_cfg.lidar_topics = pipe_cfg.lidar_topics;
                    ros_cfg.lidar_ros2_topic = pipe_cfg.lidar_topics.begin()->second;
                } else {
                    ros_cfg.lidar_topics[pipe_cfg.lidar_id] = pipe_cfg.lidar_ros2_topic;
                    ros_cfg.lidar_ros2_topic = pipe_cfg.lidar_ros2_topic;
                }
                if (!pipe_cfg.camera_topics.empty()) {
                    ros_cfg.camera_topics = pipe_cfg.camera_topics;
                    ros_cfg.camera_ros2_topic = pipe_cfg.camera_topics.begin()->second;
                } else {
                    ros_cfg.camera_topics[pipe_cfg.camera_id] = pipe_cfg.camera_ros2_topic;
                    ros_cfg.camera_ros2_topic = pipe_cfg.camera_ros2_topic;
                }
                if (!pipe_cfg.imu_topics.empty()) {
                    ros_cfg.imu_topics = pipe_cfg.imu_topics;
                    ros_cfg.imu_ros2_topic = pipe_cfg.imu_topics.begin()->second;
                } else {
                    ros_cfg.imu_topics["imu_0"] = pipe_cfg.imu_ros2_topic;
                    ros_cfg.imu_ros2_topic = pipe_cfg.imu_ros2_topic;
                }
                UnifiedDataLoader::Config load_cfg;
                load_cfg.source_type = pipe_cfg.use_ros2_bag ?
                    UnifiedDataLoader::SourceType::ROS2_BAG : UnifiedDataLoader::SourceType::ROS2_TOPIC;
                load_cfg.ros_config = ros_cfg;
                load_cfg.max_frames = 2;
                UNICALIB_INFO("  [Stage 1] 粗标定输入来源: 从 ROS2 数据源取首帧 (max_frames=2, lidar_id={})", pipe_cfg.lidar_id);
                UnifiedDataLoader loader(load_cfg);
                if (loader.load()) {
                    std::vector<LiDARScan> scans = loader.to_lidar_scans(pipe_cfg.lidar_id);
                    std::string first_cam_id = pipe_cfg.camera_id;
                    if (!pipe_cfg.lidar_camera_camera_list.empty())
                        first_cam_id = pipe_cfg.lidar_camera_camera_list.front();
                    std::vector<std::pair<double, cv::Mat>> frames = loader.to_camera_frames(first_cam_id);
                    UNICALIB_INFO("  [Stage 1] ROS2 加载结果: LiDAR {} 帧  相机 {} 帧 (sensor_id={})", scans.size(), frames.size(), first_cam_id);
                    if (!scans.empty() && !frames.empty() && scans[0].cloud && !frames[0].second.empty()) {
                        std::string coarse_dir = output_dir + "/coarse_input";
                        fs::create_directories(coarse_dir);
                        pcd_file = coarse_dir + "/coarse_first.pcd";
                        image_file = coarse_dir + "/coarse_first.png";
                        if (pcl::io::savePCDFile(pcd_file, *scans[0].cloud, false) == 0 &&
                            cv::imwrite(image_file, frames[0].second)) {
                            UNICALIB_INFO("  [Stage 1] 已写入首帧到: PCD={}  image={} (ASCII PCD 供 Python 兼容)", pcd_file, image_file);
                            if (cam_intrin.fx <= 0 || cam_intrin.width <= 0) {
                                cam_intrin.width  = frames[0].second.cols;
                                cam_intrin.height = frames[0].second.rows;
                                cam_intrin.fx = cam_intrin.fy = std::max(cam_intrin.width, cam_intrin.height) * 0.8;
                                cam_intrin.cx = cam_intrin.width * 0.5;
                                cam_intrin.cy = cam_intrin.height * 0.5;
                                UNICALIB_INFO("  [Stage 1] 内参未配置，从首帧推断: {}x{}  fx={:.1f}  cx={:.1f}  cy={:.1f}",
                                             cam_intrin.width, cam_intrin.height, cam_intrin.fx, cam_intrin.cx, cam_intrin.cy);
                            }
                        } else {
                            UNICALIB_WARN("  [Stage 1] 写入首帧失败 (PCD/图像保存异常)");
                            pcd_file.clear();
                            image_file.clear();
                        }
                    } else {
                        UNICALIB_WARN("  [Stage 1] ROS2 首帧不足: 需至少 1 帧 LiDAR 与 1 帧相机");
                    }
                } else {
                    UNICALIB_WARN("  [Stage 1] ROS2 加载首帧失败: {}", loader.get_status_message());
                }
#else
                UNICALIB_WARN("  未配置 coarse_pcd_file/coarse_image_file 且 ROS2 未编译，无法从 bag/话题取首帧");
#endif
            } else if (!lidar_dir_resolved.empty() && !camera_dir_resolved.empty() &&
                       fs::exists(lidar_dir_resolved) && fs::exists(camera_dir_resolved)) {
                std::vector<fs::path> pcd_paths;
                for (const auto& e : fs::directory_iterator(lidar_dir_resolved)) {
                    std::string ext = e.path().extension().string();
                    if (ext == ".pcd" || ext == ".PCD") pcd_paths.push_back(e.path());
                }
                std::sort(pcd_paths.begin(), pcd_paths.end());
                std::vector<fs::path> img_paths;
                for (const auto& e : fs::directory_iterator(camera_dir_resolved)) {
                    std::string ext = e.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp") img_paths.push_back(e.path());
                }
                std::sort(img_paths.begin(), img_paths.end());
                if (!pcd_paths.empty() && !img_paths.empty()) {
                    pcd_file = pcd_paths.front().string();
                    image_file = img_paths.front().string();
                    UNICALIB_INFO("  [Stage 1] 粗标定输入来源: 文件目录首帧  PCD={}  图像={}", pcd_file, image_file);
                } else {
                    UNICALIB_WARN("  [Stage 1] 文件目录无可用首帧: PCD 数={}  图像数={} (dirs: {} / {})",
                                  pcd_paths.size(), img_paths.size(), lidar_dir_resolved, camera_dir_resolved);
                }
            }
        }

        if (!pcd_file.empty() && !image_file.empty()) {
            if (cam_intrin.fx <= 0 || cam_intrin.width <= 0) {
                UNICALIB_WARN("  [Stage 1] 粗标定前内参仍为 0，Python 端可能无法正确投影；请配置 camera_intrinsic_file 或确保从 ROS2 首帧已推断");
            }
            UNICALIB_INFO("  [Stage 1] 标定对: lidar_id={}  camera_id={}", pipe_cfg.lidar_id, pipe_cfg.camera_id);
            UNICALIB_INFO("  [Stage 1] 调用粗标定: pcd={}  image={} 内参: fx={:.1f} 尺寸={}x{}",
                         pcd_file, image_file, cam_intrin.fx, cam_intrin.width, cam_intrin.height);
            auto coarse_result = ai_mgr.coarse_lidar_cam(
                pcd_file, image_file, cam_intrin);
            if (coarse_result.has_value()) {
                coarse_init = coarse_result->SE3_TargetInRef();
                UNICALIB_INFO("  ✓ AI粗标定成功: t=[{:.3f},{:.3f},{:.3f}]m",
                              coarse_init->translation().x(),
                              coarse_init->translation().y(),
                              coarse_init->translation().z());
                UNICALIB_INFO("  [Stage 1] 结果: 成功  初值将用于精标定");
            } else {
                UNICALIB_WARN("  ✗ AI粗标定失败, 使用 identity 初始化");
                UNICALIB_INFO("  [Stage 1] 结果: 失败 (MIAS-LCEC 返回空，见上方错误)");
            }
        } else {
            UNICALIB_WARN("  未提供 coarse_pcd_file/coarse_image_file 且未从当前数据源取到首帧，跳过 AI 粗标定");
            UNICALIB_INFO("  说明: 粗标定支持 1) 在 config 中设置 coarse_pcd_file / coarse_image_file 2) 使用 ROS2 bag/话题或文件目录时自动取首帧（同相机内参粗标定）");
            UNICALIB_INFO("  当前: coarse_pcd_file='{}' coarse_image_file='{}'",
                          pcd_file.empty() ? "(未设置)" : pcd_file,
                          image_file.empty() ? "(未设置)" : image_file);
        }

        if (!coarse_init.has_value()) {
            UNICALIB_WARN("【粗标定未执行】精标定将使用 identity 初值。原因: (1) MIAS-LCEC 可用={} (2) 粗标定输入已就绪={}",
                          mias_available ? "是" : "否",
                          (!pcd_file.empty() && !image_file.empty()) ? "是" : "否");
            UNICALIB_INFO("  [Stage 1] 结果: 跳过 (详见上方原因)");
        } else {
            UNICALIB_INFO("  [Stage 1] 结果: 成功  精标定初值已就绪");
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // Stage 2: 精标定
    // ─────────────────────────────────────────────────────────────────
    UNICALIB_INFO("▶ Stage 2: 精标定 (方法={})", method_str);

    LiDARCameraCalibrator::Config calib_cfg;
    if      (method_str == "edge")   calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
    else if (method_str == "target") calib_cfg.method = LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
    else if (method_str == "motion") calib_cfg.method = LiDARCameraCalibrator::Method::MOTION_BSPLINE;
    else {
        UNICALIB_WARN("未知方法 '{}', 使用 edge", method_str);
        calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
    }

    if (cfg["lidar_camera"]) {
        const auto& lc = cfg["lidar_camera"];
        if (lc["board_cols"])    calib_cfg.board_cols = lc["board_cols"].as<int>();
        if (lc["board_rows"])    calib_cfg.board_rows = lc["board_rows"].as<int>();
        if (lc["square_size"])   calib_cfg.square_size_m = lc["square_size"].as<double>();
        if (lc["square_size_m"]) calib_cfg.square_size_m = lc["square_size_m"].as<double>();
        if (lc["target_type"]) {
            const std::string tt = lc["target_type"].as<std::string>();
            if (tt == "circles_grid")      calib_cfg.target_type = LiDARCameraCalibrator::TargetType::CIRCLES_GRID;
            else if (tt == "asym_circles") calib_cfg.target_type = LiDARCameraCalibrator::TargetType::ASYMMETRIC_CIRCLES;
            else                            calib_cfg.target_type = LiDARCameraCalibrator::TargetType::CHESSBOARD;
        }
        if (lc["circle_diameter_m"]) calib_cfg.circle_diameter_m = lc["circle_diameter_m"].as<double>();
        if (lc["target_min_corners_per_frame"]) calib_cfg.target_min_corners_per_frame = lc["target_min_corners_per_frame"].as<int>();
        if (lc["target_min_frames"]) calib_cfg.target_min_frames = lc["target_min_frames"].as<int>();
        if (lc["target_use_coarse_rotation_search"]) calib_cfg.target_use_coarse_rotation_search = lc["target_use_coarse_rotation_search"].as<bool>();
        if (lc["target_coarse_rotation_range_deg"]) calib_cfg.target_coarse_rotation_range_deg = lc["target_coarse_rotation_range_deg"].as<double>();
        if (lc["target_coarse_rotation_step_deg"]) calib_cfg.target_coarse_rotation_step_deg = lc["target_coarse_rotation_step_deg"].as<double>();
        if (lc["target_coarse_inlier_threshold_px"]) calib_cfg.target_coarse_inlier_threshold_px = lc["target_coarse_inlier_threshold_px"].as<double>();
        if (lc["target_per_frame_rms_threshold_px"]) calib_cfg.target_per_frame_rms_threshold_px = lc["target_per_frame_rms_threshold_px"].as<double>();
        if (lc["target_ba_max_iter"]) calib_cfg.target_ba_max_iter = lc["target_ba_max_iter"].as<int>();
        if (lc["target_ba_use_robust_loss"]) calib_cfg.target_ba_use_robust_loss = lc["target_ba_use_robust_loss"].as<bool>();
        if (lc["target_ba_huber_scale_px"]) calib_cfg.target_ba_huber_scale_px = lc["target_ba_huber_scale_px"].as<double>();
    }
    if (cfg["board_cols"])    calib_cfg.board_cols = cfg["board_cols"].as<int>();
    if (cfg["board_rows"])    calib_cfg.board_rows = cfg["board_rows"].as<int>();
    if (cfg["square_size_m"]) calib_cfg.square_size_m = cfg["square_size_m"].as<double>();
    if (cfg["target_type"]) {
        const std::string tt = cfg["target_type"].as<std::string>();
        if (tt == "circles_grid")      calib_cfg.target_type = LiDARCameraCalibrator::TargetType::CIRCLES_GRID;
        else if (tt == "asym_circles") calib_cfg.target_type = LiDARCameraCalibrator::TargetType::ASYMMETRIC_CIRCLES;
        else                            calib_cfg.target_type = LiDARCameraCalibrator::TargetType::CHESSBOARD;
    }
    if (cfg["circle_diameter_m"]) calib_cfg.circle_diameter_m = cfg["circle_diameter_m"].as<double>();
    if (cfg["target_min_corners_per_frame"]) calib_cfg.target_min_corners_per_frame = cfg["target_min_corners_per_frame"].as<int>();
    if (cfg["target_min_frames"]) calib_cfg.target_min_frames = cfg["target_min_frames"].as<int>();
    if (cfg["target_use_coarse_rotation_search"]) calib_cfg.target_use_coarse_rotation_search = cfg["target_use_coarse_rotation_search"].as<bool>();
    if (cfg["target_coarse_rotation_range_deg"]) calib_cfg.target_coarse_rotation_range_deg = cfg["target_coarse_rotation_range_deg"].as<double>();
    if (cfg["target_coarse_rotation_step_deg"]) calib_cfg.target_coarse_rotation_step_deg = cfg["target_coarse_rotation_step_deg"].as<double>();
    if (cfg["target_coarse_inlier_threshold_px"]) calib_cfg.target_coarse_inlier_threshold_px = cfg["target_coarse_inlier_threshold_px"].as<double>();
    if (cfg["target_per_frame_rms_threshold_px"]) calib_cfg.target_per_frame_rms_threshold_px = cfg["target_per_frame_rms_threshold_px"].as<double>();
    if (cfg["target_ba_max_iter"]) calib_cfg.target_ba_max_iter = cfg["target_ba_max_iter"].as<int>();
    if (cfg["target_ba_use_robust_loss"]) calib_cfg.target_ba_use_robust_loss = cfg["target_ba_use_robust_loss"].as<bool>();
    if (cfg["target_ba_huber_scale_px"]) calib_cfg.target_ba_huber_scale_px = cfg["target_ba_huber_scale_px"].as<double>();
    if (cfg["edge_canny_low"])  calib_cfg.edge_canny_low  = cfg["edge_canny_low"].as<int>();
    if (cfg["edge_canny_high"]) calib_cfg.edge_canny_high = cfg["edge_canny_high"].as<int>();
    if (cfg["ceres_max_iter"])  calib_cfg.ceres_max_iter  = cfg["ceres_max_iter"].as<int>();
    if (cfg["frame_sync_threshold_s"]) calib_cfg.frame_sync_threshold_s = cfg["frame_sync_threshold_s"].as<double>();
    if (cfg["time_offset_search_range_s"]) calib_cfg.time_offset_search_range_s = cfg["time_offset_search_range_s"].as<double>();
    if (cfg["ncc_threshold"]) calib_cfg.ncc_threshold = cfg["ncc_threshold"].as<double>();
    if (cfg["ncc_low_skip_threshold"])  calib_cfg.ncc_low_skip_threshold  = cfg["ncc_low_skip_threshold"].as<double>();
    if (cfg["lidar_camera"] && cfg["lidar_camera"]["fine"]) {
        const auto& fine = cfg["lidar_camera"]["fine"];
        if (fine["multi_feature"]) {
            const auto& mf = fine["multi_feature"];
            if (mf["edge_weight"])   calib_cfg.edge_weight   = mf["edge_weight"].as<double>();
            if (mf["corner_weight"]) calib_cfg.corner_weight = mf["corner_weight"].as<double>();
            if (mf["intensity_weight"]) calib_cfg.intensity_weight = mf["intensity_weight"].as<double>();
            if (mf["corner_max_per_frame"]) calib_cfg.corner_max_per_frame = mf["corner_max_per_frame"].as<int>();
        }
        if (fine["robust_loss"]) {
            const auto& rl = fine["robust_loss"];
            if (rl["use"])       calib_cfg.use_robust_loss = rl["use"].as<bool>();
            if (rl["type"])     calib_cfg.robust_loss_type = rl["type"].as<std::string>();
            if (rl["threshold"]) calib_cfg.robust_loss_threshold = rl["threshold"].as<double>();
        }
    }
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");

    LiDARCameraCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& step, double prog) {
        if (prog >= 0)
            UNICALIB_INFO("  [进度] {}: {:.1f}%", step, prog * 100.0);
    });

    if (!data_ready) {
        UNICALIB_WARN("数据路径未就绪，当前运行将仅执行流水线占位阶段，不进行实际标定优化。详见上方【数据就绪】原因。");
    }

    // ─────────────────────────────────────────────────────────────────
    // Stage 3: 手动校准 (可选, 当精标定质量不足时)
    // ─────────────────────────────────────────────────────────────────
    if (do_manual) {
        UNICALIB_INFO("▶ Stage 3: 手动校准 (manual_threshold={:.2f}px)",
                      manual_rms_thresh);
        UNICALIB_INFO("  使用 ManualCalibSession 进行 6-DOF 交互式调整");
        UNICALIB_INFO("  调用示例:");
        UNICALIB_INFO("    ManualCalibSession::SessionConfig sess_cfg;");
        UNICALIB_INFO("    sess_cfg.save_dir = \"{}/manual_sessions\";", output_dir);
        UNICALIB_INFO("    ManualCalibSession session(sess_cfg);");
        UNICALIB_INFO("    auto result = session.run_lidar_cam(scan, image, intrin, auto_result, auto_rms);");
    }

    // ─────────────────────────────────────────────────────────────────
    // 注入 Stage 1 粗标定初值，使 Pipeline 内 Coarse-AI 跳过 MIAS-LCEC，精标定使用同一初值
    // ─────────────────────────────────────────────────────────────────
    if (coarse_init.has_value())
        pipeline.set_coarse_lidar_cam_init(coarse_init);

    // ─────────────────────────────────────────────────────────────────
    // 运行流水线日志摘要
    // ─────────────────────────────────────────────────────────────────
    auto report = pipeline.run();
    report.print_summary();

    bool any_placeholder = false;
    for (const auto& r : report.stage_results) {
        if (r.message.find("占位") != std::string::npos) { any_placeholder = true; break; }
    }
    if (any_placeholder) {
        UNICALIB_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        UNICALIB_WARN("本次未执行实际标定: 精标定阶段为占位实现，未加载点云/图像也未调用标定器。");
        UNICALIB_WARN("请确保: 1) 配置 data.lidar.<id> 与 data.camera.<id>.images_dir (或扁平键); 2) 设置 CALIB_DATA_DIR 或 --data-dir; 3) 数据目录存在且含 PCD/图像。");
        UNICALIB_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("LiDAR-Camera 标定流程完成");
    UNICALIB_INFO("结果目录: {}", output_dir);
    UNICALIB_INFO("标定流程正常退出 exit_code=0");
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_MAIN_TRY_END(0)
}
