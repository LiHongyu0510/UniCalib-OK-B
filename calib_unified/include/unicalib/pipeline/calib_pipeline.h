#pragma once
/**
 * UniCalib Unified — 两阶段标定流水线
 *
 * 架构:
 *   Stage 1 (粗标定, Coarse): AI模型提供初始值
 *     - DM-Calib        → 相机内参粗估
 *     - MIAS-LCEC       → LiDAR-Camera 外参粗估 (跨模态掩码匹配)
 *     - Transformer-IMU → IMU 内参粗估
 *     - L2Calib         → IMU-LiDAR 外参粗估 (强化学习)
 *
 *   Stage 2 (精标定, Fine): 无目标优化为首选
 *     - IMU内参    → Allan方差分析
 *     - 相机内参   → 无目标 (DM-Calib 精化) / 棋盘格
 *     - IMU-LiDAR → B样条连续时间
 *     - LiDAR-Cam → 边缘对齐互信息 (无目标优先)
 *     - Cam-Cam   → Bundle Adjustment (无目标优先)
 *
 *   Stage 3 (手动校准, Manual, 可选):
 *     当自动标定精度不足时, 提供6-DOF交互式调整
 *
 * 日志系统:
 *   每个环节均有 TRACE/DEBUG/INFO/WARN/ERROR 层级日志
 *   支持阶段耗时统计 / 收敛状态 / 残差指标
 */

#include "unicalib/common/logger.h"
#include "unicalib/common/calib_param.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include <opencv2/core.hpp>
#include <chrono>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ns_unicalib {

// ===========================================================================
// 标定任务类型 (可灵活组合)
// ===========================================================================
enum class CalibTaskType : uint32_t {
    NONE            = 0x00,
    IMU_INTRINSIC   = 0x01,  // IMU 内参
    CAM_INTRINSIC   = 0x02,  // 相机内参
    IMU_LIDAR_EXTRIN = 0x04, // IMU-LiDAR 外参
    LIDAR_LIDAR_EXTRIN = 0x08, // LiDAR-LiDAR 外参
    LIDAR_CAM_EXTRIN = 0x10, // LiDAR-Camera 外参
    CAM_CAM_EXTRIN  = 0x20,  // Camera-Camera 外参
    ALL             = 0x3F,  // 所有
};

inline CalibTaskType operator|(CalibTaskType a, CalibTaskType b) {
    return static_cast<CalibTaskType>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline bool has_task(CalibTaskType tasks, CalibTaskType check) {
    return (static_cast<uint32_t>(tasks) & static_cast<uint32_t>(check)) != 0;
}

// ===========================================================================
// 标定阶段枚举
// ===========================================================================
enum class CalibStage {
    COARSE_AI,      // AI 粗标定
    FINE_AUTO,      // 自动精标定
    MANUAL_REFINE,  // 手动精校准
};

inline const char* stage_name(CalibStage s) {
    switch (s) {
        case CalibStage::COARSE_AI:     return "Coarse-AI";
        case CalibStage::FINE_AUTO:     return "Fine-Auto";
        case CalibStage::MANUAL_REFINE: return "Manual-Refine";
    }
    return "Unknown";
}

// ===========================================================================
// 阶段执行结果
// ===========================================================================
struct StageResult {
    CalibStage stage;
    CalibTaskType task;
    bool success = false;
    double residual_rms = 0.0;       // 残差均方根
    double elapsed_ms   = 0.0;       // 耗时 [ms]
    std::string message;             // 人类可读状态描述
    std::string log_file;            // 本阶段详细日志路径

    // 质量阈值 (供下游判断是否需要手动校准)
    double quality_threshold = -1.0; // < 0 表示不评估
    bool needs_manual_refine() const {
        return quality_threshold > 0.0 && residual_rms > quality_threshold;
    }
};

// ===========================================================================
// 流水线总结报告
// ===========================================================================
struct PipelineReport {
    std::string pipeline_id;                    // e.g. "lidar_cam_20260303_142305"
    std::vector<StageResult> stage_results;
    CalibParamManager::Ptr final_params;

    // 指标摘要
    bool all_converged() const {
        for (const auto& r : stage_results) {
            if (!r.success) return false;
        }
        return true;
    }
    double total_elapsed_ms() const {
        double total = 0;
        for (const auto& r : stage_results) total += r.elapsed_ms;
        return total;
    }
    void print_summary() const;
    void save_report(const std::string& path) const;
};

// ===========================================================================
// 进度回调
// ===========================================================================
using StageProgressCb = std::function<void(
    CalibStage stage, const std::string& step, double progress_0_1)>;

// ===========================================================================
// 流水线配置
// ===========================================================================
struct PipelineConfig {
    // 要执行的任务组合
    CalibTaskType tasks = CalibTaskType::ALL;

    // 是否启用 AI 粗标定 (各子项)
    bool enable_coarse_imu_intrin   = true;  // Transformer-IMU-Calibrator
    bool enable_coarse_cam_intrin   = true;  // DM-Calib
    bool enable_coarse_imu_lidar    = true;  // learn-to-calibrate (L2Calib)
    bool enable_coarse_lidar_cam    = true;  // MIAS-LCEC
    bool enable_coarse_cam_cam      = false; // 暂无专用模型, 用特征匹配代替

    // Python 解释器路径 (用于调用 AI 模型)
    std::string python_executable   = "python3";
    // AI 模型根目录 (相对于 workspace)
    std::string ai_models_root      = "../";

    // 是否允许手动校准降级 (当精标定残差超过阈值时提示用户)
    bool allow_manual_fallback      = true;
    // 手动校准时使用 Pangolin 点云叠加+面板 (方案 B)；false 则用 OpenCV 叠加窗
    bool use_pangolin_manual_panel  = false;

    // 残差质量阈值 (超过则建议手动校准)
    double lidar_cam_rms_threshold  = 2.0;   // px
    double cam_cam_rms_threshold    = 1.5;   // px
    // 相机-相机：若 params 中已有该对的外参则作为初值传入；有初值时是否跳过粗标定
    bool use_cam_cam_initial_from_params = true;
    bool use_cam_cam_skip_coarse_when_initial = true;
    double imu_lidar_rot_threshold  = 0.5;   // deg
    double imu_intrin_rms_threshold = 0.1;   // Allan拟合残差阈值 [rad/s]

    // 输出目录
    std::string output_dir          = "./results";

    // 日志级别: trace/debug/info/warn/error
    std::string log_level           = "info";

    // 无目标优先 (false 则允许使用靶标方法)
    bool prefer_targetfree          = true;

    // 默认启动可视化界面 (LiDAR-Camera 等外参标定时；可用 --no-viz 关闭)
    bool enable_viz                 = true;

    // ─── 数据路径配置 (LiDAR-Camera 标定) ───
    std::string lidar_data_dir;              // LiDAR 点云目录 (PCD)
    std::string camera_images_dir;           // 相机图像目录（单目时用）
    std::string camera_intrinsic_file;       // 相机内参 YAML (可选，单目时用)
    std::string lidar_id        = "lidar_front";
    std::string camera_id       = "cam_left";
    // 多相机 LiDAR-Camera：对列表中每个相机分别做 lidar↔camera 标定（空则仅用 camera_id）
    std::vector<std::string> lidar_camera_camera_list;
    // 各相机图像目录 (camera_id -> 路径)，多相机 lidar-cam 文件模式时用
    std::map<std::string, std::string> camera_images_dirs;
    // Camera-Camera 标定对：[(id0, id1), ...]，空则按 camera_topics 顺序两两标定
    std::vector<std::pair<std::string, std::string>> cam_cam_pairs;
    /** Cam-Cam 每对初值（可选）：key="ref__target" 或 "T_ref__target"，与 pairs 中 [ref,target] 一致；value=4×4 行优先 16 个数 */
    std::map<std::string, std::vector<double>> cam_cam_initial_extrinsic_inline;

    // ─── 多传感器话题（全可配置）：sensor_id -> ROS2 话题；非空时用于数据加载与标定
    std::map<std::string, std::string> lidar_topics;
    std::map<std::string, std::string> camera_topics;
    std::map<std::string, std::string> imu_topics;
    // 多相机内参文件 (sensor_id -> yaml 路径)，cam-cam 时若 params 中无内参则按需加载
    std::map<std::string, std::string> camera_intrinsic_files;
    // 多 IMU 内参文件 (sensor_id -> yaml 路径)；外参标定且未做 IMU 内参时从 results/imu_intrinsic/ 加载
    std::map<std::string, std::string> imu_intrinsic_files;
    // results 子目录名：非空时 IMU/相机内参写入 output_dir/<该名>/；与配置 results.imu_intrinsic / results.camera_intrinsic 一致
    std::string results_imu_intrinsic;
    std::string results_camera_intrinsic;

    // ─── 全自动标定：为 true 时根据 sensors 数量自动开启对应任务（见下方说明）
    bool auto_tasks = false;
    // auto_tasks 时：有 IMU -> do_imu_intrinsic；有相机 -> do_camera_intrinsic；
    // 相机≥2 -> do_cam_cam_extrinsic；有 IMU+LiDAR -> do_imu_lidar_extrinsic；有 LiDAR+相机 -> do_lidar_camera_extrinsic

    // ─── ROS2 数据源配置 (新增) ───
    bool use_ros2_bag = false;             // 是否使用 ROS2 bag 文件
    std::string ros2_bag_file;             // ROS2 bag 文件路径
    bool use_ros2_topics = false;          // 是否使用 ROS2 实时话题订阅
    std::string lidar_ros2_topic;          // LiDAR ROS2 话题 (如 /velodyne_points)
    std::string camera_ros2_topic;         // 相机 ROS2 话题 (如 /cam_left/image_raw)
    std::string imu_ros2_topic;           // IMU ROS2 话题 (如 /imu/data)
    double ros2_max_wait_time = 30.0;     // ROS2 实时模式最大等待时间(秒)
    double ros2_sample_interval = 0.0;     // ROS2 数据采样间隔(秒)
    size_t ros2_max_frames = 100;         // ROS2 最大帧数限制
    bool   ros2_strict_topic_match = true; // 为 true 时配置话题在 bag 中无数据则加载失败，不自动回退

    // ─── 新采集格式数据源（索引 CSV） ───
    bool use_new_format = false;
    std::string new_format_root_dir;
    std::map<std::string, std::string> new_format_lidar_index_files;   // sensor_id -> csv
    std::map<std::string, std::string> new_format_camera_index_files;  // sensor_id -> csv
    std::map<std::string, std::string> new_format_imu_index_files;     // sensor_id -> csv
    std::string new_format_timestamp_unit = "s";  // s | ms | us | ns
    double new_format_oem7_imu_rate_hz = 200.0;   // OEM7 CORRIMUDATAS 解码（Hz）
    std::string new_format_oem7_time_base = "gps";
    int new_format_oem7_gps_utc_leap_sec = 18;
    double new_format_oem7_time_offset_sec = 0.0;

    // ─── LiDAR-Camera 标定参数 ───
    // 方法: "edge"(无目标边缘对齐) | "target"(标定板) | "motion"(B样条运动)
    std::string lidar_cam_method = "edge";

    // 标定板类型 (method=target 时使用): "chessboard" | "circles_grid" | "asym_circles"
    std::string lidar_cam_target_type = "chessboard";

    // 标定板尺寸 (角点/圆点数 cols×rows)
    int    board_cols      = 9;
    int    board_rows      = 6;
    double square_size_m   = 0.025;

    // 标定板目标法优化 (多帧 BA / 粗搜索 / 离群剔除)
    int    target_min_corners_per_frame = 6;
    int    target_min_frames = 1;
    bool   target_use_coarse_rotation_search = true;
    double target_coarse_rotation_range_deg = 5.0;
    double target_coarse_rotation_step_deg = 1.0;
    double target_coarse_inlier_threshold_px = 8.0;
    double target_per_frame_rms_threshold_px = 5.0;
    int    target_ba_max_iter = 80;
    bool   target_ba_use_robust_loss = true;
    double target_ba_huber_scale_px = 2.0;

    // 边缘对齐参数 (method=edge 时使用)
    int    edge_canny_low  = 50;
    int    edge_canny_high = 150;
    int    ceres_max_iter  = 50;
    double frame_sync_threshold_s = 0.5;   // LiDAR-相机帧同步时间阈值 [s]，放宽到 0.5s
    double ncc_threshold = 0.05;           // NCC 阈值：NCC > 此值才参与优化
    double ncc_low_skip_threshold  = -1e9; // NCC 低于此值可跳过（-1e9 表示不启用）
    bool   lidar_cam_optimize_time_offset = true;  // 来自 lidar_camera.optimize_time_offset
    double time_offset_search_range_s = 0.5;  // 时间偏移搜索范围 [s]

    // 多特征融合精标定 (lidar_camera.fine)
    double lidar_cam_edge_weight   = 0.5;
    double lidar_cam_corner_weight = 0.3;
    double lidar_cam_intensity_weight = 0.2;
    int    lidar_cam_corner_max_per_frame = 150;
    bool   lidar_cam_use_robust_loss = true;
    std::string lidar_cam_robust_loss_type = "huber";
    double lidar_cam_robust_loss_threshold = 2.0;
    // 运动补偿参数
    bool   lidar_cam_motion_compensation_enable = false;
    std::string lidar_cam_motion_compensation_method = "none";

    // ─── LiDAR-Camera 粗标定(C3M) 配置（来自 lidar_camera.coarse / initial_guess） ───
    // 注意：这些参数仅用于粗标定 Python 脚本 run_lidar_cam_coarse.py（MIAS-LCEC / 纯 PyTorch）侧。
    bool   lidar_cam_c3m_use_iterative_refine = false;
    int    lidar_cam_c3m_iter_max = 0;                 // 0=Python 默认
    double lidar_cam_c3m_iter_thresh = 0.0;            // 0=Python 默认
    double lidar_cam_c3m_similarity_threshold = 0.0;   // 0=Python 默认

    // ─── LiDAR-Camera 质量评估阈值（来自 lidar_camera.quality） ───
    double lidar_cam_quality_ncc_good = 0.3;
    double lidar_cam_quality_ncc_acceptable = 0.2;
    double lidar_cam_quality_rms_good_px = 2.0;
    double lidar_cam_quality_rms_acceptable_px = 5.0;
    double lidar_cam_quality_inlier_ratio_good = 0.6;
    double lidar_cam_quality_inlier_ratio_acceptable = 0.4;

    // ─── IMU 内参标定配置 ───
    std::string imu_sensor_id   = "imu_0";  // IMU传感器ID
    std::string imu_data_file;               // CSV文件路径 (备用)
    ns_unicalib::IMUIntrinsicCalibrator::Config imu_intrinsic_calib_cfg;
    
    // ─── 标定对配置 (从 YAML pairs 读取) ───
    std::vector<std::pair<std::string, std::string>> imu_lidar_pairs;      // IMU-LiDAR 标定对
    // IMU-LiDAR 初值：标定在初值基础上精化。支持两种形式（可混用）：
    // 1) 文件路径：key = "imu_id__lidar_id" 或 "T_imu_id__lidar_id"，value = YAML 文件路径
    // 2) 内联 4x4：key 同上，value = 含 rows/cols/data 的 YAML 对象（在 app 中解析为 16 个 double）
    std::map<std::string, std::string> imu_lidar_initial_extrinsic_files;
    std::map<std::string, std::vector<double>> imu_lidar_initial_extrinsic_inline;  // key -> 4x4 行优先 16 个数
    // IMU-LiDAR 手眼 180° 歧义策略（与 imu_lidar.handeye_* 一致；空串表示用 pipeline 默认）
    std::string imu_lidar_handeye_180_decision;                             // residual | prefer_identity | prefer_180
    bool        imu_lidar_handeye_prefer_identity_when_ambiguous = true;
    double      imu_lidar_handeye_180_residual_margin_deg = 3.0;
    std::vector<std::pair<std::string, std::string>> lidar_lidar_pairs;    // LiDAR-LiDAR 标定对
    std::vector<std::pair<std::string, std::string>> lidar_camera_pairs;   // LiDAR-Camera 标定对
    // cam_cam_pairs 见上方已有定义

    // 参考 IMU ID (多 IMU 时用于确定参考系)
    std::string reference_imu;
    
    // ─── Third Party / AI 配置 ───
    std::string dm_calib_model = "DM-Calib/model/calib2";
    bool dm_calib_use_cpu = true;
    int dm_calib_timeout_sec = 6000;
    int dm_calib_denoise_steps = 10;
    int dm_calib_ensemble_size = 1;
    int dm_calib_processing_res = 512;
    int dm_calib_max_images = 25;
    // MIAS-LCEC 粗标定（LiDAR-Cam）
    std::string mias_lcec_repo_dir;
    std::string mias_lcec_calib_script = "scripts/run_lidar_cam_coarse.py";
    std::string mias_lcec_model_path;   // 可选：Overlap Transformer 等模型路径，有则传脚本优先深度模型
    bool        mias_lcec_allow_pnp_fallback = true;  // 已配置 model_path 但推理不可用时允许回退 PnP
    std::string mias_lcec_python_exe   = "python3";
    int mias_lcec_timeout_sec = 180;
    std::string mias_lcec_work_dir     = "/tmp/mias_work";
    /** LiDAR-Camera 粗标定初值（来自配置 initial_extrinsic/T_cam_lidar）：作为粗标定输入，粗标定必执行，其结果为精标定初值 */
    std::optional<Sophus::SE3d> lidar_cam_coarse_initial;
    /** LiDAR-Camera 每对初值（可选）：key="lidar_id__camera_id" 或 "T_lidar_id__camera_id"，value=4x4 行优先 16 个数 */
    std::map<std::string, std::vector<double>> lidar_camera_initial_extrinsic_inline;
    /** 为 true 时：跳过 AI 粗标定与精标定，直接使用配置中的 initial_extrinsic 作为手动微调的起始值（需同时配置 initial_extrinsic 且通常配合 --manual） */
    bool lidar_cam_use_config_extrinsic_only = false;
};

// ===========================================================================
// 两阶段标定流水线主类
// ===========================================================================
class CalibPipeline {
public:
    using Ptr = std::shared_ptr<CalibPipeline>;

    explicit CalibPipeline(const PipelineConfig& cfg);
    CalibPipeline();

    // -----------------------------------------------------------------------
    // 初始化参数管理器 (从外部传入或内部创建)
    // -----------------------------------------------------------------------
    void set_param_manager(CalibParamManager::Ptr pm) { params_ = pm; }
    CalibParamManager::Ptr get_param_manager() const { return params_; }

    // -----------------------------------------------------------------------
    // 进度回调 (UI/日志集成)
    // -----------------------------------------------------------------------
    void set_progress_callback(StageProgressCb cb) { progress_cb_ = cb; }

    // -----------------------------------------------------------------------
    // 注入已有粗标定初值 (主程序 Stage 1 完成后调用，Pipeline Coarse-AI 将跳过 MIAS-LCEC)
    // -----------------------------------------------------------------------
    void set_coarse_lidar_cam_init(std::optional<Sophus::SE3d> init) { coarse_lidar_cam_init_ = std::move(init); }

    /**
     * 设置 LiDAR-LiDAR 手动阶段使用的首帧匹配数据（ref/target 各第 0 帧）。
     * 独立应用 unicalib_lidar_lidar 在进入手动前调用，或 run_fine_lidar_lidar 内部写入。
     */
    void set_manual_lidar_lidar_first_frame(const LiDARScan& ref_first,
                                            const LiDARScan& target_first,
                                            const std::string& ref_id,
                                            const std::string& target_id);

    // -----------------------------------------------------------------------
    // 阶段日志记录 (每个环节细粒度记录)
    // -----------------------------------------------------------------------
    void log_stage_begin(CalibStage stage, CalibTaskType task,
                         const std::string& detail = "");
    void log_stage_end(const StageResult& result);
    void log_step(CalibStage stage, const std::string& step,
                  const std::string& msg, spdlog::level::level_enum lv
                      = spdlog::level::info);
    void log_metric(const std::string& name, double value,
                    const std::string& unit = "");
    void log_param_change(const std::string& param_name,
                          const std::string& before,
                          const std::string& after);

    // -----------------------------------------------------------------------
    // 运行完整流水线
    // -----------------------------------------------------------------------
    PipelineReport run();

    // -----------------------------------------------------------------------
    // 单阶段入口 (细粒度控制)
    // -----------------------------------------------------------------------
    StageResult run_coarse_stage(CalibTaskType task);
    StageResult run_fine_stage(CalibTaskType task);
    StageResult run_manual_stage(CalibTaskType task);

    // -----------------------------------------------------------------------
    // 获取最终报告
    // -----------------------------------------------------------------------
    const PipelineReport& get_report() const { return report_; }

protected:
    // -----------------------------------------------------------------------
    // 内部工具
    // -----------------------------------------------------------------------
    std::string make_stage_log_path(CalibStage stage, CalibTaskType task) const;
    void setup_stage_logger(const std::string& log_path);

    using Clock = std::chrono::steady_clock;
    Clock::time_point stage_start_;

    PipelineConfig cfg_;
    CalibParamManager::Ptr params_;
    StageProgressCb progress_cb_;
    PipelineReport report_;

    // 阶段子日志 (每个 stage 独立文件)
    std::map<std::string, std::shared_ptr<spdlog::logger>> stage_loggers_;

    // 粗标定结果缓存（run_coarse_stage 写入，run_fine_* 读取）
    std::optional<Sophus::SE3d> coarse_lidar_cam_init_;

    // 手动阶段用：精标定首帧匹配数据（仅第一帧供手动调整；run_fine_* 或 set_manual_lidar_lidar_first_frame 写入，run_manual_stage 读取）
    struct ManualStageCache {
        std::optional<LiDARScan> lidar_scan;
        std::optional<cv::Mat> camera_image;
        std::string lidar_id;
        std::string camera_id;
        std::optional<CameraIntrinsics> camera_intrin;
        std::optional<cv::Mat> cam0_image;
        std::optional<cv::Mat> cam1_image;
        std::string cam0_id;
        std::string cam1_id;
        std::optional<CameraIntrinsics> cam0_intrin;
        std::optional<CameraIntrinsics> cam1_intrin;
        // LiDAR-LiDAR 首帧匹配数据（ref/target 各第 0 帧）
        std::optional<LiDARScan> lidar_lidar_ref_scan;
        std::optional<LiDARScan> lidar_lidar_target_scan;
        std::string lidar_lidar_ref_id;
        std::string lidar_lidar_target_id;
    } manual_cache_;

    // -----------------------------------------------------------------------
    // LiDAR-Camera / Camera-Camera 精标定内部实现
    // -----------------------------------------------------------------------
    StageResult run_fine_lidar_camera();
    StageResult run_fine_cam_cam();
    StageResult run_fine_imu_intrinsic();  // IMU 内参精标定
    StageResult run_fine_imu_lidar();      // IMU-LiDAR 外参精标定（支持 imu_lidar.pairs 多对）

    // 从图像推断内参 (无内参文件时使用)
    CameraIntrinsics infer_intrinsics_from_images(
        const std::vector<std::pair<double, cv::Mat>>& frames) const;

    // 保存外参结果到 YAML
    void save_extrinsic_result(
        const std::string& path,
        const LiDARCameraCalibrator::TwoStageResult& result,
        const std::string& target_camera_id = "") const;

    // 保存 Cam-Cam 外参结果到 YAML
    void save_cam_cam_extrinsic_result(
        const std::string& path,
        const CamCamCalibrator::TwoStageResult& result,
        const std::string& cam0_id,
        const std::string& cam1_id) const;

    // 保存 IMU 内参结果到 YAML
    void save_imu_intrinsic_yaml(const ns_unicalib::IMUIntrinsics& intrinsics,
                                 const std::string& path) const;
};

}  // namespace ns_unicalib
