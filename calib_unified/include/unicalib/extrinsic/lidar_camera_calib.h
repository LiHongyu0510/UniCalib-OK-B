#pragma once
/**
 * UniCalib Unified — LiDAR-Camera 外参标定
 *
 * 两阶段标定流程:
 *   粗标定 (Coarse): MIAS-LCEC AI 模型 (跨模态掩码匹配 C3M)
 *                    → 提供初始外参估计
 *   精标定 (Fine): 无目标优先
 *     Method A (运动法, B样条): 需要 IMU-LiDAR 外参已知
 *     Method B (目标法, 棋盘格): 3D-2D 对应点 PnP
 *     Method C (无目标边缘法, 优先): LiDAR强度图边缘 vs 相机边缘 — 最大化互信息
 *   手动校准 (Manual): 6-DOF 交互式调整 + click-calib 点击精化
 *
 * 日志: 每个阶段有独立 spdlog 日志, 记录迭代残差/收敛状态
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/viz/calib_visualizer.h"
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <optional>
#include <functional>
#include <memory>

namespace ns_unicalib {

// ===================================================================
// 3D-2D 对应点
// ===================================================================
struct Point3D2DCorr {
    Eigen::Vector3d point_3d;   // LiDAR 坐标系下的 3D 点
    Eigen::Vector2d point_2d;   // 图像坐标系下的 2D 点 [px]
    double weight = 1.0;
};

// ===================================================================
// 标定板检测结果
// ===================================================================
struct CalibBoardDetection {
    double timestamp;
    std::vector<Eigen::Vector3d> corners_3d;    // LiDAR 坐标系
    std::vector<Eigen::Vector2d> corners_2d;    // 图像坐标系
    cv::Mat image;
    bool valid = false;
};

// ===================================================================
// 边缘对齐评估
// ===================================================================
struct EdgeAlignmentScore {
    double mutual_information = 0.0;
    double edge_overlap_ratio = 0.0;
    double chamfer_distance   = 0.0;
    double ncc_score          = 0.0;  // 归一化互相关
    int num_lidar_edge_pts    = 0;
    int num_image_edge_pts    = 0;
};

// ===================================================================
// LiDAR-Camera 外参标定器
// ===================================================================
class LiDARCameraCalibrator {
public:
    enum class Method {
        MOTION_BSPLINE,     // 运动法 (需要 IMU 数据和 IMU-LiDAR 外参)
        TARGET_CHESSBOARD,  // 目标法 (标定板类型由 target_type 指定)
        EDGE_ALIGNMENT,     // 无目标边缘对齐
    };

    /** 标定板类型，与配置文件 target_type 对应：chessboard | circles_grid | asym_circles */
    enum class TargetType {
        CHESSBOARD,
        CIRCLES_GRID,
        ASYMMETRIC_CIRCLES,
    };

    struct Config {
        Method method = Method::TARGET_CHESSBOARD;

        /** 标定板类型（仅 method=TARGET_CHESSBOARD 时生效），默认棋盘格 */
        TargetType target_type = TargetType::CHESSBOARD;

        // 标定板尺寸（角点数/圆点数：cols×rows）
        int board_cols = 9, board_rows = 6;
        double square_size_m = 0.025;
        /** 圆点格可选：圆直径 [m]。>0 时可用于 LiDAR 端圆心精化或过滤，默认 0 表示不使用 */
        double circle_diameter_m = 0.0;

        // B样条运动法
        double spline_dt_s = 0.1;
        int    spline_order = 4;
        double time_offset_init_s = 0.0;
        bool   optimize_time_offset = true;
        // 时间偏移搜索范围：默认 0.5s，允许更大范围以处理不同步的数据
        double time_offset_search_range_s = 0.5;
        bool   optimize_intrinsic = false;  // 是否同时优化相机内参

        // 边缘对齐法
        int    edge_canny_low = 50;
        int    edge_canny_high = 150;
        double lidar_intensity_threshold = 10.0;
        int    optimizer_max_iter = 50;
        /** LiDAR-相机帧同步时间阈值 [s]，超过则丢弃该帧对，默认放宽到 0.5s */
        double frame_sync_threshold_s = 0.5;
        /** NCC 阈值：NCC 高于此值的帧才参与优化（默认 0.05） */
        double ncc_threshold = 0.05;
        /** NCC 低于此值的帧仅打诊断日志，不参与优化（可选，默认 -1e9 表示不启用） */
        double ncc_low_skip_threshold = -1e9;

        // 多特征融合精标定 (edge + corner + intensity，权重和宜为 1.0)
        double edge_weight = 0.5;
        double corner_weight = 0.3;
        double intensity_weight = 0.2;  // 强度一致性
        int    corner_max_per_frame = 150;  // 每帧最多参与优化的角点数

        // ========== 扩展特征约束 ==========
        /** 语义边缘特征权重 (基于深度学习边缘检测) */
        double semantic_edge_weight = 0.0;
        /** 深度边缘特征权重 (LiDAR 深度不连续处) */
        double depth_edge_weight = 0.0;
        /** 法向边缘特征权重 (基于点云法向变化) */
        double normal_edge_weight = 0.0;
        /** 语义边缘检测阈值 */
        double semantic_threshold = 0.5;
        /** 深度边缘检测阈值 [m] */
        double depth_edge_threshold_m = 0.1;
        /** 法向边缘角度阈值 [度] */
        double normal_edge_threshold_deg = 30.0;
        /** 特征提取下采样步长 (增大可提高速度) */
        int feature_sample_step = 1;

        // 鲁棒损失：降低外点影响
        bool   use_robust_loss = true;
        /** "huber" | "cauchy" | "none"，仅 use_robust_loss=true 时生效 */
        std::string robust_loss_type = "huber";
        double robust_loss_threshold = 2.0;  // Huber scale (px)，Cauchy 时为 scale

        // 运动补偿：对旋转式 LiDAR 进行运动畸变校正
        bool   enable_motion_compensation = false;
        /** 运动补偿方法: "none" | "linear" | "imu" */
        std::string motion_compensation_method = "none";
        double motion_compensation_max_time_s = 0.1;  // 最大补偿时间范围 [s]

        // ========== 混合标定配置 ==========
        /** 启用混合标定模式：深度学习粗标定 + 多分辨率精标定 */
        bool enable_hybrid_calibration = true;
        /** 粗标定置信度阈值，低于此值时触发 PnP 回退 */
        double coarse_confidence_threshold = 0.5;
        /** 多分辨率金字塔层数 */
        int pyramid_levels = 3;
        /** 每层迭代次数 */
        int pyramid_iterations_per_level = 20;
        /** 金字塔降采样因子 */
        double pyramid_downscale_factor = 0.5;
        /** 自适应策略：基于粗标定质量选择精标定方法 */
        bool adaptive_strategy = true;
        /** 质量评估权重 */
        double quality_ncc_weight = 0.6;
        double quality_edge_weight = 0.4;

        // 通用
        double max_reproj_error_px = 3.0;
        int    ceres_max_iter = 50;
        bool   verbose = true;

        // ========== 标定板目标法优化 (多帧 BA / 粗搜索 / 离群剔除) ==========
        /** 每帧最少角点数，低于则丢弃该帧；≥6 可支持部分可见标定板，默认 6 */
        int    target_min_corners_per_frame = 6;
        /** 至少保留的帧数（1=兼容单帧），低于则标定失败 */
        int    target_min_frames = 1;
        /** 无初值或初值为 identity 时是否进行粗旋转网格搜索；有初值时跳过 */
        bool   target_use_coarse_rotation_search = true;
        /** 粗旋转搜索范围 [度]，每轴 ±range */
        double target_coarse_rotation_range_deg = 5.0;
        /** 粗旋转搜索步长 [度] */
        double target_coarse_rotation_step_deg = 1.0;
        /** 粗搜索时投影内点判定距离阈值 [px] */
        double target_coarse_inlier_threshold_px = 8.0;
        /** 每帧重投影 RMS 超过此值则剔除该帧 [px]，≤0 表示不剔除 */
        double target_per_frame_rms_threshold_px = 5.0;
        /** 多帧重投影 BA 最大迭代次数（单帧也走 BA 精化） */
        int    target_ba_max_iter = 80;
        /** BA 是否使用鲁棒核（Huber），建议 true */
        bool   target_ba_use_robust_loss = true;
        /** BA Huber 尺度 [px] */
        double target_ba_huber_scale_px = 2.0;
    };

    explicit LiDARCameraCalibrator(const Config& cfg) : cfg_(cfg) {}
    LiDARCameraCalibrator() : cfg_(Config{}) {}

    // ===================================================================
    // 两阶段标定接口 (推荐使用)
    // ===================================================================

    // 两阶段结果 (粗标定 + 精标定)
    struct TwoStageResult {
        // 粗标定 (AI初始值)
        std::optional<ExtrinsicSE3> coarse;
        double coarse_rms = -1.0;
        std::string coarse_method;     // "MIAS-LCEC" / "edge_init" / "identity"

        // 精标定
        std::optional<ExtrinsicSE3> fine;
        double fine_rms = -1.0;
        std::string fine_method;       // "EDGE_ALIGNMENT" / "TARGET_CHESSBOARD" / "MOTION_BSPLINE"

        // 是否需要手动校准
        bool needs_manual = false;
        double manual_threshold_px = 2.0;

        // 最终推荐结果
        const ExtrinsicSE3* best() const {
            if (fine.has_value())   return &fine.value();
            if (coarse.has_value()) return &coarse.value();
            return nullptr;
        }
        double best_rms() const {
            if (fine_rms > 0)   return fine_rms;
            if (coarse_rms > 0) return coarse_rms;
            return -1.0;
        }
    };

    // 两阶段标定 (coarse_init 为 AI 粗估结果, 为空则用 identity 初始化)
    // prefer_targetfree=true 时优先使用边缘对齐 (无目标)
    TwoStageResult calibrate_two_stage(
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const CameraIntrinsics& cam_intrin,
        const std::optional<Sophus::SE3d>& coarse_init = std::nullopt,
        bool prefer_targetfree = true,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0");

    // 手动校准入口: 使用 click-calib 方式采集对应点后精化
    // 返回精化后的外参 (需要调用者提供对应点)
    std::optional<ExtrinsicSE3> calibrate_manual(
        const LiDARScan& scan,
        const cv::Mat& image,
        const CameraIntrinsics& cam_intrin,
        const ExtrinsicSE3& init_extrin,
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector2d>>& clicks_3d2d);

    // 日志辅助: 记录每次迭代的残差 (供 Ceres 回调调用)
    void log_iter_residual(int iter, double cost, double delta_cost = 0.0) const {
        UNICALIB_TRACE("[LiDAR-Cam] 迭代 {:3d}: cost={:.6f} delta={:.6f}",
                       iter, cost, delta_cost);
    }

    // ===================================================================
    // 单方法标定接口 (原有接口保留)
    // ===================================================================

    // 方法 A: 运动法 (B样条)
    // 需要: IMU 数据 + IMU-LiDAR 外参 + 相机图像序列
    std::optional<ExtrinsicSE3> calibrate_motion(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const ExtrinsicSE3& T_lidar_in_imu,
        const CameraIntrinsics& cam_intrin,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0");

    // 方法 B: 目标法 (标定板类型由 target_type 指定)
    // init_extrin: 可选初值；有初值且非 identity 时跳过粗旋转搜索，兼容单帧与多帧
    std::optional<ExtrinsicSE3> calibrate_target(
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const CameraIntrinsics& cam_intrin,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0",
        const std::optional<Sophus::SE3d>& init_extrin = std::nullopt);

    // 方法 C: 边缘对齐 (无目标)
    std::optional<ExtrinsicSE3> calibrate_edge_align(
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const CameraIntrinsics& cam_intrin,
        const Sophus::SE3d& init_guess,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0");

    // 评估标定质量 — 生成点云投影到图像上的可视化
    void visualize_projection(
        const LiDARScan& scan,
        const cv::Mat& image,
        const ExtrinsicSE3& extrin,
        const CameraIntrinsics& cam_intrin,
        const std::string& output_path);

    // 边缘对齐可视化：图像边缘(红) + LiDAR 投影边缘(绿) 叠加，比点云投影更易判读
    void visualize_edge_alignment(
        const LiDARScan& scan,
        const cv::Mat& image,
        const ExtrinsicSE3& extrin,
        const CameraIntrinsics& cam_intrin,
        const std::string& output_path,
        const EdgeAlignmentScore* score = nullptr,
        const char* verdict = nullptr);

    // BEV 俯视图：LiDAR XY 投影 + 指标文字，用于多传感器一致性快速验收
    void visualize_bev(
        const LiDARScan& scan,
        const ExtrinsicSE3& extrin,
        const std::string& output_path,
        double range_m = 30.0,
        int resolution = 512);

    // 计算边缘对齐评分
    EdgeAlignmentScore evaluate_edge_alignment(
        const LiDARScan& scan,
        const cv::Mat& image,
        const ExtrinsicSE3& extrin,
        const CameraIntrinsics& cam_intrin);

    using ProgressCallback = std::function<void(const std::string&, double)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = std::move(cb); }

    // 设置可视化器 (用于实时显示标定过程)
    void set_visualizer(CalibVisualizer::Ptr viz) { visualizer_ = viz; }

    // 获取可视化器
    CalibVisualizer::Ptr get_visualizer() const { return visualizer_; }

    // 启用/禁用实时可视化
    void enable_realtime_viz(bool enable) { enable_realtime_viz_ = enable; }

private:
    Config cfg_;
    ProgressCallback progress_cb_;
    CalibVisualizer::Ptr visualizer_;
    bool enable_realtime_viz_ = false;

    // 从 LiDAR 点云检测棋盘格角点
    bool detect_board_in_lidar(const LiDARScan& scan,
                               std::vector<Eigen::Vector3d>& corners_3d);

    // 从 LiDAR 点云检测圆点格圆心 (平面 + 网格单元质心)
    bool detect_circles_in_lidar(const LiDARScan& scan,
                                std::vector<Eigen::Vector3d>& centers_3d);

    // 生成 LiDAR 强度图 (用于边缘检测)
    cv::Mat lidar_to_intensity_image(
        const LiDARScan& scan,
        const Sophus::SE3d& T_cam_in_lidar,
        const CameraIntrinsics& cam_intrin);
};

}  // namespace ns_unicalib
