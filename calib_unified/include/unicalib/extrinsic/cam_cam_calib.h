#pragma once
/**
 * UniCalib Unified — Camera-Camera 外参标定
 *
 * 两阶段标定流程:
 *   粗标定 (Coarse): 特征匹配 + 本质矩阵初始化 (无目标优先)
 *   精标定 (Fine, 无目标优先):
 *     A. 棋盘格靶标法 (OpenCV stereoCalibrate) — 目标法
 *     B. 特征点匹配法 (ORB/SIFT + 本质矩阵)  — 无目标
 *     C. 束调整精化 (Bundle Adjustment with Ceres) — 无目标优先
 *   手动校准 (Manual): click-calib 风格 — 双目图像对应点采集 + 对极约束优化
 *
 * 算法来源: click_calib 思路 + iKalibr 视觉模块 + OpenCV
 * 日志: 每次 BA 迭代记录重投影误差 / 特征对数量 / 内点率
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include "unicalib/intrinsic/camera_calib.h"
#include "unicalib/viz/calib_visualizer.h"
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <optional>
#include <functional>

namespace ns_unicalib {

// ===================================================================
// 特征点匹配结果
// ===================================================================
struct FeatureMatch {
    Eigen::Vector2d pt_cam0;    // 相机0 图像坐标 [px]
    Eigen::Vector2d pt_cam1;    // 相机1 图像坐标 [px]
    cv::Point2f pt0;            // cv::Point2f (方便 OpenCV 调用)
    cv::Point2f pt1;
    double score = 1.0;         // 匹配置信度
};

// ===================================================================
// Bundle Adjustment 路标点
// ===================================================================
struct Landmark {
    int id;
    Eigen::Vector3d position_3d;    // 世界坐标系下 3D 位置
    bool fixed = false;

    // 各相机的 2D 观测
    std::map<int, Eigen::Vector2d> observations;  // cam_id -> 2D point
};

// ===================================================================
// Camera-Camera 外参标定器
// ===================================================================
class CamCamCalibrator {
public:
    enum class Method {
        CHESSBOARD_STEREO,  // OpenCV stereoCalibrate (最精确, 需要靶标)
        ESSENTIAL_MATRIX,   // 特征点 + 本质矩阵 (无靶标)
        BUNDLE_ADJUSTMENT,  // 特征点 + BA 精化 (无靶标)
    };

    struct Config {
        Method method = Method::CHESSBOARD_STEREO;

        // 棋盘格配置
        TargetConfig target;

        // 特征检测
        enum class FeatureType { ORB, SIFT, SURF } feature_type = FeatureType::ORB;
        int    num_features = 2000;
        double match_ratio  = 0.75;     // Lowe's ratio test
        bool   use_fundamental = false; // true=基本矩阵, false=本质矩阵 (需要内参)

        // RANSAC 参数（本质矩阵/基本矩阵）
        double ransac_reproj_threshold = 1.0;   // px（基本矩阵时）
        double ransac_confidence       = 0.999; // 置信度 (0..1)，用于 findEssentialMat
        double ransac_threshold_essential = 0.001; // 归一化坐标下到极线距离阈值
        int    ransac_max_iter = 1000;

        // 初值策略：有用户/上游初值时是否使用、是否跳过粗标定
        bool use_initial_if_available       = true;
        bool skip_coarse_when_initial_given = true;

        // Bundle Adjustment
        int    ba_max_iter = 100;
        double ba_huber_loss = 2.0;
        double ba_min_track_len = 3;    // 最短特征轨迹长度
        bool   ba_optimize_intrinsics = false;
        bool   use_full_multiview_ba = true;  // 多相机时使用完整多视图 BA（全局联合优化）
        size_t ba_min_tracks_multiview = 20;  // 多视图 BA 最少轨迹数，不足则退回两两标定
        size_t ba_min_points_multiview = 10; // 多视图 BA 三角化最少有效点数

        // 尺度约束 (参考: "Solving for Relative Pose with Constraints", CVPR 2014)
        // 通过已知距离 (如棋盘格尺寸) 消除尺度不确定性
        bool   enable_scale_constraint = false;    // 启用尺度约束
        double known_baseline_scale = 1.0;         // 已知尺度 (例如棋盘格宽度) [m]
        double scale_constraint_weight = 100.0;    // 尺度约束权重 (Ceres 优化)

        // 通用
        double max_rms_px = 2.0;
        bool   verbose = true;
    };

    explicit CamCamCalibrator(const Config& cfg) : cfg_(cfg) {}
    CamCamCalibrator() : cfg_(Config{}) {}

    // ===================================================================
    // 两阶段标定接口 (推荐使用)
    // ===================================================================

    struct TwoStageResult {
        std::optional<ExtrinsicSE3> coarse;
        double coarse_rms = -1.0;
        std::string coarse_method;    // "ESSENTIAL_MATRIX" / "ORB_MATCH"

        std::optional<ExtrinsicSE3> fine;
        double fine_rms = -1.0;
        std::string fine_method;      // "BUNDLE_ADJUSTMENT" / "CHESSBOARD_STEREO"

        bool needs_manual = false;
        double manual_threshold_px = 1.5;

        const ExtrinsicSE3* best() const {
            if (fine.has_value())   return &fine.value();
            if (coarse.has_value()) return &coarse.value();
            return nullptr;
        }
        double best_rms() const {
            return fine_rms > 0 ? fine_rms : coarse_rms;
        }
    };

    // 两阶段标定
    // 粗标定: 特征点匹配 + 本质矩阵（可被初值跳过）
    // 精标定: Bundle Adjustment (无目标优先) 或 棋盘格
    // init_extrin: 可选初值；有初值且 skip_coarse_when_initial_given 时跳过粗标定，直接用于 BA
    TwoStageResult calibrate_two_stage(
        const std::vector<std::pair<double, cv::Mat>>& frames_cam0,
        const std::vector<std::pair<double, cv::Mat>>& frames_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        bool prefer_targetfree = true,
        const std::string& cam0_id = "cam_0",
        const std::string& cam1_id = "cam_1",
        std::optional<ExtrinsicSE3> init_extrin = std::nullopt);

    // 手动校准: click-calib 风格对应点精化
    std::optional<ExtrinsicSE3> calibrate_manual(
        const cv::Mat& img0, const cv::Mat& img1,
        const CameraIntrinsics& intrin0, const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& init_extrin,
        const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& clicks_0_1);

    // 日志辅助
    void log_iter_residual(int iter, double cost) const {
        UNICALIB_TRACE("[Cam-Cam] 迭代 {:3d}: cost={:.6f}", iter, cost);
    }

    // ===================================================================
    // 单方法标定接口 (原有接口保留)
    // ===================================================================

    // 方法 A: 棋盘格立体标定
    // 假设两相机内参已知
    std::optional<ExtrinsicSE3> calibrate_stereo(
        const std::vector<std::string>& images_cam0,
        const std::vector<std::string>& images_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const std::string& cam0_id = "cam_0",
        const std::string& cam1_id = "cam_1");

    // 方法 A 重载: 从内存帧 (用于两阶段标定精化)
    std::optional<ExtrinsicSE3> calibrate_stereo(
        const std::vector<std::pair<double, cv::Mat>>& frames_cam0,
        const std::vector<std::pair<double, cv::Mat>>& frames_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const std::string& cam0_id = "cam_0",
        const std::string& cam1_id = "cam_1");

    // 方法 B: 本质矩阵
    std::optional<ExtrinsicSE3> calibrate_essential(
        const std::vector<std::pair<double, cv::Mat>>& frames_cam0,
        const std::vector<std::pair<double, cv::Mat>>& frames_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const std::string& cam0_id = "cam_0",
        const std::string& cam1_id = "cam_1");

    // 方法 C: Bundle Adjustment
    // 支持多相机 (N ≥ 2)，当前为两两标定 (cam0 为参考)
    std::vector<ExtrinsicSE3> calibrate_bundle_adjustment(
        const std::vector<std::vector<std::pair<double, cv::Mat>>>& frames_per_cam,
        const std::vector<CameraIntrinsics>& intrinsics,
        const std::vector<std::string>& cam_ids);

    // 方法 C': 完整多视图 Bundle Adjustment（全局联合优化）
    // 单次 Ceres 问题：所有外参 T_cam0_cami + 所有 3D 点，最小化全部重投影误差
    // 要求: 多相机、多帧、跨相机特征轨迹，适合标定 cam0-cam1-cam2-... 外参
    struct MultiViewBAResult {
        std::vector<ExtrinsicSE3> extrinsics;  // cam0 为参考，extrinsics[i] = T_cam0_cam(i+1)
        double final_cost = 0.0;
        int num_points = 0;
        int num_observations = 0;
        bool converged = false;
    };
    MultiViewBAResult calibrate_bundle_adjustment_full_multiview(
        const std::vector<std::vector<std::pair<double, cv::Mat>>>& frames_per_cam,
        const std::vector<CameraIntrinsics>& intrinsics,
        const std::vector<std::string>& cam_ids);

    // 验证外参: 计算重投影误差
    double evaluate(
        const std::vector<std::string>& images_cam0,
        const std::vector<std::string>& images_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& extrin);

    // 生成立体整流结果 (用于验证)
    void visualize_stereo_rectification(
        const cv::Mat& img0, const cv::Mat& img1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& extrin,
        const std::string& output_path);

    using ProgressCallback = std::function<void(const std::string&, double)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = std::move(cb); }

    // 设置可视化器 (用于实时显示标定过程)
    void set_visualizer(CalibVisualizer::Ptr viz) { visualizer_ = viz; }
    CalibVisualizer::Ptr get_visualizer() const { return visualizer_; }

    // 启用/禁用实时可视化
    void enable_realtime_viz(bool enable) { enable_realtime_viz_ = enable; }

    // 获取上次标定的路标点 (用于可视化/调试)
    const std::vector<Landmark>& get_landmarks() const { return landmarks_; }
    const std::vector<FeatureMatch>& get_last_matches() const { return last_matches_; }

private:
    Config cfg_;
    ProgressCallback progress_cb_;
    CalibVisualizer::Ptr visualizer_;
    bool enable_realtime_viz_ = false;
    std::vector<Landmark> landmarks_;
    std::vector<FeatureMatch> last_matches_;

    // 特征提取与匹配
    std::vector<FeatureMatch> match_features(
        const cv::Mat& img0, const cv::Mat& img1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1);

    // 从本质矩阵恢复 R, t
    bool recover_pose_from_E(
        const std::vector<FeatureMatch>& matches,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        Sophus::SO3d& rot, Eigen::Vector3d& trans);

    // Ceres Bundle Adjustment 精化
    ExtrinsicSE3 bundle_adjustment_two_views(
        const std::vector<FeatureMatch>& matches,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const Sophus::SE3d& init_T,
        const std::string& cam0_id,
        const std::string& cam1_id);

    // 仅两两标定（供多视图 BA 失败时回退，避免递归）
    std::vector<ExtrinsicSE3> calibrate_bundle_adjustment_pairwise_only(
        const std::vector<std::vector<std::pair<double, cv::Mat>>>& frames_per_cam,
        const std::vector<CameraIntrinsics>& intrinsics,
        const std::vector<std::string>& cam_ids);
};

}  // namespace ns_unicalib
