#pragma once
/**
 * UniCalib Unified — LiDAR-LiDAR 外参标定
 *
 * 两阶段标定流程 (基于 Multi-LiCa/GMMCalib 2024 研究):
 *   粗标定 (Coarse):
 *     Step 1: FPFH 特征提取 (Fast Point Feature Histograms)
 *     Step 2: TEASER++ 鲁棒匹配 (截断最小二乘, 抗 outlier)
 *     Step 3: GMM 联合配准 (可选, 高斯混合模型联合优化)
 *   精标定 (Fine):
 *     Step 1: GICP 精细配准 (点-面距离)
 *     Step 2: B样条连续时间优化 (多帧联合, 轨迹平滑约束)
 *   手动校准 (Manual):
 *     可视化验证 + 6-DOF 增量调整
 *
 * 适用场景:
 *   - 静止场景 FOV 重叠: Multi-LiCa 方法 (FPFH+TEASER++/GMM)
 *   - 动态场景 IMU 辅助: B样条运动约束
 *   - 无重叠 FOV: 需要级联标定或第三方参考
 *
 * 精度目标: <3cm 平移, <0.5deg 旋转
 *
 * 参考文献:
 *   [1] Multi-LiCa (TUM 2024): arXiv:2501.11088
 *   [2] GMMCalib (2024): arXiv:2404.03427
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include "unicalib/viz/calib_visualizer.h"
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <string>
#include <functional>
#include <optional>
#include <memory>

// PCL 类型 (仅点云与智能指针，供私有实现使用)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ns_unicalib {

// 前向声明 LiDARScan (在 imu_lidar_calib.h 中定义)
struct LiDARScan;

// ===================================================================
// LiDAR-LiDAR 标定配置
// ===================================================================
struct LiDARLiDARConfig {
    // ─── 粗标定参数 ───────────────────────────────────────────────────
    // 粗标定路径: true=FPFH+鲁棒位姿(TEASER++/RANSAC), false=多帧合并+NDT
    bool   use_fpfh_teaser_coarse = true;

    // FPFH 特征提取
    double fpfh_radius = 0.5;           // FPFH 特征计算半径 [m]
    double fpfh_normal_radius = 0.3;    // 法向量估计半径 [m]

    // TEASER++ / RANSAC 鲁棒匹配
    double teaser_noise_bound = 0.1;    // 噪声边界 [m]，RANSAC 时作 inlier 距离阈值
    double teaser_cbar2 = 1.0;          // 卡方阈值平方
    int    teaser_max_clique_iters = 100;
    int    ransac_max_iterations = 500; // FPFH 对应点 RANSAC 最大迭代（无 TEASER++ 时）

    // GMM 联合配准 (P2 预留，当前为多帧合并+NDT)
    bool   use_gmm_registration = false;
    int    gmm_max_iterations = 50;     // GMM 最大迭代次数
    double gmm_tolerance = 1e-6;        // GMM 收敛容差
    int    gmm_num_components = 10;     // GMM 组件数量

    // ─── 精标定参数 ───────────────────────────────────────────────────
    // GICP 精细配准
    double gicp_max_corr_dist = 1.0;    // 最大对应点距离 [m]
    int    gicp_max_iterations = 30;    // GICP 最大迭代次数
    double gicp_transformation_epsilon = 1e-8;
    double gicp_euclidean_fitness_epsilon = 1e-6;

    // P1: 多帧 GICP 融合 ("single"=仅合并后单次, "median"=逐帧后中值, "weighted"=按 fitness 加权)
    bool   use_multi_frame_fine = true;
    int    fine_fusion_max_frames = 20; // 参与融合的最大帧数
    std::string fine_fusion_method = "median";

    // NDT 配准 (GICP 替代方案)
    bool   use_ndt = false;             // true=NDT, false=GICP
    double ndt_resolution = 1.0;        // NDT 体素分辨率 [m]
    double ndt_step_size = 0.1;         // NDT 优化步长
    int    ndt_max_iterations = 35;

    // B样条连续时间优化 (动态场景)
    bool   use_bspline_refinement = true;
    double spline_dt_s = 0.1;           // 样条结时间间隔 [s]
    int    spline_order = 4;            // B样条阶数 (4=cubic)
    bool   optimize_time_offset = true; // 是否优化时间偏移
    double time_offset_init_s = 0.0;    // 初始时间偏移估计
    double time_offset_max_s = 0.1;     // 最大时间偏移范围 [s]

    // 为 true 且本对提供了 initial_extrinsics 初值时：跳过精标定/B样条，直接以该初值进入手动微调
    bool   use_config_extrinsic_only = false;

    // ─── 手动接受后 GICP 精化（--manual 且用户 Enter 确认后）────────────────
    bool   post_manual_refine = true;
    bool   post_manual_use_multi_frame = true;
    bool   post_manual_two_stage_gicp = true;
    double post_manual_gicp_corr_dist_coarse = 1.0;  // 容纳 ~3° 初值偏差 [m]
    double post_manual_gicp_corr_dist_fine = 0.35;
    double post_manual_voxel_coarse = 0.08;
    double post_manual_voxel_fine = 0.05;
    int    post_manual_gicp_max_iter = 40;
    // 相对手调位姿的最大允许修正量；超过则视为 ICP 错配，保留手调结果
    double post_manual_max_delta_deg = 5.0;
    double post_manual_max_delta_m = 0.10;
    double post_manual_min_overlap_ratio = 0.25;

    // ─── 通用参数 ───────────────────────────────────────────────────
    double voxel_size = 0.1;            // 预处理体素下采样大小 [m]
    double min_overlap_ratio = 0.3;     // 最小 FOV 重叠率
    int    max_frames = 100;            // 最大处理帧数
    int    ceres_max_iterations = 50;   // Ceres 最大迭代
    double convergence_threshold = 1e-5;// 收敛阈值
    bool   verbose = true;
};

// ===================================================================
// 配准质量评估
// ===================================================================
struct RegistrationQuality {
    double fitness_score = 0.0;         // 配准得分 (0-1, 越高越好)
    double inlier_rmse = 0.0;           // 内点 RMSE [m]
    double overlap_ratio = 0.0;         // 重叠率
    int    num_inliers = 0;             // 内点数量
    int    num_correspondences = 0;     // 对应点数量
    bool   converged = false;           // 是否收敛

    // 旋转和平移误差估计
    double rotation_error_deg = -1.0;   // 旋转误差 [deg] (需要 ground truth)
    double translation_error_m = -1.0;  // 平移误差 [m] (需要 ground truth)
};

// ===================================================================
// 两阶段标定结果
// ===================================================================
// 手动接受后的 GICP 精化结果
struct PostManualRefineResult {
    ExtrinsicSE3 extrinsic;
    bool applied = false;           // true=采用精化外参，false=保留手调
    bool gicp_converged = false;
    double delta_rot_deg = 0.0;   // |T_refined - T_manual| 旋转
    double delta_trans_m = 0.0;
    RegistrationQuality manual_quality;
    RegistrationQuality refined_quality;
    std::string message;
};

struct LiDARLiDARTwoStageResult {
    // 粗标定结果
    std::optional<ExtrinsicSE3> coarse;
    RegistrationQuality coarse_quality;
    std::string coarse_method;          // "FPFH_TEASER" / "GMM" / "USER_INIT"

    // 精标定结果
    std::optional<ExtrinsicSE3> fine;
    RegistrationQuality fine_quality;
    std::string fine_method;            // "GICP" / "NDT" / "BSPLINE_GICP"

    // 时间偏移 (动态场景)
    double time_offset_s = 0.0;

    // 诊断信息
    bool needs_manual = false;          // 是否需要手动校准
    double manual_threshold_deg = 0.5;  // 手动校准阈值 [deg]
    std::string failure_reason;         // 失败原因
    std::vector<std::string> warnings;  // 警告信息

    // 获取最佳结果
    const ExtrinsicSE3* best() const {
        if (fine.has_value()) return &fine.value();
        if (coarse.has_value()) return &coarse.value();
        return nullptr;
    }

    // 获取最佳质量
    const RegistrationQuality* best_quality() const {
        if (fine.has_value()) return &fine_quality;
        if (coarse.has_value()) return &coarse_quality;
        return nullptr;
    }
};

// ===================================================================
// 可观测性诊断 (用于无重叠 FOV 场景)
// ===================================================================
struct LiDARLiDARObservability {
    double overlap_ratio = 0.0;         // FOV 重叠率
    bool has_fov_overlap = false;       // 是否有直接 FOV 重叠
    bool needs_motion = false;          // 是否需要运动约束
    bool needs_cascading = false;       // 是否需要级联标定
    std::string recommended_method;     // 推荐方法
    std::vector<std::string> warnings;
};

// ===================================================================
// LiDAR-LiDAR 外参标定器
// ===================================================================
class LiDARLiDARCalibrator {
public:
    using Config = LiDARLiDARConfig;
    using Ptr = std::shared_ptr<LiDARLiDARCalibrator>;

    explicit LiDARLiDARCalibrator(const Config& cfg);
    LiDARLiDARCalibrator() : LiDARLiDARCalibrator(Config{}) {}
    ~LiDARLiDARCalibrator();

    // ===================================================================
    // 主标定接口
    // ===================================================================

    /**
     * @brief 两阶段标定 (推荐使用)
     * @param scans_ref 参考LiDAR点云序列
     * @param scans_target 目标LiDAR点云序列
     * @param ref_id 参考LiDAR ID
     * @param target_id 目标LiDAR ID
     * @param init_guess 初始外参估计 (可选, 无则自动粗标定)
     * @return 两阶段标定结果
     */
    LiDARLiDARTwoStageResult calibrate_two_stage(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target,
        const std::string& ref_id = "lidar_front",
        const std::string& target_id = "lidar_rear",
        const std::optional<Sophus::SE3d>& init_guess = std::nullopt);

    /**
     * @brief 粗标定 (FPFH+TEASER++ 或 GMM)
     * @return 粗标定外参, 失败返回 nullopt
     */
    std::optional<ExtrinsicSE3> calibrate_coarse(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target,
        const std::string& ref_id,
        const std::string& target_id);

    /**
     * @brief 精标定 (GICP/NDT + B样条优化)
     * @param init_extrinsic 初始外参 (来自粗标定或用户指定)
     * @return 精标定外参
     */
    std::optional<ExtrinsicSE3> calibrate_fine(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target,
        const Sophus::SE3d& init_extrinsic,
        const std::string& ref_id,
        const std::string& target_id);

    /**
     * @brief 手动接受后以手调外参为初值做 GICP 精化（信任域 + 可选两阶段）
     * @param manual_extrinsic 用户 Enter 确认的手调外参
     */
    PostManualRefineResult calibrate_post_manual(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target,
        const ExtrinsicSE3& manual_extrinsic,
        const std::string& ref_id,
        const std::string& target_id);

    // ===================================================================
    // 单方法标定接口
    // ===================================================================

    /**
     * @brief FPFH + TEASER++ 特征匹配
     * @return 初始外参估计
     */
    std::optional<Sophus::SE3d> calibrate_fpfh_teaser(
        const LiDARScan& scan_ref,
        const LiDARScan& scan_target);

    /**
     * @brief GMM 联合配准
     * @param init_guess 初始外参 (可选)
     */
    std::optional<Sophus::SE3d> calibrate_gmm(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target,
        const std::optional<Sophus::SE3d>& init_guess = std::nullopt);

    /**
     * @brief GICP 精细配准
     */
    std::optional<Sophus::SE3d> calibrate_gicp(
        const LiDARScan& scan_ref,
        const LiDARScan& scan_target,
        const Sophus::SE3d& init_guess);

    /**
     * @brief NDT 配准
     */
    std::optional<Sophus::SE3d> calibrate_ndt(
        const LiDARScan& scan_ref,
        const LiDARScan& scan_target,
        const Sophus::SE3d& init_guess);

    /**
     * @brief B样条连续时间优化 (动态场景)
     */
    ExtrinsicSE3 refine_with_bspline(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target,
        const Sophus::SE3d& init_extrinsic,
        const std::string& ref_id,
        const std::string& target_id);

    // ===================================================================
    // 质量评估与诊断
    // ===================================================================

    /**
     * @brief 评估配准质量
     */
    RegistrationQuality evaluate_registration(
        const LiDARScan& scan_ref,
        const LiDARScan& scan_target,
        const Sophus::SE3d& extrinsic);

    /**
     * @brief 分析可观测性
     */
    LiDARLiDARObservability analyze_observability(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target);

    /**
     * @brief 计算点云重叠率
     */
    double compute_overlap_ratio(
        const LiDARScan& scan_ref,
        const LiDARScan& scan_target,
        const Sophus::SE3d& extrinsic);

    // ===================================================================
    // 手动校准与可视化
    // ===================================================================

    /**
     * @brief 手动校准入口
     */
    ExtrinsicSE3 calibrate_manual(
        const LiDARScan& scan_ref,
        const LiDARScan& scan_target,
        const ExtrinsicSE3& init_extrin,
        const Sophus::SE3d& delta_transform);

    /**
     * @brief 生成可视化报告
     */
    void generate_visualization(
        const LiDARScan& scan_ref,
        const LiDARScan& scan_target,
        const ExtrinsicSE3& extrinsic,
        const std::string& output_path);

    // ===================================================================
    // 进度回调与日志
    // ===================================================================

    using ProgressCallback = std::function<void(const std::string& stage, double progress)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = std::move(cb); }

    // 设置可视化器 (用于实时显示标定过程)
    void set_visualizer(CalibVisualizer::Ptr viz) { visualizer_ = viz; }
    CalibVisualizer::Ptr get_visualizer() const { return visualizer_; }

    // 启用/禁用实时可视化
    void enable_realtime_viz(bool enable) { enable_realtime_viz_ = enable; }

    // 日志辅助: 记录阶段进展
    void log_stage(const std::string& stage, const std::string& msg) const;
    void log_iter(int iter, double cost, double delta) const;

private:
    Config cfg_;
    ProgressCallback progress_cb_;
    CalibVisualizer::Ptr visualizer_;
    bool enable_realtime_viz_ = false;

    // 内部状态
    std::vector<Sophus::SE3d> frame_transforms_;  // 帧间变换序列
    RegistrationQuality last_quality_;             // 最近配准质量
    std::string last_coarse_method_;               // 最近一次粗标定方法 ("FPFH_RANSAC"/"GMM"/"NDT")

    // ─── 内部方法 ───────────────────────────────────────────────────

    // 预处理: 体素下采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocess_cloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;

    // FPFH 特征提取
    bool compute_fpfh_features(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        std::vector<Eigen::VectorXf>& features);

    // TEASER++ 鲁棒匹配
    std::optional<Sophus::SE3d> solve_teaser(
        const std::vector<Eigen::Vector3d>& src_pts,
        const std::vector<Eigen::Vector3d>& tgt_pts);

    // GMM 初始化与优化
    bool initialize_gmm(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        int num_components);

    // 多帧联合优化
    bool optimize_multi_frame(
        const std::vector<LiDARScan>& scans_ref,
        const std::vector<LiDARScan>& scans_target,
        Sophus::SE3d& extrinsic);

    // 验证外参合理性
    bool validate_extrinsic(const Sophus::SE3d& extrinsic) const;

    // 点云转换
    pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const Sophus::SE3d& transform) const;
};

}  // namespace ns_unicalib
