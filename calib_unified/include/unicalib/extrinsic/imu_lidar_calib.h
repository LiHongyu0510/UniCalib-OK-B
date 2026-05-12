#pragma once
/**
 * UniCalib Unified — IMU-LiDAR 外参标定
 *
 * 两阶段标定流程 (无目标方法全程):
 *   粗标定 (Coarse): L2Calib RL强化学习初始估计
 *     → SE(3)-流形 PPO + Bingham 策略参数化
 *   精标定 (Fine, 无目标):
 *     Step 1: LiDAR 里程计 (NDT/ICP) 获取旋转序列
 *     Step 2: IMU 积分获取旋转序列
 *     Step 3: 手眼标定 (Daniilidis/QEF) 获取初始旋转外参 (或使用粗标定结果)
 *     Step 4: B样条连续时间精细优化 (旋转+平移+时间偏移)
 *   手动校准 (Manual): 通过可视化旋转验证 + 增量调整
 *
 * 算法来源: iKalibr calib_solver (init_prep_li_align + B样条优化)
 * 日志: Step 级别日志, 每次 B样条迭代记录残差
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <string>
#include <functional>
#include <optional>
#include <memory>

// 前向声明 (避免 PCL 头文件污染)
namespace pcl { template<typename T> class PointCloud; struct PointXYZI; }

namespace ns_unicalib {

// ===================================================================
// 传感器帧数据 (共用)
// ===================================================================
struct IMUFrame {
    double timestamp;
    Eigen::Vector3d gyro;   // rad/s
    Eigen::Vector3d accel;  // m/s^2
};

struct LiDARScan {
    double timestamp;
    using PointT = pcl::PointXYZI;
    std::shared_ptr<pcl::PointCloud<PointT>> cloud;
    // 对于旋转式 LiDAR, 每点有时间戳 (运动补偿用)
    std::vector<double> point_timestamps;
};

// ===================================================================
// LiDAR 里程计配对 (相邻帧间旋转)
// ===================================================================
struct LiDARRotPair {
    double t_begin;
    double t_end;
    Sophus::SO3d rot_LiDAR;     // LiDAR 坐标系下的旋转 (相邻两帧 NDT 相对位姿的旋转部分)
    Sophus::SO3d rot_IMU;       // IMU 积分对应的旋转 [t_begin, t_end]
    double quality = 1.0;       // 配准质量 (越高越好)
    double rel_trans_norm = 0;  // 该对 LiDAR 相对平移范数 [m]，用于诊断 NDT 是否退化
};

// ===================================================================
// IMU-LiDAR 外参标定器
// ===================================================================
class IMULiDARCalibrator {
public:
    struct Config {
        // LiDAR 里程计
        double ndt_resolution   = 1.0;   // NDT 体素分辨率 [m]
        double ndt_epsilon      = 0.01;  // NDT 收敛阈值
        int    ndt_max_iter     = 30;    // NDT 最大迭代次数
        double icp_max_corr_dist = 2.0;  // ICP 最大对应距离 [m]
        bool   use_ndt = true;           // true=NDT, false=ICP

        // 静态片段检测
        double static_gyro_thresh = 0.05;    // rad/s — 静态判断
        double min_motion_rot_deg = 3.0;     // 最小运动量 [deg]

        // 短时窗旋转对（降低陀螺积分漂移，参考：长时积分误差∝sqrt(dt)，建议 dt_target<=0.5s）
        double rot_pair_dt_target_s = 0.2;   // 短时窗目标时长 [s]，>0 时在插值位姿上额外生成旋转对
        double min_motion_rot_deg_short = 0.5; // 短时窗最小运动量 [deg]
        double rot_pair_short_step_s = 0.1;  // 短时窗时间步长 [s]

        // 可观测性检测参数 (处理平面运动)
        double min_axis_diversity = 0.3;     // 旋转轴多样性阈值 [0.0=差, 1.0=好]
        double min_pitch_motion_deg = 5.0;   // pitch 方向最小运动 [deg]
        double min_roll_motion_deg = 5.0;    // roll 方向最小运动 [deg]
        double min_yaw_motion_deg = 5.0;     // yaw 方向最小运动 [deg]
        double max_z_trans_ratio = 0.2;     // z 平移占比上限 (超过此值认为非平面运动)

        bool   enable_planar_warning = true; // 是否启用平面运动警告

        bool   use_planar_prior = true;     // 平面运动先验 (约束 z 方向)

        // B样条优化
        double spline_dt_s  = 0.1;          // 样条结时间间隔 [s]
        int    spline_order = 4;            // B样条阶数 (4=cubic)
        double time_offset_init_s = 0.0;   // 初始时间偏移估计
        double time_offset_max_s  = 0.2;   // 最大时间偏移范围
        bool   optimize_time_offset = true;
        bool   optimize_gravity = true;

        // 优化参数
        int    ceres_max_iter   = 50;
        double ceres_loss_scale = 1.0;
        bool   verbose = true;

        // 手眼 180° 歧义与质量
        bool   handeye_fix_180_ambiguity = true;   // 解接近 180° 时比较两解并择一
        // 歧义时的选择策略: "residual"=纯按残差选（谁小选谁，适合外参可能真是 180°）；"prefer_identity"=同向安装先验；"prefer_180"=反向安装先验
        std::string handeye_180_decision = "prefer_identity";
        bool   handeye_prefer_identity_when_ambiguous = true;  // 仅当 handeye_180_decision=="prefer_identity" 时生效
        double handeye_180_residual_margin_deg = 3.0;         // prefer_identity/prefer_180 时，仅当“另一侧”残差优于此值(°)才切换
        double handeye_ratio_min = 0.0;           // 旋转对比值下限（LiDAR角/IMU角），0=不滤
        double handeye_ratio_max = 0.0;           // 旋转对比值上限，0=不滤；建议 (0.4, 1.6) 滤掉异常对
        // 手眼离群点剔除：按残差分位数剔除高残差对并重解，可提升精度
        double handeye_outlier_reject_quantile = 0.0;  // 0=不剔除；0.9=剔除残差>P90的对并重解
        int    handeye_outlier_max_iter = 2;          // 最多迭代剔除次数
        int    handeye_outlier_min_pairs = 20;         // 剔除后至少保留的对数
        // RANSAC 手眼：最小集(2对)采样 + 内点计数，抑制离群对与 180° 歧义
        bool   handeye_ransac_enable = false;         // 是否启用手眼 RANSAC
        double handeye_ransac_inlier_thresh_deg = 3.0; // 内点残差阈值 [deg]
        int    handeye_ransac_max_iter = 200;         // RANSAC 最大迭代次数
    };

    // 可观测性诊断结果
    struct ObservabilityDiagnosis {
        double axis_diversity = 0.0;      // 旋转轴多样性 [0,1]
        double pitch_motion_deg = 0.0;    // pitch 方向总运动 [deg]
        double roll_motion_deg = 0.0;     // roll 方向总运动 [deg]
        double yaw_motion_deg = 0.0;      // yaw 方向总运动 [deg]
        double z_trans_ratio = 0.0;       // z 平移占比
        bool   is_planar_motion = false;  // 是否为平面运动
        std::string recommendation;       // 建议
        std::vector<std::string> warnings; // 警告列表
    };

    explicit IMULiDARCalibrator(const Config& cfg) : cfg_(cfg) {}
    IMULiDARCalibrator() : cfg_(Config{}) {}
    ~IMULiDARCalibrator() = default;  // public 析构，供 JointCalibSolver 等栈上析构

    // ===================================================================
    // 两阶段标定接口 (推荐使用)
    // ===================================================================

    struct TwoStageResult {
        std::optional<ExtrinsicSE3> coarse;   // L2Calib AI 粗估
        double coarse_rot_err_deg = -1.0;     // 旋转误差 [deg]
        std::string coarse_method;            // "L2Calib" / "handeye_only"

        std::optional<ExtrinsicSE3> fine;     // B样条精标定
        double fine_rot_err_deg  = -1.0;
        double fine_trans_err_m  = -1.0;
        double fine_time_offset_s = 0.0;
        std::string fine_method;              // "bspline_full" / "handeye_bspline"

        bool needs_manual = false;
        double manual_threshold_deg = 0.5;

        const ExtrinsicSE3* best() const {
            if (fine.has_value())   return &fine.value();
            if (coarse.has_value()) return &coarse.value();
            return nullptr;
        }
    };

    // 两阶段标定
    // coarse_init: AI粗估初始值 (来自 L2Calib), 为空则用手眼标定初始化
    TwoStageResult calibrate_two_stage(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const std::string& imu_id = "imu_0",
        const std::string& lidar_id = "lidar_0",
        const IMUIntrinsics* imu_intrin = nullptr,
        const std::optional<Sophus::SE3d>& coarse_init = std::nullopt);

    // 手动校准: 增量调整外参后重新计算旋转一致性
    ExtrinsicSE3 calibrate_manual_verify(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const ExtrinsicSE3& init_extrin);

    // 日志辅助: 记录手眼标定每对旋转的残差
    void log_handeye_pair(int idx, double rot_err_deg, double quality) const {
        UNICALIB_TRACE("[IMU-LiDAR-HE] 对 {:3d}: rot_err={:.4f}deg quality={:.3f}",
                       idx, rot_err_deg, quality);
    }

    // 日志辅助: B样条迭代残差
    void log_spline_iter(int iter, double cost, double time_offset) const {
        UNICALIB_TRACE("[IMU-LiDAR-BS] 迭代 {:3d}: cost={:.6f} dt={:.4f}s",
                       iter, cost, time_offset);
    }

    // ===================================================================
    // 单方法标定接口 (原有接口保留)
    // ===================================================================

    // 主标定函数
    // imu_id: IMU 传感器 ID (作为参考系 Br)
    // lidar_id: LiDAR 传感器 ID
    std::optional<ExtrinsicSE3> calibrate(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const std::string& imu_id = "imu_0",
        const std::string& lidar_id = "lidar_0",
        const IMUIntrinsics* imu_intrin = nullptr,
        const std::optional<Sophus::SE3d>& coarse_init = std::nullopt);  // 初值：用于手眼 180° 歧义消解与平移退化时回退

    // 仅执行手眼旋转标定 (不含B样条优化)
    std::optional<Sophus::SO3d> calibrate_rotation_handeye(
        const std::vector<LiDARRotPair>& rot_pairs);

    // 获取 LiDAR 里程计结果 (SE3 序列)
    const std::vector<std::pair<double, Sophus::SE3d>>& get_lidar_odom() const {
        return lidar_odom_;
    }

    using ProgressCallback = std::function<void(const std::string& stage, double progress)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = cb; }

private:
    Config cfg_;
    ProgressCallback progress_cb_;

    std::vector<std::pair<double, Sophus::SE3d>> lidar_odom_;  // LiDAR 里程计结果

    // Step 1: 运行 LiDAR 里程计
    bool run_lidar_odometry(const std::vector<LiDARScan>& scans);

    // Step 2: 构建旋转对 (imu_intrin 可选，用于积分时扣除陀螺零偏)
    std::vector<LiDARRotPair> build_rotation_pairs(
        const std::vector<IMUFrame>& imu_data,
        const IMUIntrinsics* imu_intrin = nullptr);

    // Step 3: 手眼旋转标定 (QEF / Daniilidis)，返回 (旋转, 平均残差[deg])
    // handeye_prior: 有 180° 歧义时优先选与 prior 更接近的解
    std::optional<std::pair<Sophus::SO3d, double>> solve_handeye_rotation(
        const std::vector<LiDARRotPair>& pairs,
        const std::optional<Sophus::SO3d>& handeye_prior = std::nullopt);

    // Step 4: 估计平移，返回 (平移向量[m], RMS[m/s]，失败或约束不足时 rms=-1)
    std::optional<std::pair<Eigen::Vector3d, double>> estimate_translation(
        const std::vector<IMUFrame>& imu_data,
        const Sophus::SO3d& rot_LiDAR_in_IMU,
        double time_offset);

    // Step 5: B样条精细优化
    ExtrinsicSE3 refine_with_spline(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const Sophus::SE3d& init_extrinsic,
        const std::string& imu_id,
        const std::string& lidar_id,
        const IMUIntrinsics* imu_intrin);

    // Step 辅助: IMU 旋转积分 (imu_intrin 非空时扣除 bias_gyro)
    // 可选 out_stats: 输出积分时长、采样数、1σ 漂移估计(ARW)，用于日志与漂移分析
    struct IntegrationStats {
        double dt_s = 0;           // 请求积分区间长度 [t_begin,t_end] (s)
        int num_samples = 0;       // 区间内 IMU 采样数
        double sigma_deg = 0;       // 1σ 角度误差 [deg]，由 noise_gyro*sqrt(dt) 估算
        double t_first_imu = 0;     // 参与积分的首/末 IMU 时间戳（用于时间对齐诊断）
        double t_last_imu = 0;
        double sum_dt = 0;         // 实际积分累加 dt 之和 (s)，应≈ t_last_imu - t_first_imu
        double dt_min = 0;         // 相邻 IMU 样本时间间隔 min/中位数/max (s)，200Hz 标称 0.005
        double dt_median = 0;
        double dt_max = 0;
        int num_skipped = 0;       // 因 dt<=0、dt>1 或 dt<dt_dup_threshold 跳过的步数
    };
    Sophus::SO3d integrate_imu_rotation(
        const std::vector<IMUFrame>& imu_data,
        double t_begin,
        double t_end,
        const IMUIntrinsics* imu_intrin = nullptr,
        IntegrationStats* out_stats = nullptr);

    // 运动激励诊断：从旋转对与里程计统计 roll/pitch/yaw 运动量、旋转轴多样性、z 平移占比
    ObservabilityDiagnosis compute_motion_excitation(
        const std::vector<LiDARRotPair>& pairs) const;

    // 对手眼结果按激励施加先验：激励不足的 RPY 分量置为 0（或保持原值），仅对激励充足的自由度保留标定值
    Sophus::SO3d apply_excitation_prior_to_rotation(
        const Sophus::SO3d& R_handeye,
        const ObservabilityDiagnosis& obs) const;
};

}  // namespace ns_unicalib
