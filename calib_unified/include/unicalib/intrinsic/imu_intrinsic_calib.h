#pragma once
/**
 * UniCalib Unified — IMU 内参标定
 * 算法来源: unicalib_C_plus_plus + iKalibr/nofree/imu_intri_calib
 * 方法:
 *   1. Allan 方差分析 — 噪声密度、零偏不稳定性、随机游走
 *   2. 六面法 (重力投影) — 加速度计比例因子、零偏
 *   3. B样条连续时间 — 陀螺仪比例因子矩阵、交轴误差 (可选，数据充分时)
 */

#include "unicalib/common/calib_param.h"
#include <vector>
#include <string>
#include <functional>
#include <optional>

namespace ns_unicalib {

// ===================================================================
// IMU 原始数据帧
// ===================================================================
struct IMURawFrame {
    double timestamp;       // 秒
    Eigen::Vector3d gyro;   // rad/s
    Eigen::Vector3d accel;  // m/s^2
};

using IMURawData = std::vector<IMURawFrame>;

// ===================================================================
// Allan 方差分析结果 (单轴)
// ===================================================================
struct AllanAxisResult {
    double angle_random_walk = 0.0;      // rad/s/sqrt(Hz) — N (Noise)
    double bias_instability   = 0.0;     // rad/s    — B (Bias)
    double rate_random_walk   = 0.0;     // rad/s^1.5/sqrt(Hz) — K

    std::vector<double> taus;            // 聚合时间 [s]
    std::vector<double> adevs;           // Allan 偏差
    std::vector<double> adevs_err;       // 偏差误差上界
};

struct AllanResult {
    std::array<AllanAxisResult, 3> gyro;   // [x, y, z]
    std::array<AllanAxisResult, 3> accel;
    double avg_gyro_noise  = 0.0;  // 三轴均值
    double avg_accel_noise = 0.0;
    double avg_gyro_bias   = 0.0;
    double avg_accel_bias  = 0.0;
};

// ===================================================================
// 六面静态标定结果
// ===================================================================
struct SixPositionResult {
    Eigen::Matrix3d scale_accel = Eigen::Matrix3d::Identity();   // 比例因子矩阵
    Eigen::Vector3d bias_accel  = Eigen::Vector3d::Zero();        // 零偏
    double gravity_magnitude = 9.81;
    double rms_residual = 0.0;
    int    num_positions = 0;
};

// ===================================================================
// 静态片段检测器
// ===================================================================
struct StaticSegment {
    int start_idx;          // 起始帧索引
    int end_idx;            // 结束帧索引 (exclusive)
    Eigen::Vector3d mean_accel;  // 平均加速度向量
    double duration_s;
};

// ===================================================================
// IMU 内参标定器
// ===================================================================
class IMUIntrinsicCalibrator {
public:
    struct Config {
        // Allan 参数
        int    allan_num_tau_points  = 50;      // tau 点数量
        double allan_max_tau_ratio   = 0.1;     // max_tau = data_duration * ratio
        bool   allan_overlap_mode    = true;    // 重叠法 (更精确但慢)

        // 六面法参数
        double static_detect_window  = 0.5;    // 静态检测窗口 [s]
        double static_gyro_threshold = 0.02;   // rad/s — 静态判断阈值
        double static_accel_var_threshold = 0.1;  // m/s^2 方差阈值
        int    min_static_frames     = 50;      // 最少静态帧数

        // MAP_COEFF 可选优化
        // 参考: iKalibr estimator_tpl.hpp, OPT_GYRO_MAP_COEFF / OPT_ACCE_MAP_COEFF
        // MAP_COEFF: [v1, v2, v3, v4, v5, v6]^T 代表传感器的比例因子矩阵
        //   M = [v1   v4   v5]  (上三角矩阵参数化)
        //       [0    v2   v6]
        //       [0    0    v3]
        bool   optimize_gyro_map_coeff = false;   // 陀螺仪比例因子矩阵优化
        bool   optimize_accel_map_coeff = false;  // 加速度计比例因子矩阵优化

        // 输出
        bool   verbose = true;
    };

    explicit IMUIntrinsicCalibrator(const Config& cfg) : cfg_(cfg) {}
    IMUIntrinsicCalibrator() : cfg_(Config{}) {}

    // 主标定函数 — 返回标定后的 IMU 内参
    IMUIntrinsics calibrate(const IMURawData& data);

    // 仅执行 Allan 方差分析
    AllanResult analyze_allan(const IMURawData& data);

    // 仅执行六面法
    std::optional<SixPositionResult> calibrate_six_position(const IMURawData& data);

    // 检测静态片段
    std::vector<StaticSegment> detect_static_segments(const IMURawData& data);

    // 回调: 每完成一阶段时调用
    using ProgressCallback = std::function<void(const std::string& stage, double progress)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = cb; }

    // 绘制 Allan 偏差图 (保存为 PNG)
    void save_allan_plot(const AllanResult& result,
                         const std::string& output_path);

private:
    Config cfg_;
    ProgressCallback progress_cb_;

    // Allan 单轴分析
    AllanAxisResult analyze_axis(
        const std::vector<double>& data, double sample_rate_hz);

    // 从 Allan 曲线提取噪声参数 (最小二乘拟合)
    static double fit_noise_density(
        const std::vector<double>& taus, const std::vector<double>& adevs);
    static double fit_bias_instability(
        const std::vector<double>& adevs);
    static double fit_rate_random_walk(
        const std::vector<double>& taus, const std::vector<double>& adevs);
};

}  // namespace ns_unicalib
