/**
 * UniCalib Unified — Allan 方差分析实现
 * 来源整合: unicalib_C_plus_plus/src/allan_variance.cpp + iKalibr nofree/imu_intri_calib
 * 算法: 重叠 Allan 方差法 (OADEV) — IEEE Std 1139-2022
 *   τ_k = m * τ_0, m = 1,2,...,floor(N/3)
 *   σ²_A(τ) = 1/(2τ²(N-2m)) * Σ [Ω(k+2m) - 2Ω(k+m) + Ω(k)]²
 * 噪声参数拟合:
 *   N (角度随机游走): σ_A(τ) ≈ N/√τ → τ=1s 处读取
 *   B (零偏不稳定性): σ_A 最小值 / 0.664
 *   K (速率随机游走): σ_A(τ) ≈ K√τ/√3 → τ=3s 处读取
 */

#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/common/logger.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <cassert>
#include <stdexcept>

// 可选: matplotlib-cpp 绘图
#ifdef UNICALIB_WITH_MATPLOTLIB
#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;
#endif

namespace ns_unicalib {

// ===================================================================
// 内部工具函数
// ===================================================================
namespace {

// 计算累积和 (phase data / integrated data)
std::vector<double> cumulative_sum(const std::vector<double>& data, double dt) {
    std::vector<double> phase(data.size() + 1, 0.0);
    for (size_t i = 0; i < data.size(); ++i) {
        phase[i + 1] = phase[i] + data[i] * dt;
    }
    return phase;
}

// 重叠 Allan 偏差 (OADEV) 计算
void compute_oadev(
    const std::vector<double>& phase,   // 累积相位数据
    double tau0,                         // 基础时间间隔 [s]
    std::vector<double>& taus_out,
    std::vector<double>& adevs_out,
    std::vector<double>& errors_out,
    int num_tau_points = 50) {

    taus_out.clear(); adevs_out.clear(); errors_out.clear();
    const size_t N = phase.size();
    if (N < 4) return;

    // 计算 tau 序列 (对数均匀分布)
    std::vector<int> m_vals;
    int m_max = static_cast<int>((N - 1) / 2);
    if (m_max < 1) return;

    // 对数均匀采样 m 值
    double log_min = 0.0;
    double log_max = std::log10(static_cast<double>(m_max));
    for (int i = 0; i < num_tau_points; ++i) {
        double log_m = log_min + i * (log_max - log_min) / (num_tau_points - 1);
        int m = std::max(1, static_cast<int>(std::round(std::pow(10.0, log_m))));
        if (m_vals.empty() || m != m_vals.back()) {
            m_vals.push_back(m);
        }
    }

    for (int m : m_vals) {
        if (m > m_max) break;
        double tau = m * tau0;

        // OADEV 计算
        double sum = 0.0;
        int count = 0;
        for (size_t j = 0; j + 2 * m < N; ++j) {
            double diff = phase[j + 2*m] - 2*phase[j + m] + phase[j];
            sum += diff * diff;
            ++count;
        }
        if (count == 0) continue;

        // 修复除零风险：tau 应该大于 0
        if (tau <= 0.0) {
            UNICALIB_WARN("[Allan] tau 非正: tau={}", tau);
            continue;
        }
        
        double avar = sum / (2.0 * tau * tau * count);
        double adev = std::sqrt(std::max(0.0, avar));

        // 不确定度估计 (chi-squared 近似)，使用浮点除法避免整数除零
        double dof  = static_cast<double>(static_cast<int64_t>(N) - 2 * m) / static_cast<double>(m);
        if (dof <= 0.0) {
            UNICALIB_WARN("[Allan] 自由度非正: dof={}", dof);
            continue;
        }
        double err  = adev / std::sqrt(2.0 * std::max(1.0, dof));

        taus_out.push_back(tau);
        adevs_out.push_back(adev);
        errors_out.push_back(err);
    }
}

// 在对数坐标下线性插值
double log_interp(const std::vector<double>& taus,
                  const std::vector<double>& adevs,
                  double tau_target) {
    if (taus.empty()) return 0.0;
    if (taus.size() == 1) return adevs[0];

    // 找到最近的两点
    for (size_t i = 0; i + 1 < taus.size(); ++i) {
        if (taus[i] <= tau_target && tau_target <= taus[i+1]) {
            // 对数线性插值
            double t  = std::log10(tau_target);
            double t0 = std::log10(taus[i]);
            double t1 = std::log10(taus[i+1]);
            // 修复数值稳定性：使用更合理的阈值
            constexpr double MIN_ADEV_THRESHOLD = 1e-12; // 最小有效方差阈值
            double a0 = std::log10(std::max(MIN_ADEV_THRESHOLD, adevs[i]));
            double a1 = std::log10(std::max(MIN_ADEV_THRESHOLD, adevs[i+1]));
            double alpha = (t - t0) / (t1 - t0 + 1e-20);
            return std::pow(10.0, a0 + alpha * (a1 - a0));
        }
    }
    // 外推
    if (tau_target < taus.front()) return adevs.front();
    return adevs.back();
}

// 最小二乘拟合 log(adev) = a * log(tau) + b
// 在指定斜率范围内寻找最佳拟合
double fit_slope(const std::vector<double>& taus,
                 const std::vector<double>& adevs,
                 double target_slope) {
    // 对每个区间计算斜率，找与 target_slope 最接近的区间
    if (taus.size() < 2) return adevs.empty() ? 0.0 : adevs[0];

    double best_err = 1e30;
    double best_adev = adevs[0];

    for (size_t i = 0; i + 1 < taus.size(); ++i) {
        double slope = (std::log10(adevs[i+1] + 1e-30) - std::log10(adevs[i] + 1e-30)) /
                       (std::log10(taus[i+1]) - std::log10(taus[i]) + 1e-30);
        double err = std::abs(slope - target_slope);
        if (err < best_err) {
            best_err = err;
            // 在中点求值
            double tau_mid = std::sqrt(taus[i] * taus[i+1]);
            best_adev = log_interp(taus, adevs, tau_mid);
        }
    }
    return best_adev;
}

}  // anonymous namespace

// ===================================================================
// 单轴 Allan 方差分析
// ===================================================================
AllanAxisResult IMUIntrinsicCalibrator::analyze_axis(
    const std::vector<double>& data,
    double sample_rate_hz) {

    AllanAxisResult res;
    if (data.size() < 4 || sample_rate_hz <= 0) {
        UNICALIB_WARN("Allan: insufficient data ({} samples)", data.size());
        return res;
    }

    double tau0 = 1.0 / sample_rate_hz;

    // 计算 OADEV
    compute_oadev(cumulative_sum(data, tau0), tau0,
                  res.taus, res.adevs, res.adevs_err,
                  cfg_.allan_num_tau_points);

    if (res.taus.empty()) {
        UNICALIB_WARN("Allan: OADEV computation failed");
        return res;
    }

    // 提取噪声参数
    // 1. 角度/速度随机游走 (N): 斜率 -1/2 区域, τ=1s 处读取
    //    σ_A(τ=1) = N [rad/s/sqrt(Hz) 或 m/s^2/sqrt(Hz)]
    res.angle_random_walk = log_interp(res.taus, res.adevs, 1.0);

    // 2. 零偏不稳定性 (B): 最小值 / 0.664
    //    (0.664 = sqrt(2*ln(2)/π) 是 flicker 噪声的修正因子)
    double min_adev = *std::min_element(res.adevs.begin(), res.adevs.end());
    res.bias_instability = min_adev / 0.664;

    // 3. 速率随机游走 (K): 斜率 +1/2 区域, τ=3s 处读取
    //    σ_A(τ=3) ≈ K * sqrt(3) / sqrt(3) = K * sqrt(τ/3)
    double adev_at_3 = log_interp(res.taus, res.adevs, 3.0);
    res.rate_random_walk = adev_at_3 * std::sqrt(1.0 / 3.0);

    UNICALIB_DEBUG("Allan axis: ARW={:.4e}, BI={:.4e}, RRW={:.4e}",
                   res.angle_random_walk, res.bias_instability, res.rate_random_walk);
    return res;
}

// ===================================================================
// 完整 Allan 方差分析 (3轴陀螺 + 3轴加速度计)
// ===================================================================
AllanResult IMUIntrinsicCalibrator::analyze_allan(const IMURawData& imu_data) {
    AllanResult result;
    if (imu_data.empty()) {
        UNICALIB_ERROR("Allan: empty IMU data");
        return result;
    }

    // 估计采样率
    double dt_sum = 0.0;
    int dt_count = 0;
    for (size_t i = 1; i < std::min(imu_data.size(), size_t(1000)); ++i) {
        double dt = imu_data[i].timestamp - imu_data[i-1].timestamp;
        if (dt > 1e-6 && dt < 1.0) {
            dt_sum += dt;
            ++dt_count;
        }
    }
    double sample_rate = (dt_count > 0) ? (1.0 / (dt_sum / dt_count)) : 200.0;
    UNICALIB_INFO("Allan: sample_rate={:.1f} Hz, N={} frames", sample_rate, imu_data.size());

    // 分离各轴数据
    const size_t N = imu_data.size();
    std::array<std::vector<double>, 3> gyro_axes, accel_axes;
    for (size_t i = 0; i < N; ++i) {
        for (int ax = 0; ax < 3; ++ax) {
            gyro_axes[ax].push_back(imu_data[i].gyro[ax]);
            accel_axes[ax].push_back(imu_data[i].accel[ax]);
        }
    }

    // 分析各轴
    const char* axis_names[] = {"X", "Y", "Z"};
    for (int ax = 0; ax < 3; ++ax) {
        UNICALIB_INFO("Allan: analyzing gyro axis {} ...", axis_names[ax]);
        result.gyro[ax] = analyze_axis(gyro_axes[ax], sample_rate);
        UNICALIB_INFO("Allan: analyzing accel axis {} ...", axis_names[ax]);
        result.accel[ax] = analyze_axis(accel_axes[ax], sample_rate);
    }

    // 计算三轴均值
    result.avg_gyro_noise  = (result.gyro[0].angle_random_walk +
                               result.gyro[1].angle_random_walk +
                               result.gyro[2].angle_random_walk) / 3.0;
    result.avg_gyro_bias   = (result.gyro[0].bias_instability +
                               result.gyro[1].bias_instability +
                               result.gyro[2].bias_instability) / 3.0;
    result.avg_accel_noise = (result.accel[0].angle_random_walk +
                               result.accel[1].angle_random_walk +
                               result.accel[2].angle_random_walk) / 3.0;
    result.avg_accel_bias  = (result.accel[0].bias_instability +
                               result.accel[1].bias_instability +
                               result.accel[2].bias_instability) / 3.0;

    return result;
}

// ===================================================================
// 静态片段检测
// ===================================================================
std::vector<StaticSegment> IMUIntrinsicCalibrator::detect_static_segments(
    const IMURawData& data) {

    std::vector<StaticSegment> segments;
    if (data.empty()) return segments;

    // 估计采样率
    double dt = 0.005;
    if (data.size() > 1) {
        dt = data[1].timestamp - data[0].timestamp;
        if (dt <= 0) dt = 0.005;
    }
    double rate = 1.0 / dt;
    int win_size = std::max(10, static_cast<int>(cfg_.static_detect_window * rate));

    bool in_static = false;
    int seg_start = 0;

    for (int i = 0; i + win_size <= static_cast<int>(data.size()); i += win_size / 2) {
        // 计算窗口内陀螺方差
        Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
        for (int j = i; j < i + win_size; ++j) {
            gyro_mean += data[j].gyro;
        }
        gyro_mean /= win_size;

        double gyro_var = 0.0;
        for (int j = i; j < i + win_size; ++j) {
            Eigen::Vector3d diff = data[j].gyro - gyro_mean;
            gyro_var += diff.squaredNorm();
        }
        gyro_var /= win_size;

        // 判断是否静止：陀螺均值与陀螺标准差均用 rad/s 阈值（避免与加速度 m/s² 混用）
        double gyro_std = std::sqrt(gyro_var);
        bool is_static = (gyro_mean.norm() < cfg_.static_gyro_threshold &&
                          gyro_std < cfg_.static_gyro_threshold);

        if (is_static && !in_static) {
            in_static = true;
            seg_start = i;
        } else if (!is_static && in_static) {
            in_static = false;
            if (i - seg_start >= cfg_.min_static_frames) {
                // 计算平均加速度
                Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
                for (int j = seg_start; j < i; ++j) {
                    mean_accel += data[j].accel;
                }
                mean_accel /= (i - seg_start);
                double dur = (data[i-1].timestamp - data[seg_start].timestamp);
                segments.push_back({seg_start, i, mean_accel, dur});
            }
        }
    }
    // 末尾
    if (in_static && static_cast<int>(data.size()) - seg_start >= cfg_.min_static_frames) {
        Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
        for (size_t j = seg_start; j < data.size(); ++j) {
            mean_accel += data[j].accel;
        }
        mean_accel /= (data.size() - seg_start);
        double dur = data.back().timestamp - data[seg_start].timestamp;
        segments.push_back({seg_start, static_cast<int>(data.size()), mean_accel, dur});
    }

    UNICALIB_INFO("Static segments detected: {}", segments.size());
    return segments;
}

// ===================================================================
// 六面法标定
// ===================================================================
std::optional<SixPositionResult> IMUIntrinsicCalibrator::calibrate_six_position(
    const IMURawData& data) {

    auto segments = detect_static_segments(data);
    if (segments.size() < 6) {
        UNICALIB_WARN("Six-position: only {} static segments found (need ≥6)",
                      segments.size());
        return std::nullopt;
    }

    // 收集各静态位姿下的平均加速度向量
    std::vector<Eigen::Vector3d> acc_meas;
    for (const auto& seg : segments) {
        acc_meas.push_back(seg.mean_accel);
    }

    // 简单估计: 重力大小
    double g_mag = 0.0;
    for (const auto& a : acc_meas) g_mag += a.norm();
    g_mag /= acc_meas.size();

    // 初始零偏: 所有测量的平均值减去预期重力
    // 使用最小二乘法: ||Ma * a_true + b_a|| = g
    // 简化: 假设比例因子接近对角, 估计零偏
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    int used = 0;
    for (size_t i = 0; i < acc_meas.size() && used < 12; ++i) {
        double n = acc_meas[i].norm();
        if (n < 1e-9) continue;  // 避免 normalized() 零向量/数值不稳定
        Eigen::Vector3d dir = acc_meas[i] / n;
        bias += acc_meas[i] - dir * g_mag;
        ++used;
    }
    if (used > 0) bias /= used;

    SixPositionResult result;
    result.bias_accel = bias;
    result.gravity_magnitude = g_mag;
    result.num_positions = static_cast<int>(segments.size());

    // 计算残差
    double rms = 0.0;
    for (const auto& a : acc_meas) {
        double err = (a - bias).norm() - g_mag;
        rms += err * err;
    }
    result.rms_residual = std::sqrt(rms / acc_meas.size());
    UNICALIB_CALC("IMU内参 六面法(legacy) 位置数={} rms_residual={:.6f} g={:.4f}",
                  result.num_positions, result.rms_residual, g_mag);
    UNICALIB_INFO("Six-position: bias=[{:.4f},{:.4f},{:.4f}] g={:.4f} rms={:.4f}",
                  bias[0], bias[1], bias[2], g_mag, result.rms_residual);
    return result;
}

// ===================================================================
// 主标定函数
// ===================================================================
IMUIntrinsics IMUIntrinsicCalibrator::calibrate(const IMURawData& data) {
    if (progress_cb_) progress_cb_("starting", 0.0);

    IMUIntrinsics result;
    if (data.size() < 100) {
        UNICALIB_ERROR("IMU intrinsic: insufficient data ({} frames)", data.size());
        return result;
    }

    UNICALIB_INFO("=== IMU 内参标定开始 ===");
    UNICALIB_INFO("  数据帧数: {}", data.size());
    double duration = data.back().timestamp - data.front().timestamp;
    UNICALIB_INFO("  数据时长: {:.1f} s", duration);

    // Step 1: Allan 方差分析
    UNICALIB_LOG_STEP("IMU-Intrin", "Step 1/2: Allan 方差分析 ...");
    if (progress_cb_) progress_cb_("allan_variance", 0.1);

    AllanResult allan = analyze_allan(data);

    // Step 2: 六面法 (可选)
    UNICALIB_LOG_STEP("IMU-Intrin", "Step 2/2: 六面法加速度计标定 ...");
    if (progress_cb_) progress_cb_("six_position", 0.7);

    auto six_pos = calibrate_six_position(data);

    // 填充结果
    result.noise_gyro       = allan.avg_gyro_noise;
    result.bias_instab_gyro = allan.avg_gyro_bias;
    result.noise_acce       = allan.avg_accel_noise;
    result.bias_instab_acce = allan.avg_accel_bias;
    result.num_samples_used = static_cast<int>(data.size());

    if (six_pos.has_value()) {
        result.bias_acce = six_pos->bias_accel;
        result.allan_fit_rms = six_pos->rms_residual;
        UNICALIB_INFO("六面法成功: g={:.4f} m/s^2", six_pos->gravity_magnitude);
    }
    UNICALIB_CALC("IMU内参(legacy) 标定完成 帧数={} allan_fit_rms={:.6f} noise_gyro={:.4e} noise_acce={:.4e}",
                  result.num_samples_used, result.allan_fit_rms, result.noise_gyro, result.noise_acce);

    if (progress_cb_) progress_cb_("done", 1.0);

    UNICALIB_INFO("=== IMU 内参标定完成 ===");
    UNICALIB_INFO("  陀螺噪声:     {:.4e} rad/s/√Hz", result.noise_gyro);
    UNICALIB_INFO("  陀螺零偏不稳: {:.4e} rad/s", result.bias_instab_gyro);
    UNICALIB_INFO("  加速度计噪声: {:.4e} m/s²/√Hz", result.noise_acce);
    UNICALIB_INFO("  加速度计零偏不稳: {:.4e} m/s²", result.bias_instab_acce);

    return result;
}

// ===================================================================
// Allan 图绘制 (保存 PNG, 使用 OpenCV 绘制)
// ===================================================================
void IMUIntrinsicCalibrator::save_allan_plot(
    const AllanResult& result,
    const std::string& output_path) {

    // 使用 OpenCV 绘制 Allan 偏差图
    const int W = 800, H = 600;
    const int MARGIN_L = 80, MARGIN_B = 60, MARGIN_R = 40, MARGIN_T = 50;
    int plot_w = W - MARGIN_L - MARGIN_R;
    int plot_h = H - MARGIN_B - MARGIN_T;

    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    // 找数据范围 (对数坐标)
    double tau_min = 1e30, tau_max = -1e30;
    double adev_min = 1e30, adev_max = -1e30;
    // BGR 颜色 (蓝, 绿, 红)
    const cv::Scalar colors[3] = {
        cv::Scalar(200, 80, 80),   // X = 蓝偏红
        cv::Scalar(80, 180, 80),   // Y = 绿
        cv::Scalar(80, 80, 220)    // Z = 蓝
    };
    const char* axes[] = {"X", "Y", "Z"};

    for (int ax = 0; ax < 3; ++ax) {
        const auto& r = result.gyro[ax];
        for (size_t i = 0; i < r.taus.size(); ++i) {
            if (r.taus[i] > 0 && r.adevs[i] > 0) {
                tau_min = std::min(tau_min, r.taus[i]);
                tau_max = std::max(tau_max, r.taus[i]);
                adev_min = std::min(adev_min, r.adevs[i]);
                adev_max = std::max(adev_max, r.adevs[i]);
            }
        }
    }
    if (tau_min >= tau_max || adev_min >= adev_max) {
        UNICALIB_WARN("Allan plot: no valid data for plotting");
        return;
    }

    // 坐标变换 (对数)
    auto to_px = [&](double tau, double adev) -> cv::Point {
        double x = (std::log10(tau) - std::log10(tau_min)) /
                   (std::log10(tau_max) - std::log10(tau_min) + 1e-9);
        double y = (std::log10(adev) - std::log10(adev_min)) /
                   (std::log10(adev_max) - std::log10(adev_min) + 1e-9);
        int px = MARGIN_L + static_cast<int>(x * plot_w);
        int py = MARGIN_T + static_cast<int>((1.0 - y) * plot_h);
        return {px, py};
    };

    // 绘制网格
    cv::rectangle(img, cv::Point(MARGIN_L, MARGIN_T),
                  cv::Point(W-MARGIN_R, H-MARGIN_B),
                  cv::Scalar(200,200,200), 1);

    // 绘制各轴
    for (int ax = 0; ax < 3; ++ax) {
        const auto& r = result.gyro[ax];
        if (r.taus.empty()) continue;
        const cv::Scalar& color = colors[ax];
        for (size_t i = 1; i < r.taus.size(); ++i) {
            if (r.adevs[i] > 0 && r.adevs[i-1] > 0) {
                cv::line(img, to_px(r.taus[i-1], r.adevs[i-1]),
                         to_px(r.taus[i], r.adevs[i]), color, 2);
            }
        }
    }

    // 标题
    cv::putText(img, "Allan Deviation (Gyroscope)",
                cv::Point(MARGIN_L, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 2);

    cv::imwrite(output_path, img);
    UNICALIB_INFO("Allan plot saved: {}", output_path);
}

}  // namespace ns_unicalib
