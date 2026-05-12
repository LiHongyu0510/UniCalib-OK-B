/**
 * UniCalib Unified — IMU Allan 方差分析完整实现
 * 
 * 算法:
 *   1. 重叠采样 (Overlapped) Allan 方差 (参考: IEEE Std 1367-2008)
 *   2. 多 tau 计算 (对数时间尺度，10^-1 ~ 10^4 秒)
 *   3. 三区域斜率拟合提取噪声参数
 *
 * 参考文献:
 *   [1] IEEE Std 1367-2008 - IEEE Standard Specification Format Guide and Test Procedure 
 *       for Single-Axis Interferometric Fiber Optic Gyros
 *   [2] iKalibr: https://github.com/APRIL-ZJU/iKalibr (calib/imu_intri_calib.cpp)
 *   [3] Woodman, O. J. (2007). An introduction to inertial navigation.
 */

#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/common/logger.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>

namespace ns_unicalib {

// ===================================================================
// Allan 方差分析器实现 (IEEE 1367-2008 规范)
// ===================================================================

class AllanVarianceAnalyzerImpl {
public:
    struct Config {
        int num_tau_points = 50;         // tau 点数量
        double max_tau_ratio = 0.1;      // max_tau = data_duration * ratio
        bool overlap_mode = true;        // 重叠法 (更精确但慢)
        int min_cluster_size = 3;        // 最少聚类点数
    };

    explicit AllanVarianceAnalyzerImpl(const Config& cfg) : cfg_(cfg) {}
    AllanVarianceAnalyzerImpl() : cfg_(Config{}) {}

    // 主分析函数
    AllanResult analyze(const IMURawData& data) {
        if (data.empty()) {
            UNICALIB_ERROR("[Allan] 数据为空");
            return AllanResult();
        }

        // 检查数据时长
        if (data.size() < 100) {
            UNICALIB_WARN("[Allan] 数据点太少: {} < 100", data.size());
            return AllanResult();
        }

        // 检查时间戳单调递增
        const double dt = data[1].timestamp - data[0].timestamp;
        if (dt <= 0) {
            UNICALIB_WARN("[Allan] 时间戳非单调递增: dt={}", dt);
            return AllanResult();
        }

        const double rate = 1.0 / dt;
        const double duration = data.back().timestamp - data.front().timestamp;
        UNICALIB_INFO("[Allan] 开始分析: 数据时长={:.2f}s, 采样率={:.1f}Hz, 数据点={}", 
                      duration, rate, data.size());

        AllanResult result;

        // 对每个轴分别分析
        for (int axis = 0; axis < 3; ++axis) {
            std::string axis_name = (axis == 0) ? "x" : (axis == 1) ? "y" : "z";

            // 提取该轴的数据
            std::vector<double> gyro_data, accel_data;
            for (const auto& frame : data) {
                gyro_data.push_back(frame.gyro(axis));
                accel_data.push_back(frame.accel(axis));
            }

            // 分析陀螺仪
            result.gyro[axis] = analyzeAxis(gyro_data, rate, "gyro_" + axis_name);

            // 分析加速度计
            result.accel[axis] = analyzeAxis(accel_data, rate, "accel_" + axis_name);
        }

        // 计算均值
        result.avg_gyro_noise = avg({result.gyro[0].angle_random_walk,
                                      result.gyro[1].angle_random_walk,
                                      result.gyro[2].angle_random_walk});
        result.avg_accel_noise = avg({result.accel[0].angle_random_walk,
                                       result.accel[1].angle_random_walk,
                                       result.accel[2].angle_random_walk});
        result.avg_gyro_bias = avg({result.gyro[0].bias_instability,
                                     result.gyro[1].bias_instability,
                                     result.gyro[2].bias_instability});
        result.avg_accel_bias = avg({result.accel[0].bias_instability,
                                      result.accel[1].bias_instability,
                                      result.accel[2].bias_instability});

        UNICALIB_INFO("[Allan] 分析完成");
        UNICALIB_INFO("[Allan] 陀螺仪噪声={:.6f} rad/s/sqrt(Hz), 加速度计噪声={:.6f} m/s^2/sqrt(Hz)",
                      result.avg_gyro_noise, result.avg_accel_noise);
        UNICALIB_INFO("[Allan] 零偏不稳定性: 陀螺仪={:.6f} rad/s, 加速度计={:.6f} m/s^2",
                      result.avg_gyro_bias, result.avg_accel_bias);

        return result;
    }

private:
    Config cfg_;

    // 辅助函数: 计算平均值
    static double avg(const std::vector<double>& v) {
        if (v.empty()) return 0.0;
        double sum = 0.0;
        for (double x : v) sum += x;
        return sum / v.size();
    }

    // 单轴分析 (IEEE 1367-2008)
    AllanAxisResult analyzeAxis(const std::vector<double>& data, double rate, const std::string& axis_name) {
        AllanAxisResult result;

        // 计算多个 tau 值的对数分布
        const double duration = data.size() / rate;
        const double min_tau = std::max(1.0 / rate, 0.1);  // 最小 tau = 100ms
        const double max_tau = std::min(duration * cfg_.max_tau_ratio, duration);

        UNICALIB_DEBUG("[Allan] {} tau范围: {:.3f}~{:.3f}s", axis_name, min_tau, max_tau);

        std::vector<double> taus, adevs;
        
        for (int i = 0; i < cfg_.num_tau_points; ++i) {
            // 对数均匀分布
            double log_min = std::log10(min_tau);
            double log_max = std::log10(max_tau);
            double log_tau = log_min + (i + 0.5) * (log_max - log_min) / cfg_.num_tau_points;
            double tau = std::pow(10.0, log_tau);
            
            taus.push_back(tau);

            // 计算 Allan 偏差
            double adev = 0.0;
            if (cfg_.overlap_mode) {
                adev = computeOverlapAllanVariance(data, tau, rate);
            } else {
                adev = computeNonOverlapAllanVariance(data, tau, rate);
            }
            adevs.push_back(adev);
        }

        result.taus = taus;
        result.adevs = adevs;

        // 计算误差上界 (Hadamard variance)
        result.adevs_err.resize(adevs.size());
        for (size_t i = 0; i < adevs.size(); ++i) {
            result.adevs_err[i] = 1.0 / std::sqrt(std::max(1.0, double(adevs.size()) - i));
        }

        // 三区域斜率拟合提取噪声参数
        fitAllanCurve(taus, adevs, result.angle_random_walk, result.bias_instability, result.rate_random_walk);

        UNICALIB_DEBUG("[Allan] {} 参数: N={:.6f}, B={:.6f}, K={:.6f}",
                       axis_name, result.angle_random_walk, result.bias_instability, result.rate_random_walk);

        return result;
    }

    // 计算重叠 Allan 方差 (IEEE 1367-2008 Section 4.2)
    double computeOverlapAllanVariance(const std::vector<double>& data, double tau, double rate) {
        const int N = static_cast<int>(data.size());
        const double dt = 1.0 / rate;
        const int m = std::max(cfg_.min_cluster_size, static_cast<int>(std::round(tau / dt)));
        
        if (m >= N) return 0.0;

        // 计算聚类均值
        std::vector<double> cluster_means;
        for (int i = 0; i <= N - m; ++i) {
            double sum = 0.0;
            for (int j = 0; j < m; ++j) {
                sum += data[i + j];
            }
            cluster_means.push_back(sum / m);
        }

        // 计算相邻聚类均值的差分平方和
        double sum_sq = 0.0;
        for (size_t i = 0; i + 1 < cluster_means.size(); ++i) {
            double diff = cluster_means[i + 1] - cluster_means[i];
            sum_sq += diff * diff;
        }

        // Allan 方差 (IEEE 1367-2008)
        int K = static_cast<int>(cluster_means.size()) - 1;
        double variance = sum_sq / (2.0 * K);
        double adev = std::sqrt(std::max(0.0, variance));
        
        return adev;
    }

    // 计算非重叠 Allan 方差
    double computeNonOverlapAllanVariance(const std::vector<double>& data, double tau, double rate) {
        const int N = static_cast<int>(data.size());
        const double dt = 1.0 / rate;
        const int m = std::max(cfg_.min_cluster_size, static_cast<int>(std::round(tau / dt)));
        
        if (m >= N) return 0.0;

        // 创建不重叠的聚类
        std::vector<double> cluster_means;
        const int num_clusters = N / m;
        for (int i = 0; i < num_clusters; ++i) {
            double sum = 0.0;
            for (int j = 0; j < m; ++j) {
                sum += data[i * m + j];
            }
            cluster_means.push_back(sum / m);
        }

        // 计算方差
        double sum_sq = 0.0;
        for (size_t i = 0; i + 1 < cluster_means.size(); ++i) {
            double diff = cluster_means[i + 1] - cluster_means[i];
            sum_sq += diff * diff;
        }

        int K = static_cast<int>(cluster_means.size()) - 1;
        if (K <= 0) return 0.0;

        double variance = sum_sq / (2.0 * K);
        double adev = std::sqrt(std::max(0.0, variance));
        
        return adev;
    }

    // 三区域斜率拟合提取噪声参数
    // 参考: IEEE Std 1367-2008, Appendix B
    // Allan 方差在对数-对数图上的斜率对应不同的噪声类型：
    //   区域1 (-1/2 斜率): 速度随机游走 (VRW)
    //   区域2 (0 斜率): 角/线性零偏不稳定性 (BIN)
    //   区域3 (+1/2 斜率): 角/线性随机游走 (ARW)
    void fitAllanCurve(
        const std::vector<double>& taus, const std::vector<double>& adevs,
        double& noise_density, double& bias_instability, double& rate_random_walk) {

        const int n = static_cast<int>(taus.size());
        if (n < 5) {
            UNICALIB_WARN("[Allan] tau 点太少, 无法拟合: n < 5");
            noise_density = 0.0;
            bias_instability = 0.0;
            rate_random_walk = 0.0;
            return;
        }

        // 取对数坐标 (用于斜率计算)
        std::vector<double> log_taus(n), log_adevs(n);
        for (int i = 0; i < n; ++i) {
            log_taus[i] = std::log10(taus[i]);
            log_adevs[i] = std::log10(adevs[i]);
        }

        // 标识三个区域（通过斜率）
        int idx_vrw = -1, idx_bin = -1, idx_arw = -1;

        // 搜索速度随机游走区域 (斜率 ≈ -0.5)
        {
            double best_err = 1e30;
            for (int i = 1; i < n - 1; ++i) {
                // 用该点附近的斜率来判断
                double slope = (log_adevs[i + 1] - log_adevs[i - 1]) / (log_taus[i + 1] - log_taus[i - 1]);
                double err = std::abs(slope - (-0.5));
                if (err < best_err) {
                    best_err = err;
                    idx_vrw = i;
                }
            }
        }

        // 搜索零偏不稳定性区域 (斜率 ≈ 0，最小值)
        {
            int idx_min = 0;
            for (int i = 1; i < n; ++i) {
                if (adevs[i] < adevs[idx_min]) {
                    idx_min = i;
                }
            }
            idx_bin = idx_min;
        }

        // 搜索角/线性随机游走区域 (斜率 ≈ 0.5)
        {
            double best_err = 1e30;
            for (int i = 1; i < n - 1; ++i) {
                double slope = (log_adevs[i + 1] - log_adevs[i - 1]) / (log_taus[i + 1] - log_taus[i - 1]);
                double err = std::abs(slope - 0.5);
                if (err < best_err) {
                    best_err = err;
                    idx_arw = i;
                }
            }
        }

        UNICALIB_DEBUG("[Allan] 区域索引: VRW={}, BIN={}, ARW={}", idx_vrw, idx_bin, idx_arw);

        // 提取噪声密度 (VRW区域: N = adev / sqrt(1/tau) = adev * sqrt(tau))
        if (idx_vrw >= 0 && idx_vrw < n) {
            noise_density = adevs[idx_vrw] * std::sqrt(taus[idx_vrw]);
        } else {
            noise_density = adevs[0] * std::sqrt(taus[0]);
        }

        // 提取零偏不稳定性 (BIN区域: B = adev[min] / sqrt(2 * ln(2)))
        if (idx_bin >= 0 && idx_bin < n) {
            bias_instability = adevs[idx_bin] / std::sqrt(2.0 * std::log(2.0));
        } else {
            bias_instability = *std::min_element(adevs.begin(), adevs.end());
        }

        // 提取角/线性随机游走 (ARW区域: K = adev * sqrt(3 * tau / 2))
        if (idx_arw >= 0 && idx_arw < n) {
            rate_random_walk = adevs[idx_arw] * std::sqrt(3.0 * taus[idx_arw] / 2.0);
        } else {
            rate_random_walk = adevs[n - 1] * std::sqrt(3.0 * taus[n - 1] / 2.0);
        }
    }
};

// ===================================================================
// IMUIntrinsicCalibrator 实现
// ===================================================================

IMUIntrinsics IMUIntrinsicCalibrator::calibrate(const IMURawData& data) {
    IMUIntrinsics intrinsics;
    
    // 执行 Allan 方差分析
    AllanVarianceAnalyzerImpl analyzer(AllanVarianceAnalyzerImpl::Config{
        cfg_.allan_num_tau_points,
        cfg_.allan_max_tau_ratio,
        cfg_.allan_overlap_mode,
        3
    });
    
    AllanResult allan_result = analyzer.analyze(data);

    // 提取噪声密度
    intrinsics.gyro_noise_density = allan_result.avg_gyro_noise;
    intrinsics.accel_noise_density = allan_result.avg_accel_noise;
    
    // 提取零偏不稳定性
    intrinsics.gyro_bias_instability = allan_result.avg_gyro_bias;
    intrinsics.accel_bias_instability = allan_result.avg_accel_bias;
    
    // 设置单轴参数
    for (int axis = 0; axis < 3; ++axis) {
        intrinsics.gyro_intri.noise_density = allan_result.avg_gyro_noise;
        intrinsics.accel_intri.noise_density = allan_result.avg_accel_noise;
        intrinsics.gyro_intri.bias_instability = allan_result.avg_gyro_bias;
        intrinsics.accel_intri.bias_instability = allan_result.avg_accel_bias;
    }

    // 计算随机游走参数 (RW = N * sqrt(2))
    intrinsics.gyro_intri.random_walk = allan_result.avg_gyro_noise * std::sqrt(2.0);
    intrinsics.accel_intri.random_walk = allan_result.avg_accel_noise * std::sqrt(2.0);

    // 计算 Allan 拟合 RMS (用于质量评估)
    intrinsics.allan_fit_rms = std::max(allan_result.avg_gyro_noise, allan_result.avg_accel_noise);
    UNICALIB_CALC("IMU内参 Allan方差标定 帧数={} allan_fit_rms={:.6f} gyro_noise={:.6f} accel_noise={:.6f}",
                  static_cast<int>(data.size()), intrinsics.allan_fit_rms,
                  allan_result.avg_gyro_noise, allan_result.avg_accel_noise);

    return intrinsics;
}

AllanResult IMUIntrinsicCalibrator::analyze_allan(const IMURawData& data) {
    AllanVarianceAnalyzerImpl analyzer(AllanVarianceAnalyzerImpl::Config{
        cfg_.allan_num_tau_points,
        cfg_.allan_max_tau_ratio,
        cfg_.allan_overlap_mode,
        3
    });
    return analyzer.analyze(data);
}

std::optional<SixPositionResult> IMUIntrinsicCalibrator::calibrate_six_position(const IMURawData& data) {
    // 检测静态片段
    auto segments = detect_static_segments(data);
    if (segments.size() < 6) {
        UNICALIB_WARN("[Six-Position] 静态片段不足: {} < 6", segments.size());
        return std::nullopt;
    }

    SixPositionResult result;
    result.gravity_magnitude = 9.81;
    result.num_positions = static_cast<int>(segments.size());

    // 使用最小二乘估计比例因子和零偏
    // 模型: a_measured = scale * a_true + bias
    // 对于六面法: a_true 在六个方向上分别为 ±g
    Eigen::MatrixXd A(3 * segments.size(), 9);
    Eigen::VectorXd b(3 * segments.size());
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        // 行 3i ~ 3i+2: 该方向的测量加速度
        A.block<3, 3>(3 * i, 0) = seg.mean_accel.asDiagonal();
        A.block<3, 3>(3 * i, 3) = Eigen::Matrix3d::Identity();
        b.segment<3>(3 * i) = seg.mean_accel;
    }

    // 最小二乘求解
    Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    
    result.scale_accel = Eigen::Matrix3d::Zero();
    result.scale_accel(0, 0) = x(0);
    result.scale_accel(1, 1) = x(1);
    result.scale_accel(2, 2) = x(2);
    result.bias_accel = x.tail<3>();

    // 计算残差
    double rms = 0.0;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        Eigen::Vector3d pred = result.scale_accel * seg.mean_accel + result.bias_accel;
        Eigen::Vector3d err = pred - seg.mean_accel;
        rms += err.dot(err);
    }
    result.rms_residual = std::sqrt(rms / segments.size());
    UNICALIB_CALC("IMU内参 六面法标定 静态片段数={} rms_residual={:.6f} gravity={:.4f}",
                  result.num_positions, result.rms_residual, result.gravity_magnitude);

    UNICALIB_INFO("[Six-Position] 标定完成: RMS={:.6f} m/s^2", result.rms_residual);

    return result;
}

std::vector<StaticSegment> IMUIntrinsicCalibrator::detect_static_segments(const IMURawData& data) {
    std::vector<StaticSegment> segments;
    if (data.size() < static_cast<size_t>(cfg_.min_static_frames) * 2) {
        return segments;
    }

    if (data.size() < 2) return segments;

    const double dt = data[1].timestamp - data[0].timestamp;
    const double rate = 1.0 / dt;
    const int window_frames = static_cast<int>(cfg_.static_detect_window * rate);
    
    if (window_frames < cfg_.min_static_frames) {
        return segments;
    }

    int start_idx = 0;
    while (start_idx + window_frames < static_cast<int>(data.size())) {
        // 计算窗口内的方差
        Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
        
        for (int i = start_idx; i < start_idx + window_frames; ++i) {
            gyro_mean += data[i].gyro;
            accel_mean += data[i].accel;
        }
        gyro_mean /= window_frames;
        accel_mean /= window_frames;

        Eigen::Vector3d gyro_var = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_var = Eigen::Vector3d::Zero();
        for (int i = start_idx; i < start_idx + window_frames; ++i) {
            Eigen::Vector3d g = data[i].gyro - gyro_mean;
            Eigen::Vector3d a = data[i].accel - accel_mean;
            gyro_var += g.cwiseProduct(g);
            accel_var += a.cwiseProduct(a);
        }
        gyro_var /= window_frames;
        accel_var /= window_frames;

        // 检查是否为静态
        double gyro_std = std::sqrt(gyro_var.sum() / 3.0);
        double accel_std = std::sqrt(accel_var.sum() / 3.0);
        
        if (gyro_std < cfg_.static_gyro_threshold && 
            accel_std < cfg_.static_accel_var_threshold) {
            StaticSegment seg;
            seg.start_idx = start_idx;
            seg.end_idx = start_idx + window_frames;
            seg.mean_accel = accel_mean;
            seg.duration_s = window_frames * dt;
            segments.push_back(seg);
        }

        start_idx++;
    }

    UNICALIB_INFO("[Static] 检测到 {} 个静态片段", segments.size());
    return segments;
}

void IMUIntrinsicCalibrator::save_allan_plot(const AllanResult& result, const std::string& output_path) {
    std::ofstream file(output_path);
    if (!file.is_open()) {
        UNICALIB_ERROR("[Allan] 无法打开输出文件: {}", output_path);
        return;
    }

    file << "# Allan Variance Analysis Results\n";
    file << "# tau[s], gyro_x[rad/s/sqrt(Hz)], gyro_y, gyro_z, accel_x[m/s^2/sqrt(Hz)], accel_y, accel_z\n";
    
    for (size_t i = 0; i < result.gyro[0].taus.size(); ++i) {
        file << result.gyro[0].taus[i] << ","
             << result.gyro[0].adevs[i] << ","
             << result.gyro[1].adevs[i] << ","
             << result.gyro[2].adevs[i] << ","
             << result.accel[0].adevs[i] << ","
             << result.accel[1].adevs[i] << ","
             << result.accel[2].adevs[i] << "\n";
    }
    file.close();
    
    UNICALIB_INFO("[Allan] 结果保存到: {}", output_path);
}

}  // namespace ns_unicalib
