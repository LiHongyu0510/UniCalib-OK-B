/**
 * UniCalib C++ — Allan 方差分析 (IMU 噪声参数)
 * 对应 Python: unicalib/intrinsic/allan_variance.py
 */
#pragma once

#include <vector>
#include <string>
#include <unordered_map>

namespace unicalib {

/** IMU 数据输入 (与 Python DataManager.load_imu_data 一致) */
struct IMUData {
  std::vector<double> timestamps;  // 秒
  std::vector<std::vector<double>> gyro;   // N×3
  std::vector<std::vector<double>> accel;  // N×3
  double sample_rate = 200.0;
};

/** 单轴 Allan 分析结果 */
struct AllanAxisResult {
  double random_walk = 0.0;
  double bias_instability = 0.0;
  double rate_random_walk = 0.0;
  std::vector<double> taus;
  std::vector<double> adevs;
};

/** 完整 Allan 分析输出 */
struct AllanVarianceResult {
  std::unordered_map<std::string, AllanAxisResult> gyro_axis;   // axis_x, axis_y, axis_z
  std::unordered_map<std::string, AllanAxisResult> accel_axis;
  double gyro_noise_avg = 0.0;
  double gyro_bias_avg = 0.0;
  double accel_noise_avg = 0.0;
  double accel_bias_avg = 0.0;
};

class AllanVarianceAnalyzer {
 public:
  AllanVarianceResult analyze(const IMUData& imu_data);

 private:
  AllanAxisResult analyze_axis(
    const std::vector<double>& data,
    double rate,
    const std::string& name = "");
  AllanAxisResult simple_analysis(const std::vector<double>& data, double rate);
  double extract_random_walk(const std::vector<double>& taus, const std::vector<double>& adevs);
  double extract_bias_instability(const std::vector<double>& adevs);
  double extract_rate_random_walk(const std::vector<double>& taus, const std::vector<double>& adevs);
};

}  // namespace unicalib
