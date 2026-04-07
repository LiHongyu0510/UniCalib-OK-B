/**
 * UniCalib C++ — Allan 方差分析实现 (简化版，不依赖 allantools)
 */
#include "unicalib/allan_variance.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <stdexcept>

namespace unicalib {

namespace {

double stddev(const std::vector<double>& v) {
  if (v.size() < 2) return 0.0;
  double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  double sq_sum = 0.0;
  for (double x : v) sq_sum += (x - mean) * (x - mean);
  return std::sqrt(sq_sum / (v.size() - 1));
}

}  // namespace

AllanAxisResult AllanVarianceAnalyzer::simple_analysis(
  const std::vector<double>& data,
  double rate) {
  AllanAxisResult res;
  if (data.empty() || rate <= 0) return res;
  double dt = 1.0 / rate;
  res.random_walk = stddev(data) / std::sqrt(rate);
  size_t window = std::max(size_t(1), static_cast<size_t>(rate * 100));
  if (data.size() > window * 2) {
    std::vector<double> means;
    for (size_t i = 0; i + window <= data.size(); i += window / 2) {
      double sum = 0.0;
      for (size_t j = i; j < i + window && j < data.size(); ++j) sum += data[j];
      means.push_back(sum / window);
    }
    res.bias_instability = stddev(means);
  } else {
    res.bias_instability = stddev(data) * 0.1;
  }
  res.rate_random_walk = res.random_walk * 0.1;
  return res;
}

double AllanVarianceAnalyzer::extract_random_walk(
  const std::vector<double>& taus,
  const std::vector<double>& adevs) {
  if (adevs.empty()) return 0.0;
  size_t idx = 0;
  double best = 1e30;
  for (size_t i = 0; i < taus.size(); ++i) {
    double d = std::abs(taus[i] - 1.0);
    if (d < best) { best = d; idx = i; }
  }
  if (idx < adevs.size()) return adevs[idx];
  return adevs[0];
}

double AllanVarianceAnalyzer::extract_bias_instability(const std::vector<double>& adevs) {
  if (adevs.empty()) return 0.0;
  return *std::min_element(adevs.begin(), adevs.end());
}

double AllanVarianceAnalyzer::extract_rate_random_walk(
  const std::vector<double>& taus,
  const std::vector<double>& adevs) {
  if (adevs.empty()) return 0.0;
  size_t idx = 0;
  double best = 1e30;
  for (size_t i = 0; i < taus.size(); ++i) {
    double d = std::abs(taus[i] - 3.0);
    if (d < best) { best = d; idx = i; }
  }
  if (idx < adevs.size()) return adevs[idx] / std::sqrt(3.0);
  return 0.0;
}

AllanAxisResult AllanVarianceAnalyzer::analyze_axis(
  const std::vector<double>& data,
  double rate,
  const std::string& /*name*/) {
  return simple_analysis(data, rate);
}

AllanVarianceResult AllanVarianceAnalyzer::analyze(const IMUData& imu_data) {
  AllanVarianceResult result;
  if (imu_data.gyro.empty() || imu_data.accel.empty()) return result;

  const double rate = imu_data.sample_rate;
  const char* axis_names[] = {"x", "y", "z"};
  std::vector<double> gyro_rw, gyro_bi, accel_rw, accel_bi;

  for (int i = 0; i < 3; ++i) {
    std::vector<double> g_col, a_col;
    for (const auto& row : imu_data.gyro) {
      if (row.size() >= 3) g_col.push_back(row[i]);
    }
    for (const auto& row : imu_data.accel) {
      if (row.size() >= 3) a_col.push_back(row[i]);
    }
    std::string ax = std::string("axis_") + axis_names[i];
    result.gyro_axis[ax] = analyze_axis(g_col, rate, "gyro_" + ax);
    result.accel_axis[ax] = analyze_axis(a_col, rate, "accel_" + ax);
    gyro_rw.push_back(result.gyro_axis[ax].random_walk);
    gyro_bi.push_back(result.gyro_axis[ax].bias_instability);
    accel_rw.push_back(result.accel_axis[ax].random_walk);
    accel_bi.push_back(result.accel_axis[ax].bias_instability);
  }

  auto avg = [](const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  };
  result.gyro_noise_avg = avg(gyro_rw);
  result.gyro_bias_avg = avg(gyro_bi);
  result.accel_noise_avg = avg(accel_rw);
  result.accel_bias_avg = avg(accel_bi);
  return result;
}

}  // namespace unicalib
