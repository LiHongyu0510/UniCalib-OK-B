/**
 * UniCalib C++ — ReportGenerator 实现
 */
#include "unicalib/report_generator.hpp"
#include "unicalib/system.hpp"
#include "unicalib/transforms.hpp"
#include <cstring>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>

namespace unicalib {

ReportGenerator::ReportGenerator(const std::string& output_dir) : output_dir_(output_dir) {}

void ReportGenerator::print_summary(
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const std::unordered_map<std::string, CalibResult>& extrinsics,
  const ValidationReport& validation) {
  const std::string line(72, '=');
  std::cout << "\n" << line << "\n";
  std::cout << "  UniCalib 标定结果汇总\n";
  std::cout << line << "\n";

  std::cout << "\n【内参标定结果】\n";
  for (const auto& [sid, h] : intrinsics) {
    if (h.camera_pinhole) {
      const auto& c = *h.camera_pinhole;
      std::cout << "  " << std::left << std::setw(20) << sid
                << " fx=" << std::fixed << std::setprecision(1) << c.K(0, 0)
                << " fy=" << c.K(1, 1) << " cx=" << c.K(0, 2) << " cy=" << c.K(1, 2)
                << " reproj=" << std::setprecision(3) << c.reprojection_error << "px ["
                << c.method << "]\n";
    } else if (h.camera_fisheye) {
      const auto& f = *h.camera_fisheye;
      double fx = 0, fy = 0;
      auto it = f.params.find("fx"); if (it != f.params.end()) fx = it->second;
      it = f.params.find("fy"); if (it != f.params.end()) fy = it->second;
      std::cout << "  " << std::left << std::setw(20) << sid
                << " model=" << f.model_type << " fx=" << fx << " reproj="
                << std::setprecision(3) << f.reprojection_error << "px\n";
    } else if (h.imu) {
      const auto& imu = *h.imu;
      std::cout << "  " << std::left << std::setw(20) << sid
                << " gyro_ARW=" << std::scientific << std::setprecision(2) << imu.gyro_noise
                << " accel_VRW=" << imu.accel_noise << " [" << imu.method << "]\n";
    }
  }

  std::cout << "\n【外参标定结果】\n";
  for (const auto& [key, result] : extrinsics) {
    Eigen::Vector3d euler = rotation_matrix_to_euler(result.rotation, true);
    std::cout << "  " << std::left << std::setw(12) << result.pair.first
              << " -> " << std::setw(12) << result.pair.second
              << " R=[" << std::fixed << std::setprecision(2)
              << euler(0) << "," << euler(1) << "," << euler(2) << "]° "
              << " t=[" << std::setprecision(3)
              << result.translation(0) << "," << result.translation(1) << ","
              << result.translation(2) << "]m td="
              << (result.time_offset * 1000) << "ms [" << result.method_used << "]\n";
  }

  std::cout << "\n【验证指标】\n";
  for (const auto& [key, metric] : validation.metrics) {
    auto it = metric.extra.find("pass");
    bool pass = (it == metric.extra.end() || it->second != 0.0);
    std::cout << "  " << std::left << std::setw(35) << key
              << " mean=" << std::fixed << std::setprecision(2) << metric.mean_error_px << "px "
              << "<1px=" << metric.pct_within_1px << "% " << (pass ? "PASS" : "FAIL") << "\n";
  }
  std::cout << "\n  总体结果: " << validation.summary << "\n";
  std::cout << line << "\n";
}

void ReportGenerator::generate_html(
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  const std::unordered_map<std::string, CalibResult>& extrinsics,
  const ValidationReport& validation) {
  std::string path = output_dir_ + "/calibration_report.html";
  auto now = std::chrono::system_clock::now();
  time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf;
#ifdef _WIN32
  localtime_s(&tm_buf, &t);
#else
  localtime_r(&t, &tm_buf);
#endif
  char time_str[64];
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);

  std::string rows_intr;
  for (const auto& [sid, h] : intrinsics) {
    if (h.camera_pinhole) {
      const auto& c = *h.camera_pinhole;
      rows_intr += "<tr><td>" + sid + "</td><td>pinhole</td><td>" +
        std::to_string(c.K(0, 0)) + "</td><td>" + std::to_string(c.K(1, 1)) + "</td><td>" +
        std::to_string(c.K(0, 2)) + "</td><td>" + std::to_string(c.K(1, 2)) + "</td><td>" +
        std::to_string(c.reprojection_error) + "</td><td>" + c.method + "</td></tr>\n";
    } else if (h.camera_fisheye) {
      const auto& f = *h.camera_fisheye;
      double fx = 0, fy = 0, cx = 0, cy = 0;
      auto it = f.params.find("fx"); if (it != f.params.end()) fx = it->second;
      it = f.params.find("fy"); if (it != f.params.end()) fy = it->second;
      it = f.params.find("cx"); if (it != f.params.end()) cx = it->second;
      it = f.params.find("cy"); if (it != f.params.end()) cy = it->second;
      rows_intr += "<tr><td>" + sid + "</td><td>" + f.model_type + "</td><td>" +
        std::to_string(fx) + "</td><td>" + std::to_string(fy) + "</td><td>" +
        std::to_string(cx) + "</td><td>" + std::to_string(cy) + "</td><td>" +
        std::to_string(f.reprojection_error) + "</td><td>" + f.method + "</td></tr>\n";
    } else if (h.imu) {
      const auto& imu = *h.imu;
      rows_intr += "<tr><td>" + sid + "</td><td>IMU</td><td colspan=4>ARW=" +
        std::to_string(imu.gyro_noise) + " VRW=" + std::to_string(imu.accel_noise) +
        "</td><td>-</td><td>" + imu.method + "</td></tr>\n";
    }
  }

  std::string rows_extr;
  for (const auto& [key, result] : extrinsics) {
    Eigen::Vector3d euler = rotation_matrix_to_euler(result.rotation, true);
    rows_extr += "<tr><td>" + result.pair.first + "</td><td>" + result.pair.second + "</td><td>[" +
      std::to_string(euler(0)) + "," + std::to_string(euler(1)) + "," + std::to_string(euler(2)) +
      "]°</td><td>[" + std::to_string(result.translation(0)) + "," +
      std::to_string(result.translation(1)) + "," + std::to_string(result.translation(2)) +
      "]m</td><td>" + std::to_string(result.time_offset * 1000) + "ms</td><td>" +
      std::to_string(result.reprojection_error) + "</td><td>" + result.method_used + "</td></tr>\n";
  }

  std::string rows_val;
  for (const auto& [key, metric] : validation.metrics) {
    auto it = metric.extra.find("pass");
    bool pass = (it == metric.extra.end() || it->second != 0.0);
    std::string color = pass ? "green" : "red";
    rows_val += "<tr><td>" + key + "</td><td>" + std::to_string(metric.mean_error_px) +
      "</td><td>" + std::to_string(metric.median_error_px) + "</td><td>" +
      std::to_string(metric.pct_within_1px) + "%</td><td style='color:" + color + "'>" +
      (pass ? "PASS" : "FAIL") + "</td></tr>\n";
  }

  std::string overall_color = validation.overall_pass ? "green" : "red";
  std::string html = R"(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>UniCalib 标定报告</title>
<style>
  body { font-family: 'Arial', sans-serif; margin: 40px; background: #f5f5f5; }
  h1 { color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }
  h2 { color: #34495e; margin-top: 30px; }
  table { border-collapse: collapse; width: 100%; background: white;
          box-shadow: 0 1px 3px rgba(0,0,0,0.1); margin-bottom: 20px; }
  th { background: #3498db; color: white; padding: 10px; text-align: left; }
  td { padding: 8px 10px; border-bottom: 1px solid #eee; }
  tr:hover { background: #f8f9fa; }
  .summary { background: )" + overall_color + R"(; color: white; padding: 15px 25px;
              border-radius: 8px; font-size: 1.2em; display: inline-block; }
  .meta { color: #7f8c8d; font-size: 0.9em; margin-bottom: 20px; }
</style>
</head>
<body>
<h1>UniCalib 多传感器标定报告</h1>
<p class="meta">生成时间: )" + std::string(time_str) + R"(</p>
<div class="summary">总体结果: )" + validation.summary + R"(</div>
<h2>1. 内参标定结果</h2>
<table>
  <tr><th>传感器</th><th>模型</th><th>fx</th><th>fy</th><th>cx</th><th>cy</th><th>重投影误差(px)</th><th>方法</th></tr>
)" + rows_intr + R"(
</table>
<h2>2. 外参标定结果</h2>
<table>
  <tr><th>传感器A</th><th>传感器B</th><th>旋转(ZYX欧拉角)</th><th>平移</th><th>时间偏移</th><th>重投影误差(px)</th><th>方法</th></tr>
)" + rows_extr + R"(
</table>
<h2>3. 验证指标</h2>
<table>
  <tr><th>标定对</th><th>均值误差(px)</th><th>中值误差(px)</th><th>&lt;1px比例</th><th>状态</th></tr>
)" + rows_val + R"(
</table>
</body>
</html>)";

  std::ofstream f(path);
  if (f) {
    f << html;
    std::cout << "  HTML report saved: " << path << "\n";
  }
}

}  // namespace unicalib
