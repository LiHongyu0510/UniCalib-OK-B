/**
 * UniCalib C++ — InteractiveReportGenerator 实现
 */
#include "unicalib/interactive_report.hpp"
#include "unicalib/transforms.hpp"
#include "unicalib/logger.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>

namespace unicalib {

InteractiveReportGenerator::InteractiveReportGenerator(const std::string& output_dir)
    : output_dir_(output_dir) {
  // 确保输出目录存在
  std::string mkdir_cmd = "mkdir -p " + output_dir_;
  system(mkdir_cmd.c_str());
}

std::string InteractiveReportGenerator::generate_interactive_html(
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const std::unordered_map<std::string, CalibResult>& extrinsics,
    const ValidationReport& validation,
    const InteractiveReportConfig& config) {
  
  std::string html_path = output_dir_ + "/interactive_calibration_report.html";
  
  // 获取时间戳
  auto now = std::time(nullptr);
  auto tm = *std::localtime(&now);
  char time_str[64];
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm);
  
  // 生成各部分内容
  std::string summary_section = generate_summary_section(validation);
  std::string intrinsics_section = generate_intrinsics_section(intrinsics);
  std::string extrinsics_section = generate_extrinsics_section(extrinsics);
  std::string validation_section = generate_validation_section(validation);
  std::string plotly_scripts = generate_plotly_scripts(extrinsics);
  
  // 构建 HTML
  std::string html = R"(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>UniCalib 交互式标定报告</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
<style>
  body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 0; background: #f8f9fa; }
  .navbar { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 15px 30px; }
  .navbar h1 { color: white; margin: 0; font-size: 1.8em; }
  .meta { color: rgba(255,255,255,0.8); margin-top: 5px; font-size: 0.9em; }
  .container { max-width: 1400px; margin: 30px auto; padding: 0 20px; }
  .card { border: none; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.08); margin-bottom: 25px; }
  .card-header { background: white; border-bottom: 2px solid #f0f0f0; padding: 15px 20px; }
  .card-header h2 { margin: 0; color: #2c3e50; font-size: 1.3em; }
  .card-body { padding: 20px; }
  .summary-badge { display: inline-block; padding: 12px 25px; border-radius: 25px; 
                font-size: 1.1em; font-weight: bold; }
  .summary-pass { background: #10b981; color: white; }
  .summary-fail { background: #ef4444; color: white; }
  table { width: 100%; border-collapse: collapse; }
  th { background: #f8f9fa; color: #495057; font-weight: 600; padding: 12px; text-align: left; border-bottom: 2px solid #dee2e6; }
  td { padding: 10px 12px; border-bottom: 1px solid #e9ecef; }
  tr:hover { background: #f8f9fa; }
  .metric-pass { color: #10b981; font-weight: 600; }
  .metric-fail { color: #ef4444; font-weight: 600; }
  .plot-container { margin: 20px 0; border-radius: 8px; overflow: hidden; }
  #sensor3d { width: 100%; height: 600px; background: #1a1a2e; border-radius: 12px; }
  .nav-tabs .nav-link { border: none; color: #495057; padding: 10px 20px; }
  .nav-tabs .nav-link.active { background: #667eea; color: white; border-radius: 8px; }
</style>
</head>
<body>
<nav class="navbar">
  <div>
    <h1>🎯 UniCalib 多传感器标定报告</h1>
    <div class="meta">生成时间: )" + std::string(time_str) + R"(</div>
  </div>
</nav>

<div class="container">
  <!-- 总体摘要 -->
  <div class="card">
    <div class="card-header"><h2>📊 总体摘要</h2></div>
    <div class="card-body">
      )" + summary_section + R"(
    </div>
  </div>

  <!-- 内参标定结果 -->
  <div class="card">
    <div class="card-header"><h2>🎥 内参标定结果</h2></div>
    <div class="card-body">
      )" + intrinsics_section + R"(
    </div>
  </div>

  <!-- 外参标定结果 -->
  <div class="card">
    <div class="card-header"><h2>🔗 外参标定结果</h2></div>
    <div class="card-body">
      )" + extrinsics_section + R"(
    </div>
  </div>

  <!-- 验证指标与可视化 -->
  <div class="card">
    <div class="card-header"><h2>✓ 验证指标</h2></div>
    <div class="card-body">
      <ul class="nav nav-tabs" id="validationTabs" role="tablist">
        <li class="nav-item">
          <a class="nav-link active" data-bs-toggle="tab" href="#metrics">指标汇总</a>
        </li>
        <li class="nav-item">
          <a class="nav-link" data-bs-toggle="tab" href="#charts">交互式图表</a>
        </li>
        <li class="nav-item">
          <a class="nav-link" data-bs-toggle="tab" href="#sensor3d">3D传感器视图</a>
        </li>
      </ul>
      <div class="tab-content pt-3">
        <div class="tab-pane fade show active" id="metrics">
          )" + validation_section + R"(
        </div>
        <div class="tab-pane fade" id="charts">
          <div class="plot-container">
            <div id="errorHistogram"></div>
          </div>
          <div class="plot-container">
            <div id="errorCDF"></div>
          </div>
          <div class="plot-container">
            <div id="residualPlot"></div>
          </div>
        </div>
        <div class="tab-pane fade" id="sensor3d">
          <div id="sensor3d"></div>
          <div class="info-box" style="background: #e7f3ff; border-left: 4px solid #2196f3; padding: 15px; margin: 15px 0; border-radius: 4px;">
            <strong>操作说明:</strong><br>
            • 鼠标左键拖动: 旋转视角<br>
            • 鼠标右键拖动: 平移<br>
            • 鼠标滚轮: 缩放<br>
            • 双击传感器名称: 隐藏/显示该传感器
          </div>
        </div>
      </div>
    </div>
  </div>

  <!-- 精度说明 -->
  <div class="card">
    <div class="card-header"><h2>📏 精度说明</h2></div>
    <div class="card-body">
      <table class="table">
        <thead>
          <tr>
            <th>标定任务</th>
            <th>粗标定精度</th>
            <th>精标定精度</th>
            <th>合格阈值</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>相机内参(fx,fy)</td>
            <td>±5%</td>
            <td>±0.1% (reproj&lt;0.3px)</td>
            <td>reproj&lt;1.0px</td>
          </tr>
          <tr>
            <td>IMU-LiDAR旋转</td>
            <td>~1-3°</td>
            <td>~0.05-0.1°</td>
            <td>-</td>
          </tr>
          <tr>
            <td>IMU-LiDAR平移</td>
            <td>~5-10cm</td>
            <td>~1-3cm</td>
            <td>-</td>
          </tr>
          <tr>
            <td>LiDAR-Camera旋转</td>
            <td>~1-5°</td>
            <td>~0.02-0.05°</td>
            <td>-</td>
          </tr>
          <tr>
            <td>LiDAR-Camera平移</td>
            <td>~5-20cm</td>
            <td>~1-5mm</td>
            <td>-</td>
          </tr>
          <tr>
            <td>时间偏移</td>
            <td>N/A</td>
            <td>~0.1-1ms</td>
            <td>&lt;5ms</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"></script>
)" + plotly_scripts + R"(
</body>
</html>)";

  // 写入文件
  std::ofstream f(html_path);
  if (f) {
    f << html;
    LOG_INFO(std::string("Interactive HTML report saved: ") + html_path);
  } else {
    LOG_ERROR(std::string("Failed to write HTML report: ") + html_path);
  }
  
  return html_path;
}

std::string InteractiveReportGenerator::generate_summary_section(
    const ValidationReport& validation) {
  
  bool overall_pass = validation.overall_pass;
  std::string summary_text = validation.summary;
  std::string badge_class = overall_pass ? "summary-pass" : "summary-fail";
  std::string status_text = overall_pass ? "✓ PASS" : "✗ FAIL";
  
  std::ostringstream oss;
  oss << "<div class=\"summary-badge " << badge_class << "\">\n";
  oss << "  总体状态: " << status_text << " - " << summary_text << "\n";
  oss << "</div>\n";
  
  oss << "<div class=\"info-box\" style=\"background: #e7f3ff; border-left: 4px solid #2196f3; padding: 15px; margin-top: 20px; border-radius: 4px;\">\n";
  oss << "  <strong>关键指标:</strong><br>\n";
  
  // 计算平均重投影误差
  double avg_error = 0.0;
  if (!validation.metrics.empty()) {
    for (const auto& [key, metric] : validation.metrics) {
      avg_error += metric.mean_error_px;
    }
    avg_error /= validation.metrics.size();
  }
  
  // 计算通过率
  double pass_rate = 0.0;
  if (!validation.metrics.empty()) {
    int pass_count = 0;
    for (const auto& [key, metric] : validation.metrics) {
      auto it = metric.extra.find("pass");
      if (it == metric.extra.end() || it->second != 0.0) {
        pass_count++;
      }
    }
    pass_rate = static_cast<double>(pass_count) / validation.metrics.size() * 100.0;
  }
  
  oss << "  平均重投影误差: " << std::fixed << std::setprecision(3) << avg_error << " px<br>\n";
  oss << "  验证通过率: " << std::fixed << std::setprecision(1) << pass_rate << "%\n";
  oss << "</div>\n";
  
  return oss.str();
}

std::string InteractiveReportGenerator::generate_intrinsics_section(
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics) {
  
  std::ostringstream oss;
  oss << "<table class=\"table\">\n";
  oss << "  <thead>\n";
  oss << "    <tr>\n";
  oss << "      <th>传感器</th><th>模型</th><th>fx</th><th>fy</th>\n";
  oss << "      <th>cx</th><th>cy</th><th>重投影误差(px)</th><th>方法</th>\n";
  oss << "    </tr>\n";
  oss << "  </thead>\n";
  oss << "  <tbody>\n";
  
  for (const auto& [sid, h] : intrinsics) {
    if (h.camera_pinhole) {
      const auto& c = *h.camera_pinhole;
      oss << "    <tr>\n";
      oss << "      <td>" << sid << "</td><td>pinhole</td>\n";
      oss << "      <td>" << std::fixed << std::setprecision(2) << c.K(0,0) << "</td>\n";
      oss << "      <td>" << c.K(1,1) << "</td>\n";
      oss << "      <td>" << c.K(0,2) << "</td>\n";
      oss << "      <td>" << c.K(1,2) << "</td>\n";
      oss << "      <td>" << std::setprecision(4) << c.reprojection_error << "</td>\n";
      oss << "      <td>" << c.method << "</td>\n";
      oss << "    </tr>\n";
    } else if (h.camera_fisheye) {
      const auto& f = *h.camera_fisheye;
      double fx = 0, fy = 0, cx = 0, cy = 0;
      auto it = f.params.find("fx"); if (it != f.params.end()) fx = it->second;
      it = f.params.find("fy"); if (it != f.params.end()) fy = it->second;
      it = f.params.find("cx"); if (it != f.params.end()) cx = it->second;
      it = f.params.find("cy"); if (it != f.params.end()) cy = it->second;
      
      oss << "    <tr>\n";
      oss << "      <td>" << sid << "</td><td>" << f.model_type << "</td>\n";
      oss << "      <td>" << fx << "</td><td>" << fy << "</td>\n";
      oss << "      <td>" << cx << "</td><td>" << cy << "</td>\n";
      oss << "      <td>" << std::setprecision(4) << f.reprojection_error << "</td>\n";
      oss << "      <td>" << f.method << "</td>\n";
      oss << "    </tr>\n";
    } else if (h.imu) {
      const auto& imu = *h.imu;
      oss << "    <tr>\n";
      oss << "      <td>" << sid << "</td><td>IMU</td>\n";
      oss << "      <td colspan=\"4\">ARW=" << std::scientific << std::setprecision(2) 
          << imu.gyro_noise << " VRW=" << imu.accel_noise << "</td>\n";
      oss << "      <td>-</td><td>" << imu.method << "</td>\n";
      oss << "    </tr>\n";
    }
  }
  
  oss << "  </tbody>\n";
  oss << "</table>\n";
  
  return oss.str();
}

std::string InteractiveReportGenerator::generate_extrinsics_section(
    const std::unordered_map<std::string, CalibResult>& extrinsics) {
  
  std::ostringstream oss;
  oss << "<table class=\"table\">\n";
  oss << "  <thead>\n";
  oss << "    <tr>\n";
  oss << "      <th>传感器A</th><th>传感器B</th><th>旋转(ZYX欧拉角)</th>\n";
  oss << "      <th>平移</th><th>时间偏移</th><th>重投影误差(px)</th><th>方法</th>\n";
  oss << "    </tr>\n";
  oss << "  </thead>\n";
  oss << "  <tbody>\n";
  
  for (const auto& [key, result] : extrinsics) {
    Eigen::Vector3d euler = rotation_matrix_to_euler(result.rotation, true);
    
    oss << "    <tr>\n";
    oss << "      <td>" << result.pair.first << "</td><td>" << result.pair.second << "</td>\n";
    oss << "      <td>[" << std::fixed << std::setprecision(3) 
        << euler(0) << ", " << euler(1) << ", " << euler(2) << "]°</td>\n";
    oss << "      <td>[" << std::setprecision(4) << result.translation(0) << ", "
        << result.translation(1) << ", " << result.translation(2) << "]m</td>\n";
    oss << "      <td>" << std::fixed << std::setprecision(2) 
        << result.time_offset * 1000 << "ms</td>\n";
    oss << "      <td>" << std::setprecision(4) << result.reprojection_error << "</td>\n";
    oss << "      <td>" << result.method_used << "</td>\n";
    oss << "    </tr>\n";
  }
  
  oss << "  </tbody>\n";
  oss << "</table>\n";
  
  return oss.str();
}

std::string InteractiveReportGenerator::generate_validation_section(
    const ValidationReport& validation) {
  
  std::ostringstream oss;
  oss << "<table class=\"table\">\n";
  oss << "  <thead>\n";
  oss << "    <tr>\n";
  oss << "      <th>标定对</th><th>均值误差(px)</th><th>中值误差(px)</th>\n";
  oss << "      <th>&lt;1px比例</th><th>状态</th>\n";
  oss << "    </tr>\n";
  oss << "  </thead>\n";
  oss << "  <tbody>\n";
  
  for (const auto& [key, metric] : validation.metrics) {
    auto it = metric.extra.find("pass");
    bool pass = (it == metric.extra.end() || it->second != 0.0);
    std::string status_class = pass ? "metric-pass" : "metric-fail";
    std::string status_text = pass ? "PASS" : "FAIL";
    
    oss << "    <tr>\n";
    oss << "      <td>" << key << "</td>\n";
    oss << "      <td>" << std::fixed << std::setprecision(3) << metric.mean_error_px << "</td>\n";
    oss << "      <td>" << metric.median_error_px << "</td>\n";
    oss << "      <td>" << std::setprecision(1) << metric.pct_within_1px << "%</td>\n";
    oss << "      <td class=\"" << status_class << "\">" << status_text << "</td>\n";
    oss << "    </tr>\n";
  }
  
  oss << "  </tbody>\n";
  oss << "</table>\n";
  
  return oss.str();
}

std::string InteractiveReportGenerator::generate_plotly_scripts(
    const std::unordered_map<std::string, CalibResult>& extrinsics) {
  
  std::ostringstream oss;
  oss << "<script>\n";
  
  // 3D传感器视图
  std::string sensor_3d = generate_sensor_3d_view(extrinsics);
  oss << sensor_3d;
  
  // 生成空的Plotly图表（实际数据由JavaScript动态加载）
  oss << "\n";
  oss << "// 误差分布直方图\n";
  oss << "const errorTrace = {\n";
  oss << "  x: [],\n";
  oss << "  type: 'histogram',\n";
  oss << "  name: '误差分布',\n";
  oss << "  marker: { color: 'rgba(102, 126, 234, 0.7)', line: { color: 'rgba(102, 126, 234, 1)' } },\n";
  oss << "  nbinsx: 50\n";
  oss << "};\n";
  oss << "const errorLayout = {\n";
  oss << "  title: '重投影误差分布',\n";
  oss << "  xaxis: { title: '误差 (像素)' },\n";
  oss << "  yaxis: { title: '频次' },\n";
  oss << "  margin: { t: 30, b: 40, l: 50, r: 20 },\n";
  oss << "  height: 350\n";
  oss << "};\n";
  oss << "Plotly.newPlot('errorHistogram', [errorTrace], errorLayout, {responsive: true});\n\n";
  
  // 误差CDF
  oss << "// 误差CDF曲线\n";
  oss << "const cdfTrace = {\n";
  oss << "  x: [],\n";
  oss << "  y: [],\n";
  oss << "  type: 'scatter',\n";
  oss << "  mode: 'lines',\n";
  oss << "  name: '累积分布',\n";
  oss << "  line: { color: 'rgb(239, 68, 68)', width: 2 }\n";
  oss << "};\n";
  oss << "const cdfLayout = {\n";
  oss << "  title: '误差累积分布函数',\n";
  oss << "  xaxis: { title: '误差 (像素)' },\n";
  oss << "  yaxis: { title: '累积比例 (%)', range: [0, 100] },\n";
  oss << "  margin: { t: 30, b: 40, l: 60, r: 20 },\n";
  oss << "  height: 350\n";
  oss << "};\n";
  oss << "Plotly.newPlot('errorCDF', [cdfTrace], cdfLayout, {responsive: true});\n\n";
  
  oss << "</script>\n";
  
  return oss.str();
}

std::string InteractiveReportGenerator::generate_sensor_3d_view(
    const std::unordered_map<std::string, CalibResult>& extrinsics) {
  
  std::ostringstream oss;
  
  oss << "// 3D传感器可视化\n";
  oss << "const sensor3dData = [];\n";
  oss << "const sensor3dLayout = {\n";
  oss << "  title: '传感器3D布局',\n";
  oss << "  scene: {\n";
  oss << "    xaxis: { title: 'X (m)' },\n";
  oss << "    yaxis: { title: 'Y (m)' },\n";
  oss << "    zaxis: { title: 'Z (m)' },\n";
  oss << "    aspectmode: 'cube'\n";
  oss << "  },\n";
  oss << "  margin: { t: 30, b: 0, l: 0, r: 0 }\n";
  oss << "};\n";
  
  // 遍历外参结果
  std::vector<std::string> trace_names;
  for (const auto& [key, result] : extrinsics) {
    std::string name = result.pair.first + "-" + result.pair.second;
    trace_names.push_back(name);
    
    oss << "const trace" << name << " = {\n";
    oss << "  x: [" << (-result.rotation.transpose() * result.translation)(0) << "],\n";
    oss << "  y: [" << (-result.rotation.transpose() * result.translation)(1) << "],\n";
    oss << "  z: [" << (-result.rotation.transpose() * result.translation)(2) << "],\n";
    
    // 根据传感器类型设置颜色
    std::string color = "blue";
    if (result.pair.second.find("lidar") != std::string::npos) {
      color = "red";
    } else if (result.pair.second.find("imu") != std::string::npos) {
      color = "green";
    }
    
    oss << "  mode: 'markers',\n";
    oss << "  type: 'scatter3d',\n";
    oss << "  marker: { size: 10, color: '" << color << "', opacity: 0.8 },\n";
    oss << "  name: '" << name << "'\n";
    oss << "};\n";
  }
  
  // 添加IMU原点
  oss << "const traceIMU = {\n";
  oss << "  x: [0], y: [0], z: [0],\n";
  oss << "  mode: 'markers',\n";
  oss << "  type: 'scatter3d',\n";
  oss << "  marker: { size: 10, color: 'green', opacity: 0.8 },\n";
  oss << "  name: 'IMU (Reference)'\n";
  oss << "};\n";
  trace_names.push_back("traceIMU");
  
  // 组合所有轨迹
  oss << "sensor3dData = [";
  for (size_t i = 0; i < trace_names.size(); ++i) {
    oss << trace_names[i];
    if (i < trace_names.size() - 1) oss << ", ";
  }
  oss << "];\n";
  
  oss << "Plotly.newPlot('sensor3d', sensor3dData, sensor3dLayout, {responsive: true});\n";
  
  return oss.str();
}

void InteractiveReportGenerator::generate_plotly_chart(
    const std::vector<double>& errors,
    const std::string& chart_type,
    const std::string& title,
    const std::string& output_file) {
  
  std::string output_path = output_dir_ + "/" + output_file;
  
  // 生成Python脚本
  std::string script_path = output_dir_ + "/temp_plotly_script.py";
  std::ofstream script(script_path);
  
  script << "#!/usr/bin/env python3\n";
  script << "import plotly.graph_objects as go\n";
  script << "import numpy as np\n";
  script << "import json\n\n";
  
  script << "errors = [";
  for (size_t i = 0; i < errors.size(); ++i) {
    script << errors[i];
    if (i < errors.size() - 1) script << ", ";
  }
  script << "]\n\n";
  
  if (chart_type == "histogram") {
    script << "fig = go.Figure(data=[go.Histogram(x=errors, nbinsx=50,\n";
    script << "                                   marker_color='rgba(102, 126, 234, 0.7)',\n";
    script << "                                   marker_line_color='rgba(102, 126, 234, 1)')])\n";
    script << "fig.update_layout(title='" << title << "',\n";
    script << "                  xaxis_title='Error (pixels)',\n";
    script << "                  yaxis_title='Count')\n";
  } else if (chart_type == "cdf") {
    std::vector<double> sorted_errors = errors;
    std::sort(sorted_errors.begin(), sorted_errors.end());
    
    script << "errors_sorted = np.sort(np.array(errors))\n";
    script << "cdf = np.arange(1, len(errors_sorted) + 1) / len(errors_sorted) * 100\n";
    script << "fig = go.Figure(data=[go.Scatter(x=errors_sorted, y=cdf, mode='lines')])\n";
    script << "fig.update_layout(title='" << title << "',\n";
    script << "                  xaxis_title='Error (pixels)',\n";
    script << "                  yaxis_title='Cumulative Percentage (%)')\n";
  } else if (chart_type == "residual") {
    script << "fig = go.Figure(data=[go.Scatter(x=list(range(1, len(errors)+1)), y=errors, mode='lines+markers')])\n";
    script << "fig.update_layout(title='" << title << "',\n";
    script << "                  xaxis_title='Iteration',\n";
    script << "                  yaxis_title='Residual')\n";
  }
  
  script << "fig.write_html('" << output_path << "')\n";
  script.close();
  
  // 执行Python脚本
  std::string cmd = "python3 " + script_path + " && rm " + script_path;
  int ret = system(cmd.c_str());
  
  if (ret == 0) {
    LOG_INFO(std::string("Generated Plotly chart: ") + output_path);
  } else {
    LOG_WARNING("Failed to generate Plotly chart");
  }
}

}  // namespace unicalib
