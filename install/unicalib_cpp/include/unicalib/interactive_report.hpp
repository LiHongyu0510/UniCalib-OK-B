/**
 * UniCalib C++ — 交互式报告生成器
 * 对应 Python: unicalib/validation/interactive_report.py
 * 
 * 功能:
 * - 生成交互式HTML报告
 * - 使用Plotly生成交互式图表
 * - 3D传感器可视化
 */
#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include "calib_result.hpp"
#include "sensor_config.hpp"

namespace unicalib {

/** 交互式报告配置 */
struct InteractiveReportConfig {
  bool enable_3d_plot = true;        // 是否包含3D可视化
  std::string plotly_version = "2.27.0";  // Plotly版本
  std::string bootstrap_version = "5.3.0";  // Bootstrap版本
};

/** 交互式报告生成器 */
class InteractiveReportGenerator {
 public:
  explicit InteractiveReportGenerator(const std::string& output_dir);
  ~InteractiveReportGenerator() = default;

  /**
   * 生成交互式 HTML 报告
   * @param intrinsics 内参标定结果
   * @param extrinsics 外参标定结果
   * @param validation 验证报告
   * @param config 可视化配置
   * @return HTML 文件路径
   */
  std::string generate_interactive_html(
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const std::unordered_map<std::string, CalibResult>& extrinsics,
    const ValidationReport& validation,
    const InteractiveReportConfig& config = InteractiveReportConfig());

  /**
   * 生成 Plotly 图表（误差分布、残差曲线）
   * @param errors 误差数据
   * @param chart_type 图表类型 ("histogram", "cdf", "residual")
   * @param title 图表标题
   * @param output_file 输出文件名
   */
  void generate_plotly_chart(
    const std::vector<double>& errors,
    const std::string& chart_type,
    const std::string& title,
    const std::string& output_file);

  /**
   * 生成3D传感器视图（JavaScript代码）
   * @param extrinsics 外参结果
   * @return JavaScript代码字符串
   */
  std::string generate_sensor_3d_view(
    const std::unordered_map<std::string, CalibResult>& extrinsics);

 private:
  std::string output_dir_;

  // HTML 生成辅助方法
  std::string generate_summary_section(const ValidationReport& validation);
  std::string generate_intrinsics_section(
      const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics);
  std::string generate_extrinsics_section(
      const std::unordered_map<std::string, CalibResult>& extrinsics);
  std::string generate_validation_section(const ValidationReport& validation);
  std::string generate_plotly_scripts(
      const std::unordered_map<std::string, CalibResult>& extrinsics);

  // Plotly 图表生成
  std::string create_error_histogram_plot(const std::vector<double>& errors);
  std::string create_error_cdf_plot(const std::vector<double>& errors);
  std::string create_residual_plot(const std::vector<double>& iteration_errors);
};

}  // namespace unicalib
