/**
 * UniCalib C++ — 标定报告生成器
 * 对应 Python: unicalib/validation/report_generator.py
 */
#pragma once

#include <string>
#include <unordered_map>
#include "calib_result.hpp"
#include "sensor_config.hpp"
#include "system.hpp"

namespace unicalib {

/** 标定报告生成器：终端汇总 + HTML 报告 */
class ReportGenerator {
 public:
  explicit ReportGenerator(const std::string& output_dir);

  /** 在终端打印内参/外参/验证汇总 */
  void print_summary(
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const std::unordered_map<std::string, CalibResult>& extrinsics,
    const ValidationReport& validation);

  /** 生成 calibration_report.html */
  void generate_html(
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const std::unordered_map<std::string, CalibResult>& extrinsics,
    const ValidationReport& validation);

 private:
  std::string output_dir_;
};

}  // namespace unicalib
