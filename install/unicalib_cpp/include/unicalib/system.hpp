/**
 * UniCalib C++ — 系统主控调度器
 * 对应 Python: unicalib/core/system.py
 * 首版提供配置加载、传感器/标定对解析、分阶段接口占位。
 */
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include "sensor_config.hpp"
#include "calib_result.hpp"
#include "data_manager.hpp"
#include "external_tools_config.hpp"

namespace unicalib {

class UniCalibSystem {
 public:
  explicit UniCalibSystem(const std::string& config_path);

  /** 执行完整四阶段流水线 */
  void run_full_pipeline(const std::string& data_path);

  /** 运行单阶段 (调试用) */
  void run_stage(const std::string& stage, const std::string& data_path);

  const std::vector<SensorConfig>& sensors_list() const { return sensors_list_; }
  const std::vector<CalibPair>& calib_pairs() const { return calib_pairs_; }
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsic_results() const {
    return intrinsic_results_;
  }
  const std::unordered_map<std::string, CalibResult>& extrinsic_results() const {
    return extrinsic_results_;
  }
  const std::string& output_dir() const { return output_dir_; }

 private:
  std::string config_path_;
  std::string output_dir_;
  std::vector<SensorConfig> sensors_list_;
  std::vector<CalibPair> calib_pairs_;
  std::unordered_map<std::string, IntrinsicResultHolder> intrinsic_results_;
  std::unordered_map<std::string, CalibResult> extrinsic_results_;  // key: "sensor_a:sensor_b"
  ExternalToolsConfig tools_config_;  // 深度融合：六方路径
  std::string pipeline_data_path_;   // 当前流水线数据目录（供 Stage2/3 子进程使用）

  void load_config();
  void load_tools_config();
  void parse_config();
  void stage_intrinsic(DataManager& data_mgr);
  void stage_coarse_extrinsic(DataManager& data_mgr);
  void stage_fine_extrinsic(DataManager& data_mgr);
  void stage_validation(DataManager& data_mgr, ValidationReport& report);
  void save_results(const ValidationReport& validation);
};

}  // namespace unicalib
