/**
 * UniCalib C++ — 深度融合：第三方标定工具路径配置
 * 从 YAML third_party / external_tools 或环境变量读取。
 */
#pragma once

#include <string>
#include <unordered_map>

namespace unicalib {

/** 六方/第三方工具根目录（用于子进程调用） */
struct ExternalToolsConfig {
  std::string dm_calib;               // DM-Calib 根目录，用于针孔内参
  std::string learn_to_calibrate;     // learn-to-calibrate 根目录，用于粗外参 RL
  std::string mias_lcec;              // MIAS-LCEC 根目录，LiDAR-Camera 粗/精
  std::string ikalibr;                // iKalibr 根目录，联合优化
  std::string click_calib;            // click_calib 根目录，Camera-Camera BA / 验证
  std::string transformer_imu;       // Transformer-IMU-Calibrator，IMU 内参备选

  /** 是否启用深度融合（任一路径非空则尝试调用） */
  bool integration_enabled() const {
    return !dm_calib.empty() || !learn_to_calibrate.empty() ||
           !mias_lcec.empty() || !ikalibr.empty() ||
           !click_calib.empty() || !transformer_imu.empty();
  }

  /** 从环境变量填充未设置的路径：UNICALIB_DM_CALIB, UNICALIB_LEARN_TO_CALIB 等 */
  void apply_env_defaults();
};

}  // namespace unicalib
