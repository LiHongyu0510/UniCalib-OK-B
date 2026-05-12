/**
 * UniCalib C++ — 第三方工具路径配置实现（环境变量兜底）
 */
#include "unicalib/external_tools_config.hpp"
#include <cstdlib>

namespace unicalib {

void ExternalToolsConfig::apply_env_defaults() {
  auto get = [](const char* name) -> std::string {
    const char* v = std::getenv(name);
    return v ? std::string(v) : "";
  };
  if (dm_calib.empty()) dm_calib = get("UNICALIB_DM_CALIB");
  if (learn_to_calibrate.empty()) learn_to_calibrate = get("UNICALIB_LEARN_TO_CALIB");
  if (mias_lcec.empty()) mias_lcec = get("UNICALIB_MIAS_LCEC");
  if (ikalibr.empty()) ikalibr = get("UNICALIB_IKALIBR");
  if (click_calib.empty()) click_calib = get("UNICALIB_CLICK_CALIB");
  if (transformer_imu.empty()) transformer_imu = get("UNICALIB_TRANSFORMER_IMU");
}

}  // namespace unicalib
