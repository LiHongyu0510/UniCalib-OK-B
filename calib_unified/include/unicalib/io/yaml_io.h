#pragma once
/**
 * UniCalib Unified — YAML/CSV I/O 接口
 */

#include "unicalib/common/sensor_types.h"
#include "unicalib/common/calib_param.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/intrinsic/camera_calib.h"
#include <yaml-cpp/yaml.h>
#include <string>

namespace ns_unicalib {

class YamlIO {
public:
    // 读取系统配置 (YAML)
    static SystemConfig load_system_config(const std::string& yaml_path);

    // 保存标定结果 (YAML)
    static void save_calib_results(
        const CalibParamManager& params,
        const std::string& yaml_path);

    // IMU CSV 数据加载
    // 格式: timestamp_s, gx, gy, gz, ax, ay, az
    static IMURawData load_imu_csv(const std::string& csv_path);

    // 相机内参 YAML 读写
    static CameraIntrinsics load_camera_intrinsics(const std::string& yaml_path);
    static void save_camera_intrinsics(
        const CameraIntrinsics& intrin,
        const std::string& yaml_path);

    // IMU 内参 YAML 读写（支持 app 输出格式与 YamlIO 保存格式）
    static IMUIntrinsics load_imu_intrinsics(const std::string& yaml_path);
    static void save_imu_intrinsics(
        const IMUIntrinsics& intrin,
        const std::string& yaml_path);
};

}  // namespace ns_unicalib
