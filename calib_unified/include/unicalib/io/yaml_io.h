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
    // sensor_id 非空且 YAML 含 cameras 映射时，从 cameras[sensor_id] 读取（多相机合一文件）
    static CameraIntrinsics load_camera_intrinsics(
        const std::string& yaml_path,
        const std::string& sensor_id = "");
    static void save_camera_intrinsics(
        const CameraIntrinsics& intrin,
        const std::string& yaml_path);

    // IMU 内参 YAML 读写（支持 app 输出格式与 YamlIO 保存格式）
    static IMUIntrinsics load_imu_intrinsics(const std::string& yaml_path);
    static void save_imu_intrinsics(
        const IMUIntrinsics& intrin,
        const std::string& yaml_path);
};

/// 解析 YAML 4×4 行优先矩阵节点（rows/cols/data），返回 16 个 double 的 vector，格式无效时返回空
std::vector<double> parse_4x4_matrix_from_yaml(const YAML::Node& node);

/// 解析 3×3 旋转或 4×4 SE3（行优先）；3×3 时平移补 0，供 IMU-LiDAR initial_extrinsics 使用
std::vector<double> parse_se3_matrix_from_yaml(const YAML::Node& node);

}  // namespace ns_unicalib
