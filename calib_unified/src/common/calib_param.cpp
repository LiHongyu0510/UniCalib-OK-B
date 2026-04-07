/**
 * UniCalib — 标定参数管理器实现
 *
 * 工程化改造:
 *   - 使用统一的异常体系
 *   - 改造 YAML 加载异常处理
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/sensor_types.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace ns_unicalib {

// ===================================================================
// ExtrinsicSE3 — 欧拉角
// ===================================================================
Eigen::Vector3d ExtrinsicSE3::euler_deg() const {
    Eigen::Matrix3d R = SO3_TargetInRef.matrix();
    // ZYX 欧拉角 (Yaw-Pitch-Roll)
    double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
    double roll, pitch, yaw;
    if (sy > 1e-6) {
        roll  = std::atan2( R(2,1), R(2,2));
        pitch = std::atan2(-R(2,0), sy);
        yaw   = std::atan2( R(1,0), R(0,0));
    } else {
        roll  = std::atan2(-R(1,2), R(1,1));
        pitch = std::atan2(-R(2,0), sy);
        yaw   = 0.0;
    }
    return Eigen::Vector3d(roll, pitch, yaw) * (180.0 / M_PI);
}

// ===================================================================
// CalibParamManager
// ===================================================================
ExtrinsicSE3::Ptr CalibParamManager::get_extrinsic(
    const std::string& ref, const std::string& target) const {
    auto key = extrinsic_key(ref, target);
    auto it = extrinsics.find(key);
    if (it != extrinsics.end()) return it->second;
    // 尝试反向
    auto key_inv = extrinsic_key(target, ref);
    it = extrinsics.find(key_inv);
    if (it != extrinsics.end()) {
        // 返回逆变换
        auto inv = std::make_shared<ExtrinsicSE3>(*it->second);
        auto T_inv = it->second->SE3_TargetInRef().inverse();
        inv->SO3_TargetInRef = T_inv.so3();
        inv->POS_TargetInRef = T_inv.translation();
        inv->ref_sensor_id    = target;
        inv->target_sensor_id = ref;
        inv->time_offset_s   = -it->second->time_offset_s;
        return inv;
    }
    return nullptr;
}

ExtrinsicSE3::Ptr CalibParamManager::get_or_create_extrinsic(
    const std::string& ref, const std::string& target) {
    auto key = extrinsic_key(ref, target);
    auto it = extrinsics.find(key);
    if (it == extrinsics.end()) {
        auto ext = std::make_shared<ExtrinsicSE3>();
        ext->ref_sensor_id    = ref;
        ext->target_sensor_id = target;
        extrinsics[key] = ext;
        return ext;
    }
    return it->second;
}

// ===================================================================
// YAML 序列化
// ===================================================================
void CalibParamManager::save_yaml(const std::string& path) const {
    YAML::Emitter out;
    out << YAML::BeginMap;

    // IMU 内参
    if (!imu_intrinsics.empty()) {
        out << YAML::Key << "imu_intrinsics" << YAML::Value << YAML::BeginMap;
        for (const auto& [id, intrin] : imu_intrinsics) {
            if (!intrin) continue;
            out << YAML::Key << id << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "noise_gyro"        << YAML::Value << intrin->noise_gyro;
            out << YAML::Key << "bias_instab_gyro"  << YAML::Value << intrin->bias_instab_gyro;
            out << YAML::Key << "noise_accel"       << YAML::Value << intrin->noise_acce;
            out << YAML::Key << "bias_instab_accel" << YAML::Value << intrin->bias_instab_acce;
            out << YAML::Key << "bias_gyro" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << intrin->bias_gyro[0] << intrin->bias_gyro[1] << intrin->bias_gyro[2] << YAML::EndSeq;
            out << YAML::Key << "bias_accel" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << intrin->bias_acce[0] << intrin->bias_acce[1] << intrin->bias_acce[2] << YAML::EndSeq;
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
    }

    // 相机内参
    if (!camera_intrinsics.empty()) {
        out << YAML::Key << "camera_intrinsics" << YAML::Value << YAML::BeginMap;
        for (const auto& [id, intrin] : camera_intrinsics) {
            if (!intrin) continue;
            out << YAML::Key << id << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "model"  << YAML::Value
                << (intrin->model == CameraIntrinsics::Model::FISHEYE ? "fisheye" : "pinhole");
            out << YAML::Key << "width"  << YAML::Value << intrin->width;
            out << YAML::Key << "height" << YAML::Value << intrin->height;
            out << YAML::Key << "fx" << YAML::Value << intrin->fx;
            out << YAML::Key << "fy" << YAML::Value << intrin->fy;
            out << YAML::Key << "cx" << YAML::Value << intrin->cx;
            out << YAML::Key << "cy" << YAML::Value << intrin->cy;
            out << YAML::Key << "dist_coeffs" << YAML::Value
                << YAML::Flow << intrin->dist_coeffs;
            out << YAML::Key << "rms_reproj_px" << YAML::Value << intrin->rms_reproj_error;
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
    }

    // 外参
    if (!extrinsics.empty()) {
        out << YAML::Key << "extrinsics" << YAML::Value << YAML::BeginMap;
        for (const auto& [key, ext] : extrinsics) {
            if (!ext) continue;
            out << YAML::Key << key << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "ref_sensor"    << YAML::Value << ext->ref_sensor_id;
            out << YAML::Key << "target_sensor" << YAML::Value << ext->target_sensor_id;

            // SO3 as quaternion [qw, qx, qy, qz]
            Eigen::Quaterniond q = ext->SO3_TargetInRef.unit_quaternion();
            out << YAML::Key << "quaternion" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << q.w() << q.x() << q.y() << q.z() << YAML::EndSeq;

            // 平移
            out << YAML::Key << "translation_m" << YAML::Value << YAML::Flow
                << YAML::BeginSeq
                << ext->POS_TargetInRef[0]
                << ext->POS_TargetInRef[1]
                << ext->POS_TargetInRef[2]
                << YAML::EndSeq;

            // 欧拉角 (参考)
            Eigen::Vector3d euler = ext->euler_deg();
            out << YAML::Key << "euler_zyx_deg" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << euler[0] << euler[1] << euler[2] << YAML::EndSeq;

            out << YAML::Key << "time_offset_s"  << YAML::Value << ext->time_offset_s;
            out << YAML::Key << "residual_rms"   << YAML::Value << ext->residual_rms;
            out << YAML::Key << "converged"      << YAML::Value << ext->is_converged;
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
    }

    // 重力
    out << YAML::Key << "gravity" << YAML::Value << YAML::Flow
        << YAML::BeginSeq << gravity[0] << gravity[1] << gravity[2] << YAML::EndSeq;

    out << YAML::EndMap;

    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                           "无法打开文件写入标定参数: " + path);
    }
    try {
        f << out.c_str();
    } catch (const std::exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
            "写入标定参数时发生异常 path=" + path + " detail=" + e.what());
    }
    UNICALIB_INFO("CalibParams saved to: {}", path);
}

void CalibParamManager::save_json(const std::string& path) const {
    // JSON 格式通过 cereal 序列化
    // 当前用 YAML 格式作为简化实现
    UNICALIB_WARN("save_json: 使用 YAML 格式替代 (将在 V2 中实现原生 JSON)");
    save_yaml(path + ".yaml");
}

void CalibParamManager::load_yaml(const std::string& path) {
    UNICALIB_INFO("CalibParamManager::load_yaml: {}", path);
    try {
        YAML::Node root = YAML::LoadFile(path);

        // 加载外参
        if (root["extrinsics"]) {
            for (const auto& item : root["extrinsics"]) {
                std::string key = item.first.as<std::string>();
                auto ext = std::make_shared<ExtrinsicSE3>();
                const auto& v = item.second;
                ext->ref_sensor_id    = v["ref_sensor"].as<std::string>("");
                ext->target_sensor_id = v["target_sensor"].as<std::string>("");
                ext->time_offset_s    = v["time_offset_s"].as<double>(0.0);
                ext->residual_rms     = v["residual_rms"].as<double>(0.0);
                ext->is_converged     = v["converged"].as<bool>(false);

                // 优先 4x4 矩阵 (与 imu_intrinsic T_Body_Imu / imu_lidar T_Imu_Lidar 格式一致)
                const auto& t_node = v["T_Imu_Lidar"] ? v["T_Imu_Lidar"] : v["T_4x4"];
                if (t_node && t_node["rows"] && t_node["cols"] && t_node["data"] &&
                    t_node["rows"].as<int>() == 4 && t_node["cols"].as<int>() == 4 &&
                    t_node["data"].IsSequence() && t_node["data"].size() >= 16) {
                    const auto& data = t_node["data"];
                    Eigen::Matrix4d T;
                    for (int i = 0; i < 4; ++i)
                        for (int j = 0; j < 4; ++j)
                            T(i, j) = data[i * 4 + j].as<double>();
                    ext->SO3_TargetInRef = Sophus::SO3d(T.block<3,3>(0,0));
                    ext->POS_TargetInRef = T.block<3,1>(0,3);
                } else {
                    // 四元数 + 平移
                    if (v["quaternion"]) {
                        auto qv = v["quaternion"];
                        Eigen::Quaterniond q(qv[0].as<double>(), qv[1].as<double>(),
                                             qv[2].as<double>(), qv[3].as<double>());
                        ext->SO3_TargetInRef = Sophus::SO3d(q.normalized());
                    }
                    if (v["translation_m"]) {
                        auto tv = v["translation_m"];
                        ext->POS_TargetInRef = Eigen::Vector3d(
                            tv[0].as<double>(), tv[1].as<double>(), tv[2].as<double>());
                    }
                }
                extrinsics[key] = ext;
            }
        }
        UNICALIB_INFO("  加载了 {} 个外参", extrinsics.size());

    } catch (const YAML::Exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::DATA_PARSE_ERROR,
                            "load_yaml YAML 解析失败: " + path + "\n" + e.what());
    } catch (const UniCalibException&) {
        throw;
    } catch (const std::exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::DATA_PARSE_ERROR,
            "load_yaml 失败 path=" + path + " detail=" + e.what());
    }
}

void CalibParamManager::load_json(const std::string& path) {
    UNICALIB_WARN("load_json: 使用 load_yaml 替代");
    load_yaml(path);
}

// ===================================================================
// 打印摘要
// ===================================================================
void CalibParamManager::print_summary() const {
    std::cout << "\n╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║              标定参数汇总                            ║\n";
    std::cout << "╠══════════════════════════════════════════════════════╣\n";

    if (!imu_intrinsics.empty()) {
        std::cout << "║ IMU 内参:\n";
        for (const auto& [id, intrin] : imu_intrinsics) {
            if (!intrin) continue;
            std::cout << "║   " << id << ":\n"
                      << "║     陀螺噪声: " << std::scientific << std::setprecision(3)
                      << intrin->noise_gyro << " rad/s/√Hz\n"
                      << "║     加速计噪声: " << intrin->noise_acce << " m/s²/√Hz\n";
        }
    }

    if (!camera_intrinsics.empty()) {
        std::cout << "║ 相机内参:\n";
        for (const auto& [id, intrin] : camera_intrinsics) {
            if (!intrin) continue;
            std::cout << "║   " << id << ": fx=" << std::fixed << std::setprecision(2)
                      << intrin->fx << " fy=" << intrin->fy
                      << " cx=" << intrin->cx << " cy=" << intrin->cy
                      << " RMS=" << intrin->rms_reproj_error << "px\n";
        }
    }

    if (!extrinsics.empty()) {
        std::cout << "║ 外参:\n";
        for (const auto& [key, ext] : extrinsics) {
            if (!ext) continue;
            Eigen::Vector3d euler = ext->euler_deg();
            std::cout << "║   " << key << ":\n"
                      << "║     R(RPY°): [" << std::fixed << std::setprecision(2)
                      << euler[0] << ", " << euler[1] << ", " << euler[2] << "]\n"
                      << "║     T(m):    [" << ext->POS_TargetInRef[0] << ", "
                      << ext->POS_TargetInRef[1] << ", " << ext->POS_TargetInRef[2] << "]\n"
                      << "║     时偏(s): " << ext->time_offset_s << "\n"
                      << "║     RMS:     " << ext->residual_rms
                      << (ext->is_converged ? " ✓" : " ✗") << "\n";
        }
    }

    std::cout << "╚══════════════════════════════════════════════════════╝\n\n";
}

}  // namespace ns_unicalib
