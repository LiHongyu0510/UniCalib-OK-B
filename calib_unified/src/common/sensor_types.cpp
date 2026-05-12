/**
 * UniCalib Unified — 传感器类型工具函数实现
 */

#include "unicalib/common/sensor_types.h"
#include <stdexcept>

namespace ns_unicalib {

const SensorDesc* SystemConfig::find_sensor(const std::string& id) const {
    for (const auto& s : sensors) {
        if (s.sensor_id == id) return &s;
    }
    return nullptr;
}

std::vector<const SensorDesc*> SystemConfig::sensors_of_type(SensorType t) const {
    std::vector<const SensorDesc*> result;
    for (const auto& s : sensors) {
        if (s.type == t) result.push_back(&s);
    }
    return result;
}

std::string sensor_type_to_str(SensorType t) {
    switch (t) {
    case SensorType::IMU:    return "imu";
    case SensorType::LiDAR:  return "lidar";
    case SensorType::CAMERA: return "camera";
    case SensorType::RADAR:  return "radar";
    case SensorType::RGBD:   return "rgbd";
    case SensorType::EVENT:  return "event";
    default:                 return "unknown";
    }
}

SensorType sensor_type_from_str(const std::string& s) {
    if (s == "imu")    return SensorType::IMU;
    if (s == "lidar")  return SensorType::LiDAR;
    if (s == "camera") return SensorType::CAMERA;
    if (s == "radar")  return SensorType::RADAR;
    if (s == "rgbd")   return SensorType::RGBD;
    if (s == "event")  return SensorType::EVENT;
    return SensorType::CAMERA; // 默认
}

std::string calib_type_to_str(CalibType t) {
    switch (t) {
    case CalibType::IMU_INTRINSIC:          return "imu_intrinsic";
    case CalibType::CAMERA_INTRINSIC:       return "camera_intrinsic";
    case CalibType::IMU_LIDAR_EXTRINSIC:    return "imu_lidar";
    case CalibType::LIDAR_CAMERA_EXTRINSIC: return "lidar_camera";
    case CalibType::CAM_CAM_EXTRINSIC:      return "cam_cam";
    case CalibType::JOINT:                  return "joint";
    default:                                return "unknown";
    }
}

std::string lidar_type_to_str(LidarType t) {
    switch (t) {
    case LidarType::SPINNING:    return "spinning";
    case LidarType::SOLID_STATE: return "solid_state";
    default:                     return "unknown";
    }
}

LidarType lidar_type_from_str(const std::string& s) {
    if (s == "spinning")     return LidarType::SPINNING;
    if (s == "solid_state")  return LidarType::SOLID_STATE;
    return LidarType::SPINNING;
}

std::string camera_model_to_str(CameraModel m) {
    switch (m) {
    case CameraModel::PINHOLE:  return "pinhole";
    case CameraModel::FISHEYE:  return "fisheye";
    case CameraModel::OMNI:     return "omni";
    default:                    return "unknown";
    }
}

CameraModel camera_model_from_str(const std::string& s) {
    if (s == "fisheye") return CameraModel::FISHEYE;
    if (s == "omni")    return CameraModel::OMNI;
    return CameraModel::PINHOLE;
}

std::string imu_model_to_str(IMUModel m) {
    switch (m) {
    case IMUModel::SCALE_MISALIGNMENT: return "scale_misalignment";
    case IMUModel::SCALE_ONLY:         return "scale_only";
    case IMUModel::BIAS_ONLY:          return "bias_only";
    default:                           return "scale_misalignment";
    }
}

IMUModel imu_model_from_str(const std::string& s) {
    if (s == "bias_only")          return IMUModel::BIAS_ONLY;
    if (s == "scale_only")         return IMUModel::SCALE_ONLY;
    if (s == "scale_misalignment") return IMUModel::SCALE_MISALIGNMENT;
    return IMUModel::SCALE_MISALIGNMENT;
}

}  // namespace ns_unicalib
