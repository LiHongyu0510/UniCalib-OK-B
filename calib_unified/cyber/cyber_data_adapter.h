// ============================================================
// CyberDataAdapter — Cyber 消息到 UniCalib 内部结构的适配器
// ============================================================
// 说明：
//   本文件把 Apollo Cyber RT 的 sensor_msgs（PointCloud2、Image、Imu）
//   转换成项目内部的 Frame / PointCloud / ImuData 结构。
//   目前仅提供接口声明和占位实现，真正映射逻辑待补充。
// ============================================================
// 启用方法：
//   1. 包含 cyber 头文件后取消下面的 #if 0
//   2. 实现 from_cyber_pointcloud2 等函数
// ============================================================

#pragma once

// #if 0   // <--- 取消这行注释即可启用

#include "unicalib/common/sensor_types.h"
#include "unicalib/common/frame.h"          // 项目内部 Frame 定义

// Cyber 10.0.0 头文件（启用时取消注释）
// #include "cyber/cyber.h"
// #include "cyber/proto/sensor_msgs.pb.h"   // 或你实际的 proto 路径

namespace ns_unicalib {
namespace cyber {

struct CyberAdapterConfig {
    bool convert_rgb_to_gray = true;     // 相机图像是否转灰度
    double lidar_timestamp_offset = 0.0; // LiDAR 时间戳补偿
};

/**
 * Cyber PointCloud2 → 项目内部点云
 */
inline bool from_cyber_pointcloud2(
    /* const apollo::cyber::proto::PointCloud2& cloud_msg, */
    const void* /*cloud_msg*/,
    PointCloud& out_cloud,
    const CyberAdapterConfig& cfg = {})
{
    // TODO: 实现真正的字段拷贝（x,y,z,intensity,ring,timestamp）
    // 目前返回 false 表示未实现
    (void)out_cloud;
    (void)cfg;
    return false;
}

/**
 * Cyber Image → 项目内部图像帧
 */
inline bool from_cyber_image(
    /* const apollo::cyber::proto::Image& img_msg, */
    const void* /*img_msg*/,
    Frame& out_frame,
    const CyberAdapterConfig& cfg = {})
{
    // TODO: 实现 cv::Mat 构造 + 时间戳 + 相机 ID 映射
    (void)out_frame;
    (void)cfg;
    return false;
}

/**
 * Cyber Imu → 项目内部 IMU 数据
 */
inline bool from_cyber_imu(
    /* const apollo::cyber::proto::Imu& imu_msg, */
    const void* /*imu_msg*/,
    ImuData& out_imu,
    const CyberAdapterConfig& cfg = {})
{
    // TODO: 实现 acc/gyro 时间戳转换
    (void)out_imu;
    (void)cfg;
    return false;
}

/**
 * 工具函数：把内部 ExtrinsicSE3 填充到 proto::ExtrinsicSE3
 */
inline void to_proto_extrinsic(const ExtrinsicSE3& ex, /* proto::ExtrinsicSE3* out */ void* /*out*/) {
    // TODO: 实现
}

/**
 * 工具函数：把内部 CameraIntrinsic 填充到 proto
 */
inline void to_proto_camera_intrinsic(const CameraIntrinsic& intrin, /* proto::CameraIntrinsic* out */ void* /*out*/) {
    // TODO: 实现
}

} // namespace cyber
} // namespace ns_unicalib

// #endif // UNICALIB_ENABLE_CYBER