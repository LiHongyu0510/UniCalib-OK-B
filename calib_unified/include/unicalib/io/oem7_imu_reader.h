#pragma once

#include "unicalib/io/ros2_data_source.h"

#include <string>
#include <vector>

namespace ns_unicalib {

/// OEM7 解码后 IMU 时间戳语义（与 LiDAR/相机索引对齐用）
enum class Oem7ImuTimeBase {
    /// 保持 OEM7 时间轴：周内秒展开为「自 GPS 历元 1980-01-06 起的连续秒」（与报文 week+sow 一致），非 Unix
    GpsSinceEpoch = 0,
    /// 近似 UTC 的 Unix 秒：315964800 + gps_since_epoch - gps_minus_utc_leap_sec + time_offset_sec
    /// Bynav 文本 RAWIMU 已是 Unix、CORRIMU 为 GPST，解码器按数值自动识别，避免重复换算
    UnixUtcApprox = 1,
};

/// OEM7 / Bynav 惯导日志解码参数
struct Oem7ImuDecodeParams {
    double imu_output_rate_hz = 200.0;
    Oem7ImuTimeBase time_base = Oem7ImuTimeBase::GpsSinceEpoch;
    int gps_minus_utc_leap_sec = 18;
    double time_offset_sec = 0.0;
    /// RAWIMUSX 用：-1 表示从报文 INSCONFIG/RAWIMUSX 自动推断
    int imu_type = -1;
    /// 非 0 时覆盖 oem7_imu_scale 查表（与 bynav 节点参数 imu_gyro_scale_factor 一致）
    double imu_gyro_scale_factor = 0.0;
    double imu_accel_scale_factor = 0.0;
};

/**
 * 从 NovAtel OEM7 二进制流或 Bynav 驱动 ASCII .log 解析 IMU。
 *
 * 自动识别：
 *   - OEM7 二进制（0xAA 0x44 0x12/0x13）：CORRIMUDATAS(813)、IMURATECORRIMUS(1362)、CORRIMUS(2264)、RAWIMUSX(1462)、INSCONFIG(1945)
 *   - Bynav 文本 .log：spdlog 行 accel:[..] gyro:[..]（与驱动 Record*ImuData 一致），或 7 列 CSV
 *
 * 逻辑对齐 bynav_ros_driver ins_handler.cpp + oem7_imu.cpp。
 */
bool decode_oem7_imu_binary_file(const std::string& abs_path,
                                 const Oem7ImuDecodeParams& params,
                                 std::vector<IMUFrameRos>& out,
                                 std::string& err_msg);

bool decode_oem7_corr_imu_short_binary_file(const std::string& abs_path,
                                            double imu_output_rate_hz,
                                            std::vector<IMUFrameRos>& out,
                                            std::string& err_msg);

}  // namespace ns_unicalib
