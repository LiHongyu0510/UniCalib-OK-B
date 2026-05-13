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
    UnixUtcApprox = 1,
};

/// OEM7 二进制 IMU 解码参数（频率 + 时间基）
struct Oem7ImuDecodeParams {
    double imu_output_rate_hz = 200.0;
    Oem7ImuTimeBase time_base = Oem7ImuTimeBase::GpsSinceEpoch;
    /// GPS 时与 UTC 的闰秒差（IERS 公布值会随年代变化，默认 18；与 LiDAR 对不齐时可微调 time_offset_sec）
    int gps_minus_utc_leap_sec = 18;
    /// 在选定 time_base 之后再叠加的秒偏移（用于与 bag/相机 时钟对齐）
    double time_offset_sec = 0.0;
};

/**
 * 从 NovAtel OEM7 二进制流文件中解析 IMU（与 bynav_ros_driver oem7_messages.h + ins_handler.cpp 一致）。
 *
 * 支持两种后端：
 *   1) 轻量手写解析器（默认）：短二进制 CORRIMUDATAS(813)、IMURATECORRIMUS(1362)、CORRIMUS(2264)
 *   2) NovAtel EDIE 完整解析器（CMake -DUNICALIB_USE_EDIE=ON）：支持 RAWIMUSX(1462)、INSPVAX(1465)、
 *      INSPVAS(508)、INSSTDEV(2051) 等全部日志，自动处理 CRC、分帧、所有消息 ID。
 *
 * 启用 EDIE 后优先使用 EDIE，失败自动 fallback 到手写路径。EDIE 静态库需放在
 * calib_unified/thirdparty/bynav_edie/（含 usr/include + usr/lib/{x86,arm64}）。
 *
 * 时间戳默认按 GPS 连续秒；可选 `UnixUtcApprox` 转为近似 Unix UTC，便于与索引里 Unix 时间对齐。
 */
bool decode_oem7_imu_binary_file(const std::string& abs_path,
                                 const Oem7ImuDecodeParams& params,
                                 std::vector<IMUFrameRos>& out,
                                 std::string& err_msg);

/// 等价于 `decode_oem7_imu_binary_file` 且 `time_base=GpsSinceEpoch`、无额外偏移（兼容旧调用）
bool decode_oem7_corr_imu_short_binary_file(const std::string& abs_path,
                                            double imu_output_rate_hz,
                                            std::vector<IMUFrameRos>& out,
                                            std::string& err_msg);

}  // namespace ns_unicalib
