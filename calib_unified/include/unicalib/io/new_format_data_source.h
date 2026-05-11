#pragma once

#include "unicalib/io/ros2_data_source.h"
#include <map>
#include <string>
#include <vector>

namespace ns_unicalib {

/**
 * 新采集格式适配配置（基于索引 CSV）
 *
 * 目录建议：
 *   <root_dir>/lidar/<sensor_id>.csv   (timestamp,path)
 *   <root_dir>/camera/<sensor_id>.csv  (timestamp,path)
 *   <root_dir>/imu/<sensor_id>.csv     (timestamp,gx,gy,gz,ax,ay,az)
 *   或 OEM7 短二进制：索引每行两列「占位,oem7_log.bin」（首列可填 0），见 oem7_imu_reader.h
 */
struct NewFormatConfig {
    std::string root_dir;
    std::map<std::string, std::string> lidar_index_files;
    std::map<std::string, std::string> camera_index_files;
    std::map<std::string, std::string> imu_index_files;
    std::string timestamp_unit = "s";  // s | ms | us | ns
    /// CORRIMUDATAS / IMURATECORRIMUS 增量转物理量时的 IMU 输出频率（Hz），与 bynav INSHandler 的 imu_rate 一致
    double oem7_imu_output_rate_hz = 200.0;
    /// OEM7 解码后时间戳：gps = 自 GPS 历元连续秒（默认，与报文一致）；unix = 近似 UTC 的 Unix 秒（与索引 Unix 时间对齐）
    std::string oem7_time_base = "gps";
    /// GPS 时与 UTC 闰秒差（仅 oem7_time_base=unix 时使用），需与采集年代一致，可查 IERS
    int oem7_gps_utc_leap_sec = 18;
    /// 在选定时间基上再叠加的秒偏移（细调与 LiDAR/相机时钟差）
    double oem7_time_offset_sec = 0.0;
    size_t max_frames = 0;             // 0=无限制
    double sample_interval = 0.0;      // 秒，0=不抽样
};

class NewFormatDataSource {
public:
    explicit NewFormatDataSource(const NewFormatConfig& cfg);

    bool load();
    bool is_ready() const { return loaded_; }
    std::string get_status_message() const { return status_msg_; }

    std::vector<LiDARScanRos> get_lidar_scans(const std::string& sensor_id) const;
    std::vector<CameraFrameRos> get_camera_frames(const std::string& sensor_id) const;
    std::vector<IMUFrameRos> get_imu_frames(const std::string& sensor_id) const;

    std::vector<std::string> get_lidar_ids() const;
    std::vector<std::string> get_camera_ids() const;
    std::vector<std::string> get_imu_ids() const;

private:
    NewFormatConfig cfg_;
    bool loaded_ = false;
    std::string status_msg_ = "未加载";

    std::map<std::string, std::vector<LiDARScanRos>> lidar_data_;
    std::map<std::string, std::vector<CameraFrameRos>> camera_data_;
    std::map<std::string, std::vector<IMUFrameRos>> imu_data_;

    double to_seconds(double ts) const;
    bool should_keep(double ts, double& last_kept_ts, size_t kept_count) const;
};

}  // namespace ns_unicalib

