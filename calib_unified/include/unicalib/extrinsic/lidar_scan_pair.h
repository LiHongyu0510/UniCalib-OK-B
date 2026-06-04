#pragma once
/**
 * LiDAR 扫描序列工具：LiDAR-LiDAR / LiDAR-Camera 时间戳对齐。
 */
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include <opencv2/core.hpp>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ns_unicalib {

/** 从文件名 stem 解析时间戳 [s]；支持纯数字 epoch（ms/us 自动换算）。失败返回 nullopt。 */
std::optional<double> parse_timestamp_from_filename_stem(const std::string& stem);

/** 在 LiDAR 序列中为相机时间 ts_cam + time_offset 找最近且点云非空的一帧；返回索引与 |dt|。 */
std::pair<std::size_t, double> nearest_lidar_scan_index_for_camera_ts(
    const std::vector<LiDARScan>& lidar_scans,
    double ts_cam,
    double time_offset_s = 0.0);

/** LiDAR-LiDAR 手动微调：|t_ref - t_tgt| 最小的非空帧对。 */
std::pair<std::size_t, std::size_t> time_aligned_lidar_scan_indices_for_manual(
    const std::vector<LiDARScan>& ref,
    const std::vector<LiDARScan>& tgt);

struct LidarCamSyncIndexPair {
    std::size_t cam_idx = 0;
    std::size_t lidar_idx = 0;
    double dt_s = 0.0;
};

/**
 * 以相机为驱动，为每帧图像找最近 LiDAR；|dt| <= sync_threshold_s 的才保留。
 * 相机帧按时间戳升序；结果按相机时间顺序排列。
 */
std::vector<LidarCamSyncIndexPair> collect_lidar_cam_sync_index_pairs(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    double sync_threshold_s,
    double time_offset_s = 0.0);

/** LiDAR-Camera 手动微调：在同步阈值内 |dt| 最小的 (cam_idx, lidar_idx)。 */
std::pair<std::size_t, std::size_t> time_aligned_lidar_cam_indices_for_manual(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    double sync_threshold_s,
    double time_offset_s = 0.0);

/**
 * 按时间戳匹配重建序列（仅保留可配对帧，最多 max_pairs 对）。
 * 返回实际配对数量；0 表示无有效对（调用方应保留原序列或报错）。
 */
std::size_t build_time_aligned_lidar_camera_sequences(
    std::vector<LiDARScan>& lidar_out,
    std::vector<std::pair<double, cv::Mat>>& camera_out,
    const std::vector<LiDARScan>& lidar_in,
    const std::vector<std::pair<double, cv::Mat>>& camera_in,
    double sync_threshold_s,
    double time_offset_s = 0.0,
    std::size_t max_pairs = 100);

}  // namespace ns_unicalib
