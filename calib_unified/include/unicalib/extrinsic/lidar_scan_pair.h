#pragma once
/**
 * 与具体标定算法无关的 LiDAR 扫描序列工具（例如 LiDAR-LiDAR 手动微调选帧）。
 */
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include <cstddef>
#include <utility>
#include <vector>

namespace ns_unicalib {

/** 手动微调用：在 ref/tgt 序列中各选一帧，使 |timestamp_ref - timestamp_tgt| 最小（且点云非空）。 */
std::pair<std::size_t, std::size_t> time_aligned_lidar_scan_indices_for_manual(
    const std::vector<LiDARScan>& ref,
    const std::vector<LiDARScan>& tgt);

}  // namespace ns_unicalib
