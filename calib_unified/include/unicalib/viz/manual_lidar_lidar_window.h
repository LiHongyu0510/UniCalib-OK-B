/**
 * UniCalib — LiDAR-LiDAR 手动标定 Pangolin 窗口
 *
 * 双点云融合可视化 + 左侧 6-DOF 面板，参考 SensorsCalibration lidar2lidar/manual_calib。
 * Ref 点云固定显示，Target 点云按 T_TargetInRef 变换到 Ref 系下显示，便于观察对齐效果。
 * 仅当 UNICALIB_WITH_PANGOLIN 时编译。
 */

#pragma once

#include "unicalib/common/calib_param.h"
#include "unicalib/pipeline/manual_calib.h"
#include <optional>

namespace ns_unicalib {

#if UNICALIB_WITH_PANGOLIN
/**
 * 运行 Pangolin 手动标定窗口：主视图为双 LiDAR 点云融合（Ref 系），左侧为 6-DOF 与选项面板。
 * T_TargetInRef：将 target 系下的点变换到 ref 系，即 p_ref = T_TargetInRef * p_target。
 * initial_extrinsic: 用于 Reset 按钮恢复的初值（一般为自动标定结果）。
 * 返回用户接受的外参，取消则返回 nullopt。
 */
std::optional<ExtrinsicSE3> run_lidar_lidar_pangolin_panel(
    ManualExtrinsicAdjuster& adjuster,
    const LiDARScan& ref_scan,
    const LiDARScan& target_scan,
    const ManualAdjustStep& step,
    const ExtrinsicSE3* initial_extrinsic = nullptr);
#endif

}  // namespace ns_unicalib
