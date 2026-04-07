/**
 * UniCalib — LiDAR-Camera 手动标定 Pangolin 窗口 (方案 B 融合)
 *
 * 点云叠加到图像 + 左侧 6-DOF 面板，参考 SensorsCalibration lidar2camera/manual_calib。
 * 仅当 UNICALIB_WITH_PANGOLIN 时编译。
 */

#pragma once

#include "unicalib/common/calib_param.h"
#include "unicalib/pipeline/manual_calib.h"
#include <opencv2/core.hpp>
#include <optional>

namespace ns_unicalib {

#if UNICALIB_WITH_PANGOLIN
/**
 * 运行 Pangolin 手动标定窗口：主视图为点云叠加图，左侧为 6-DOF 与选项面板。
 * initial_extrinsic: 用于 Reset 按钮恢复的初值（一般为自动标定结果）。
 * 返回用户接受的外参，取消则返回 nullopt。
 */
std::optional<ExtrinsicSE3> run_lidar_cam_pangolin_panel(
    ManualExtrinsicAdjuster& adjuster,
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ManualAdjustStep& step,
    const ExtrinsicSE3* initial_extrinsic = nullptr);
#endif

}  // namespace ns_unicalib
