#pragma once
/**
 * 相机图像去畸变 — 严格使用内参 (fx, fy, cx, cy) 与畸变系数
 * 供 LiDAR-Camera 等外参标定在优化前将图像转为针孔无畸变模型
 */

#include "unicalib/common/calib_param.h"
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

namespace ns_unicalib {

/** 畸变系数是否非零（任一系数绝对值 > 1e-12） */
bool camera_has_distortion(const CameraIntrinsics& intrin);

/** 是否具备去畸变所需的内参 (fx, fy > 0) */
bool camera_intrin_valid_for_undistort(const CameraIntrinsics& intrin);

/**
 * 单帧去畸变：保持原 K (newCameraMatrix = K)，输出为针孔无畸变图像。
 * PINHOLE: cv::undistort；FISHEYE: cv::fisheye::undistortImage。
 * 无畸变或内参无效时返回 src.clone()。
 */
cv::Mat undistort_camera_image(const cv::Mat& src, const CameraIntrinsics& intrin);

/** 去畸变后用于标定的内参：K 不变，畸变系数清零 */
CameraIntrinsics pinhole_intrinsics_without_distortion(const CameraIntrinsics& intrin);

struct LidarCamPreparedImages {
    std::vector<std::pair<double, cv::Mat>> frames;
    CameraIntrinsics intrin;
    bool undistorted = false;
};

/** 批量去畸变图像序列，供 LiDAR-Camera 标定使用 */
LidarCamPreparedImages prepare_lidar_cam_calibration_images(
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const CameraIntrinsics& cam_intrin);

}  // namespace ns_unicalib
