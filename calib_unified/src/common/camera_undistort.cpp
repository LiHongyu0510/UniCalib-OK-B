/**
 * 相机图像去畸变实现
 */

#include "unicalib/common/camera_undistort.h"
#include "unicalib/common/logger.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace ns_unicalib {

bool camera_has_distortion(const CameraIntrinsics& intrin) {
    for (double d : intrin.dist_coeffs) {
        if (std::abs(d) > 1e-12) return true;
    }
    return false;
}

bool camera_intrin_valid_for_undistort(const CameraIntrinsics& intrin) {
    return intrin.fx > 0.0 && intrin.fy > 0.0;
}

static cv::Mat make_K_mat(const CameraIntrinsics& intrin) {
    return (cv::Mat_<double>(3, 3) <<
        intrin.fx, 0, intrin.cx,
        0, intrin.fy, intrin.cy,
        0, 0, 1);
}

static cv::Mat make_D_mat(const CameraIntrinsics& intrin) {
    if (intrin.dist_coeffs.empty()) return cv::Mat();
    return cv::Mat(intrin.dist_coeffs).reshape(1, 1).clone();
}

cv::Mat undistort_camera_image(const cv::Mat& src, const CameraIntrinsics& intrin) {
    if (src.empty()) return cv::Mat();
    if (!camera_has_distortion(intrin)) return src.clone();
    if (!camera_intrin_valid_for_undistort(intrin)) {
        UNICALIB_WARN("[Undistort] 内参无效 (fx/fy)，跳过去畸变");
        return src.clone();
    }

    const cv::Mat K = make_K_mat(intrin);
    cv::Mat D = make_D_mat(intrin);

    if (intrin.model == CameraIntrinsics::Model::FISHEYE) {
        if (intrin.dist_coeffs.size() < 4) {
            UNICALIB_WARN("[Undistort] 鱼眼模型畸变系数不足 4 个，跳过去畸变");
            return src.clone();
        }
        if (D.cols < 4) {
            cv::Mat D4 = cv::Mat::zeros(1, 4, CV_64F);
            for (int i = 0; i < std::min(4, D.cols); ++i)
                D4.at<double>(0, i) = D.at<double>(0, i);
            D = D4;
        }
    } else {
        if (intrin.dist_coeffs.size() < 4) {
            UNICALIB_WARN("[Undistort] 针孔模型畸变系数不足 4 个 (k1,k2,p1,p2)，跳过去畸变");
            return src.clone();
        }
    }

    const int w = intrin.width > 0 ? intrin.width : src.cols;
    const int h = intrin.height > 0 ? intrin.height : src.rows;
    if (src.cols != w || src.rows != h) {
        UNICALIB_WARN(
            "[Undistort] 图像尺寸 {}x{} 与内参标称 {}x{} 不一致，仍按内参 K/D 去畸变",
            src.cols, src.rows, w, h);
    }

    cv::Mat dst;
    try {
        if (intrin.model == CameraIntrinsics::Model::FISHEYE) {
            cv::fisheye::undistortImage(src, dst, K, D, K, cv::Size(w, h));
        } else {
            cv::undistort(src, dst, K, D, K);
        }
    } catch (const cv::Exception& e) {
        UNICALIB_ERROR("[Undistort] OpenCV 去畸变失败: {}", e.what());
        return src.clone();
    }

    if (dst.empty()) {
        UNICALIB_WARN("[Undistort] 去畸变输出为空，使用原图");
        return src.clone();
    }
    return dst;
}

CameraIntrinsics pinhole_intrinsics_without_distortion(const CameraIntrinsics& intrin) {
    CameraIntrinsics out = intrin;
    out.dist_coeffs.clear();
    if (out.width <= 0 || out.height <= 0) { /* keep from intrin */ }
    return out;
}

LidarCamPreparedImages prepare_lidar_cam_calibration_images(
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const CameraIntrinsics& cam_intrin) {

    LidarCamPreparedImages prep;
    prep.intrin = cam_intrin;
    prep.frames = camera_frames;

    if (!camera_has_distortion(cam_intrin)) {
        prep.undistorted = false;
        return prep;
    }
    if (!camera_intrin_valid_for_undistort(cam_intrin)) {
        UNICALIB_WARN(
            "[LiDAR-Cam] 已配置畸变系数但内参 fx/fy 无效，标定将使用原始畸变图像");
        prep.undistorted = false;
        return prep;
    }

    prep.frames.clear();
    prep.frames.reserve(camera_frames.size());
    size_t n_ok = 0;
    for (const auto& [ts, img] : camera_frames) {
        cv::Mat ud = undistort_camera_image(img, cam_intrin);
        if (!ud.empty()) {
            prep.frames.emplace_back(ts, std::move(ud));
            ++n_ok;
        }
    }
    if (n_ok == 0) {
        UNICALIB_WARN("[LiDAR-Cam] 去畸变失败，回退使用原始图像");
        prep.frames = camera_frames;
        prep.intrin = cam_intrin;
        prep.undistorted = false;
        return prep;
    }

    prep.intrin = pinhole_intrinsics_without_distortion(cam_intrin);
    prep.undistorted = true;

    const char* model_str =
        cam_intrin.model == CameraIntrinsics::Model::FISHEYE ? "fisheye" : "pinhole";
    UNICALIB_INFO(
        "[LiDAR-Cam] 已对 {} 帧图像去畸变 (模型={}, fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}, "
        "畸变系数 {} 个)，标定使用针孔无畸变投影",
        n_ok, model_str, cam_intrin.fx, cam_intrin.fy, cam_intrin.cx, cam_intrin.cy,
        cam_intrin.dist_coeffs.size());
    if (cam_intrin.dist_coeffs.size() >= 4) {
        UNICALIB_INFO(
            "[LiDAR-Cam] 畸变系数: k1={:.6f} k2={:.6f} p1={:.6f} p2={:.6f}",
            cam_intrin.dist_coeffs[0], cam_intrin.dist_coeffs[1],
            cam_intrin.dist_coeffs[2], cam_intrin.dist_coeffs[3]);
        if (cam_intrin.dist_coeffs.size() > 4)
            UNICALIB_INFO("[LiDAR-Cam] 额外畸变系数 k3={:.6f}", cam_intrin.dist_coeffs[4]);
    }

    return prep;
}

}  // namespace ns_unicalib
