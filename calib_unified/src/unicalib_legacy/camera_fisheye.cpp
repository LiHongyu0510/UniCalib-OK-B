/**
 * UniCalib C++ — 鱼眼相机内参标定实现（OpenCV fisheye）
 */
#include "unicalib/camera_fisheye.hpp"
#include "unicalib/logger.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

#if defined(UNICALIB_USE_OPENCV) && UNICALIB_USE_OPENCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace unicalib {

std::optional<FisheyeIntrinsic> CameraFisheyeCalibrator::calibrate(
    DataManager& data_mgr, const SensorConfig& sensor) {
#if defined(UNICALIB_USE_OPENCV) && UNICALIB_USE_OPENCV
  std::vector<std::vector<cv::Point3d>> objpoints;
  std::vector<std::vector<cv::Point2d>> imgpoints;
  int image_width = 0, image_height = 0;

  const int pattern_w = kCheckerCols - 1;
  const int pattern_h = kCheckerRows - 1;
  std::vector<cv::Point3d> objp;
  for (int i = 0; i < pattern_h; ++i)
    for (int j = 0; j < pattern_w; ++j)
      objp.push_back(cv::Point3d(j * kSquareSize, i * kSquareSize, 0));

  int count = 0;
  data_mgr.iter_image_paths(
      sensor.topic,
      [&](double /*ts*/, const std::string& path) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) return;
        if (image_width == 0) {
          image_width = img.cols;
          image_height = img.rows;
        }
        cv::Mat gray;
        if (img.channels() == 3)
          cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else
          gray = img;

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray, cv::Size(pattern_w, pattern_h), corners,
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found) return;

        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 1e-6));

        std::vector<cv::Point2d> corners_d(corners.begin(), corners.end());
        objpoints.push_back(objp);
        imgpoints.push_back(corners_d);
        ++count;
      },
      kMaxFrames, kFrameSkip);

  if (count < kMinViews) {
    LOG_WARNING("Fisheye: not enough views (have " + std::to_string(count) +
                ", need " + std::to_string(kMinViews) + ") for " + sensor.sensor_id);
    return std::nullopt;
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = K.at<double>(1, 1) = std::max(image_width, image_height);
  K.at<double>(0, 2) = image_width / 2.0;
  K.at<double>(1, 2) = image_height / 2.0;
  cv::Mat D(4, 1, CV_64F);
  D.setTo(0);

  int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW;
  std::vector<cv::Mat> rvecs, tvecs;
  double rms;
  try {
    rms = cv::fisheye::calibrate(objpoints, imgpoints, cv::Size(image_width, image_height),
                                 K, D, rvecs, tvecs, flags);
  } catch (const cv::Exception& e) {
    LOG_WARNING("Fisheye calibrate failed: " + std::string(e.what()));
    return std::nullopt;
  }

  FisheyeIntrinsic out;
  out.model_type = "equidistant";
  out.image_size = {image_width, image_height};
  out.reprojection_error = rms;
  out.method = "OpenCV_fisheye";
  out.params["fx"] = K.at<double>(0, 0);
  out.params["fy"] = K.at<double>(1, 1);
  out.params["cx"] = K.at<double>(0, 2);
  out.params["cy"] = K.at<double>(1, 2);
  out.params["k1"] = D.at<double>(0);
  out.params["k2"] = D.at<double>(1);
  out.params["k3"] = D.at<double>(2);
  out.params["k4"] = D.at<double>(3);

  LOG_INFO("Fisheye intrinsic calibrated: " + sensor.sensor_id +
           " model=equidistant reproj=" + std::to_string(rms) + "px");
  return out;

#else
  (void)data_mgr;
  (void)sensor;
  LOG_WARNING("Fisheye calibration requires OpenCV (UNICALIB_USE_OPENCV); skipped.");
  return std::nullopt;
#endif
}

}  // namespace unicalib
