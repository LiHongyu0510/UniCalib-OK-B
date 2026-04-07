/**
 * UniCalib C++ — Camera-Camera 特征匹配粗外参（OpenCV）
 */
#include "unicalib/feature_matching.hpp"
#include "unicalib/logger.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

#if defined(UNICALIB_USE_OPENCV) && UNICALIB_USE_OPENCV
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace unicalib {

namespace {

constexpr double kSyncThresholdSec = 0.1;

#if defined(UNICALIB_USE_OPENCV) && UNICALIB_USE_OPENCV
bool get_K_from_intrinsic(const IntrinsicResultHolder& h, cv::Mat& K) {
  if (h.camera_pinhole) {
    K = (cv::Mat_<double>(3, 3) <<
         h.camera_pinhole->K(0, 0), 0, h.camera_pinhole->K(0, 2),
         0, h.camera_pinhole->K(1, 1), h.camera_pinhole->K(1, 2),
         0, 0, 1);
    return true;
  }
  if (h.camera_fisheye) {
    auto it_fx = h.camera_fisheye->params.find("fx");
    auto it_fy = h.camera_fisheye->params.find("fy");
    auto it_cx = h.camera_fisheye->params.find("cx");
    auto it_cy = h.camera_fisheye->params.find("cy");
    if (it_fx == h.camera_fisheye->params.end() || it_fy == h.camera_fisheye->params.end()) return false;
    double fx = it_fx->second, fy = it_fy->second;
    double cx = it_cx != h.camera_fisheye->params.end() ? it_cx->second : 0;
    double cy = it_cy != h.camera_fisheye->params.end() ? it_cy->second : 0;
    K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    return true;
  }
  return false;
}
#endif

}  // namespace

std::optional<CalibResult> run_feature_matching_coarse(
  DataManager& data_mgr,
  const SensorConfig& sensor_a,
  const SensorConfig& sensor_b,
  const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
  int max_frames,
  int min_matches) {
#if defined(UNICALIB_USE_OPENCV) && UNICALIB_USE_OPENCV
  auto it_a = intrinsics.find(sensor_a.sensor_id);
  auto it_b = intrinsics.find(sensor_b.sensor_id);
  if (it_a == intrinsics.end() || it_b == intrinsics.end()) return std::nullopt;

  cv::Mat K_a, K_b;
  if (!get_K_from_intrinsic(it_a->second, K_a) || !get_K_from_intrinsic(it_b->second, K_b))
    return std::nullopt;

  std::vector<std::pair<double, std::string>> paths_a, paths_b;
  data_mgr.iter_image_paths(sensor_a.topic, [&](double ts, const std::string& path) {
    paths_a.emplace_back(ts, path);
  }, max_frames * 2, 1);
  data_mgr.iter_image_paths(sensor_b.topic, [&](double ts, const std::string& path) {
    paths_b.emplace_back(ts, path);
  }, max_frames * 2, 1);
  if (paths_a.empty() || paths_b.empty()) return std::nullopt;

  std::sort(paths_a.begin(), paths_a.end());
  std::sort(paths_b.begin(), paths_b.end());

  std::vector<cv::Point2f> all_pts_a, all_pts_b;
  int paired = 0;
  for (const auto& [ts_a, path_a] : paths_a) {
    if (paired >= max_frames) break;
    auto it = std::lower_bound(paths_b.begin(), paths_b.end(), ts_a - kSyncThresholdSec,
        [](const std::pair<double, std::string>& x, double t) { return x.first < t; });
    if (it == paths_b.end() || std::abs(it->first - ts_a) > kSyncThresholdSec) continue;

    cv::Mat img_a = cv::imread(path_a);
    cv::Mat img_b = cv::imread(it->second);
    if (img_a.empty() || img_b.empty()) continue;

    cv::Mat gray_a, gray_b;
    cv::cvtColor(img_a, gray_a, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_b, gray_b, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::SIFT> sift = cv::SIFT::create(200);
    std::vector<cv::KeyPoint> kp_a, kp_b;
    cv::Mat desc_a, desc_b;
    sift->detectAndCompute(gray_a, cv::noArray(), kp_a, desc_a);
    sift->detectAndCompute(gray_b, cv::noArray(), kp_b, desc_b);
    if (desc_a.empty() || desc_b.empty()) continue;

    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(desc_a, desc_b, matches);
    if (matches.size() < static_cast<size_t>(min_matches)) continue;

    std::vector<cv::Point2f> pts_a, pts_b;
    for (const auto& m : matches) {
      pts_a.push_back(kp_a[m.queryIdx].pt);
      pts_b.push_back(kp_b[m.trainIdx].pt);
    }

    cv::Mat E, R, t;
    cv::Mat mask;
    E = cv::findEssentialMat(pts_a, pts_b, K_a, cv::RANSAC, 0.999, 1.0, mask);
    if (E.empty()) continue;
    int inliers = cv::recoverPose(E, pts_a, pts_b, K_a, R, t, mask);
    if (inliers < min_matches) continue;

    for (int i = 0; i < mask.rows; ++i) {
      if (mask.at<unsigned char>(i)) {
        all_pts_a.push_back(pts_a[i]);
        all_pts_b.push_back(pts_b[i]);
      }
    }
    ++paired;
  }

  if (all_pts_a.size() < static_cast<size_t>(min_matches)) {
    LOG_WARNING("Feature matching: insufficient inliers");
    return std::nullopt;
  }

  cv::Mat E2, R, t, mask2;
  E2 = cv::findEssentialMat(all_pts_a, all_pts_b, K_a, cv::RANSAC, 0.999, 1.0, mask2);
  if (E2.empty()) return std::nullopt;
  cv::recoverPose(E2, all_pts_a, all_pts_b, K_a, R, t, mask2);

  CalibResult out;
  out.pair = {sensor_a.sensor_id, sensor_b.sensor_id};
  out.method_used = "feature_matching";
  out.confidence = 0.6;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out.rotation(i, j) = R.at<double>(i, j);
  out.translation(0) = t.at<double>(0);
  out.translation(1) = t.at<double>(1);
  out.translation(2) = t.at<double>(2);
  double n = out.translation.norm();
  if (n > 1e-6) out.translation /= n;
  LOG_INFO("Feature matching coarse OK: " + sensor_a.sensor_id + " -> " + sensor_b.sensor_id);
  return out;
#else
  (void)data_mgr;
  (void)sensor_a;
  (void)sensor_b;
  (void)intrinsics;
  (void)max_frames;
  (void)min_matches;
  LOG_WARNING("Feature matching requires OpenCV");
  return std::nullopt;
#endif
}

}  // namespace unicalib
