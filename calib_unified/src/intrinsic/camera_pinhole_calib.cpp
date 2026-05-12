/**
 * UniCalib Unified — 针孔相机内参标定实现
 * 使用 OpenCV calib3d — 棋盘格 / 圆点靶标
 */

#include "unicalib/intrinsic/camera_calib.h"
#include "unicalib/common/logger.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===================================================================
// TargetConfig — 目标角点3D坐标
// ===================================================================
std::vector<cv::Point3f> TargetConfig::object_points() const {
    std::vector<cv::Point3f> pts;
    switch (type) {
    case Type::CHESSBOARD:
    case Type::ASYMMETRIC_CIRCLES:
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                pts.emplace_back(c * square_size_m, r * square_size_m, 0.0f);
        break;
    case Type::CIRCLES_GRID:
        // 标准圆点网格
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                pts.emplace_back(c * square_size_m, r * square_size_m, 0.0f);
        break;
    }
    return pts;
}

// ===================================================================
// 角点检测
// ===================================================================
CalibFrame CameraIntrinsicCalibrator::detect_corners(const cv::Mat& image) {
    CalibFrame frame;

    cv::Mat gray;
    if (image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image.clone();

    cv::Size pattern_size(cfg_.target.cols, cfg_.target.rows);

    switch (cfg_.target.type) {
    case TargetConfig::Type::CHESSBOARD: {
        int flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                    cv::CALIB_CB_NORMALIZE_IMAGE |
                    cv::CALIB_CB_FAST_CHECK;
        frame.detection_ok = cv::findChessboardCorners(
            gray, pattern_size, frame.corners, flags);
        if (frame.detection_ok && cfg_.refine_corners) {
            int max_iter = std::max(1, cfg_.subpix_max_iter);
            double eps = cfg_.subpix_epsilon > 0 ? cfg_.subpix_epsilon : 0.01;
            cv::cornerSubPix(gray, frame.corners,
                cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, max_iter, eps));
        }
        break;
    }
    case TargetConfig::Type::CIRCLES_GRID: {
        frame.detection_ok = cv::findCirclesGrid(
            gray, pattern_size, frame.corners,
            cv::CALIB_CB_SYMMETRIC_GRID);
        break;
    }
    case TargetConfig::Type::ASYMMETRIC_CIRCLES: {
        frame.detection_ok = cv::findCirclesGrid(
            gray, pattern_size, frame.corners,
            cv::CALIB_CB_ASYMMETRIC_GRID);
        break;
    }
    }

    return frame;
}

// ===================================================================
// 从图像路径列表标定
// ===================================================================
std::optional<CameraIntrinsics> CameraIntrinsicCalibrator::calibrate(
    const std::vector<std::string>& image_paths) {

    frames_.clear();
    if (image_paths.empty()) {
        UNICALIB_ERROR("Camera intrinsic: no images provided");
        return std::nullopt;
    }

    UNICALIB_INFO("=== 相机内参标定开始 ===");
    UNICALIB_INFO("  图像数量: {}", image_paths.size());
    UNICALIB_INFO("  标定板: {}×{}, 格宽={:.3f}m",
                  cfg_.target.cols, cfg_.target.rows, cfg_.target.square_size_m);

    int total = static_cast<int>(image_paths.size());
    int width = 0, height = 0;

    // 加载并检测角点
    int skip = std::max(1, cfg_.frame_skip);
    for (int i = 0; i < total; i += skip) {
        if (progress_cb_) progress_cb_(i, total);

        cv::Mat img = cv::imread(image_paths[i]);
        if (img.empty()) {
            UNICALIB_WARN("Failed to load image: {}", image_paths[i]);
            continue;
        }
        if (width == 0) { width = img.cols; height = img.rows; }

        CalibFrame frame = detect_corners(img);
        frame.image_path = image_paths[i];
        if (frame.detection_ok) {
            frames_.push_back(std::move(frame));
            if (cfg_.verbose && frames_.size() % 10 == 0)
                UNICALIB_INFO("  检测到角点: {}/{}", frames_.size(), total / skip);
        }

        if (static_cast<int>(frames_.size()) >= cfg_.max_images) break;
    }

    UNICALIB_INFO("  有效图像帧: {}/{}", frames_.size(), total);

    if (static_cast<int>(frames_.size()) < cfg_.min_images) {
        if (cfg_.checkerboard_optional) {
            UNICALIB_WARN("Camera intrinsic: only {} valid frames (need ≥{} for 棋盘格精化，已采用无棋盘格/先验结果)",
                          frames_.size(), cfg_.min_images);
        } else {
            UNICALIB_ERROR("Camera intrinsic: only {} valid frames (need ≥{})",
                           frames_.size(), cfg_.min_images);
        }
        return std::nullopt;
    }

    // 执行标定
    if (cfg_.model == CameraIntrinsics::Model::FISHEYE) {
        return calibrate_fisheye(frames_, width, height);
    } else {
        return calibrate_pinhole(frames_, width, height);
    }
}

// ===================================================================
// 针孔标定
// ===================================================================
std::optional<CameraIntrinsics> CameraIntrinsicCalibrator::calibrate_pinhole(
    const std::vector<CalibFrame>& frames, int width, int height) {

    if (frames.empty()) return std::nullopt;

    std::vector<std::vector<cv::Point3f>> obj_pts_all;
    std::vector<std::vector<cv::Point2f>> img_pts_all;

    auto obj_pts = cfg_.target.object_points();
    for (const auto& f : frames) {
        obj_pts_all.push_back(obj_pts);
        img_pts_all.push_back(f.corners);
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    int flags = cv::CALIB_FIX_ASPECT_RATIO;
    if (cfg_.fix_k3) flags |= cv::CALIB_FIX_K3;
    if (cfg_.fix_k4) flags |= cv::CALIB_FIX_K4;
    if (cfg_.fix_k5) flags |= cv::CALIB_FIX_K5;
    if (cfg_.fix_k6) flags |= cv::CALIB_FIX_K6;
    if (cfg_.rational_model) flags |= cv::CALIB_RATIONAL_MODEL;

    int max_iter = std::max(1, cfg_.calibrate_max_iter);
    double term_eps = (cfg_.calibrate_epsilon > 0 && cfg_.calibrate_epsilon < 1.0) ? cfg_.calibrate_epsilon : 1e-7;
    double rms = cv::calibrateCamera(
        obj_pts_all, img_pts_all,
        cv::Size(width, height),
        K, dist_coeffs, rvecs, tvecs,
        flags,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, max_iter, term_eps));

    UNICALIB_CALC("相机内参 针孔标定 calibrateCamera 帧数={} rms={:.4f}px", static_cast<int>(frames.size()), rms);
    UNICALIB_INFO("针孔标定完成: RMS = {:.4f} px", rms);

    if (rms > cfg_.max_rms_px) {
        UNICALIB_WARN("RMS {:.4f} > threshold {:.4f} px — 标定质量低",
                      rms, cfg_.max_rms_px);
    }

    // 临时内参（用于每帧误差与离群剔除）
    CameraIntrinsics intrin_tmp;
    intrin_tmp.model = CameraIntrinsics::Model::PINHOLE;
    intrin_tmp.width = width;
    intrin_tmp.height = height;
    intrin_tmp.fx = K.at<double>(0,0);
    intrin_tmp.fy = K.at<double>(1,1);
    intrin_tmp.cx = K.at<double>(0,2);
    intrin_tmp.cy = K.at<double>(1,2);
    for (int i = 0; i < dist_coeffs.cols; ++i)
        intrin_tmp.dist_coeffs.push_back(dist_coeffs.at<double>(0, i));

    // 每帧重投影误差（用于离群剔除）
    std::vector<double> per_frame_rms(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        double err = compute_reproj_error(intrin_tmp, obj_pts_all[i], img_pts_all[i], rvecs[i], tvecs[i]);
        per_frame_rms[i] = err;
        const_cast<CalibFrame&>(frames[i]).reprojection_error = err;
    }

    // 离群帧剔除：剔除误差 > mean + sigma*std 的帧后重新标定
    if (cfg_.outlier_rejection_sigma > 0 && frames.size() > 2) {
        double mean_err = 0.0;
        for (double e : per_frame_rms) mean_err += e;
        mean_err /= per_frame_rms.size();
        double var = 0.0;
        for (double e : per_frame_rms) var += (e - mean_err) * (e - mean_err);
        double std_err = (per_frame_rms.size() > 1) ? std::sqrt(var / (per_frame_rms.size() - 1)) : 0.0;
        double thresh = mean_err + cfg_.outlier_rejection_sigma * std_err;
        std::vector<size_t> kept_indices;
        for (size_t i = 0; i < frames.size(); ++i)
            if (per_frame_rms[i] <= thresh) kept_indices.push_back(i);
        if (kept_indices.size() >= static_cast<size_t>(std::max(1, cfg_.min_images)) && kept_indices.size() < frames.size()) {
            UNICALIB_INFO("离群帧剔除: 剔除 {} 帧 (重投影误差 > {:.4f} px)，保留 {} 帧并重新标定",
                          frames.size() - kept_indices.size(), thresh, kept_indices.size());
            std::vector<std::vector<cv::Point3f>> obj_kept;
            std::vector<std::vector<cv::Point2f>> img_kept;
            for (size_t k : kept_indices) {
                obj_kept.push_back(obj_pts_all[k]);
                img_kept.push_back(img_pts_all[k]);
            }
            rvecs.clear(); tvecs.clear();
            K = cv::Mat::eye(3, 3, CV_64F);
            dist_coeffs.release();
            rms = cv::calibrateCamera(
                obj_kept, img_kept,
                cv::Size(width, height),
                K, dist_coeffs, rvecs, tvecs,
                flags,
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, max_iter, term_eps));
            UNICALIB_CALC("相机内参 针孔标定(剔除离群后) 保留帧数={} rms={:.4f}px", static_cast<int>(kept_indices.size()), rms);
            UNICALIB_INFO("针孔标定完成(剔除离群后): RMS = {:.4f} px", rms);
            CameraIntrinsics intrin_out;
            intrin_out.model  = CameraIntrinsics::Model::PINHOLE;
            intrin_out.width  = width;
            intrin_out.height = height;
            intrin_out.fx = K.at<double>(0,0);
            intrin_out.fy = K.at<double>(1,1);
            intrin_out.cx = K.at<double>(0,2);
            intrin_out.cy = K.at<double>(1,2);
            intrin_out.rms_reproj_error = rms;
            intrin_out.num_images_used  = static_cast<int>(kept_indices.size());
            for (int i = 0; i < dist_coeffs.cols; ++i)
                intrin_out.dist_coeffs.push_back(dist_coeffs.at<double>(0, i));
            for (size_t j = 0; j < kept_indices.size(); ++j)
                const_cast<CalibFrame&>(frames[kept_indices[j]]).reprojection_error =
                    compute_reproj_error(intrin_out, obj_kept[j], img_kept[j], rvecs[j], tvecs[j]);
            UNICALIB_INFO("  fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}",
                          intrin_out.fx, intrin_out.fy, intrin_out.cx, intrin_out.cy);
            UNICALIB_INFO("  畸变: k1={:.4f} k2={:.4f} p1={:.4f} p2={:.4f}",
                          intrin_out.dist_coeffs.size() > 0 ? intrin_out.dist_coeffs[0] : 0,
                          intrin_out.dist_coeffs.size() > 1 ? intrin_out.dist_coeffs[1] : 0,
                          intrin_out.dist_coeffs.size() > 2 ? intrin_out.dist_coeffs[2] : 0,
                          intrin_out.dist_coeffs.size() > 3 ? intrin_out.dist_coeffs[3] : 0);
            return intrin_out;
        }
    }

    CameraIntrinsics intrin;
    intrin.model  = CameraIntrinsics::Model::PINHOLE;
    intrin.width  = width;
    intrin.height = height;
    intrin.fx = K.at<double>(0,0);
    intrin.fy = K.at<double>(1,1);
    intrin.cx = K.at<double>(0,2);
    intrin.cy = K.at<double>(1,2);
    intrin.rms_reproj_error = rms;
    intrin.num_images_used  = static_cast<int>(frames.size());

    // 畸变系数
    for (int i = 0; i < dist_coeffs.cols; ++i) {
        intrin.dist_coeffs.push_back(dist_coeffs.at<double>(0, i));
    }

    // 计算每帧重投影误差
    for (size_t i = 0; i < frames.size(); ++i) {
        double err = compute_reproj_error(intrin, obj_pts_all[i], img_pts_all[i],
                                          rvecs[i], tvecs[i]);
        const_cast<CalibFrame&>(frames[i]).reprojection_error = err;
    }

    UNICALIB_INFO("  fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}",
                  intrin.fx, intrin.fy, intrin.cx, intrin.cy);
    UNICALIB_INFO("  畸变: k1={:.4f} k2={:.4f} p1={:.4f} p2={:.4f}",
                  intrin.dist_coeffs.size() > 0 ? intrin.dist_coeffs[0] : 0,
                  intrin.dist_coeffs.size() > 1 ? intrin.dist_coeffs[1] : 0,
                  intrin.dist_coeffs.size() > 2 ? intrin.dist_coeffs[2] : 0,
                  intrin.dist_coeffs.size() > 3 ? intrin.dist_coeffs[3] : 0);

    return intrin;
}

// ===================================================================
// 鱼眼标定
// ===================================================================
std::optional<CameraIntrinsics> CameraIntrinsicCalibrator::calibrate_fisheye(
    const std::vector<CalibFrame>& frames, int width, int height) {

    if (frames.empty()) return std::nullopt;

    std::vector<std::vector<cv::Point3f>> obj_pts_all;
    std::vector<std::vector<cv::Point2f>> img_pts_all;

    auto obj_pts_template = cfg_.target.object_points();
    for (const auto& f : frames) {
        // OpenCV fisheye::calibrate 要求每个角点有单独的 vector
        std::vector<std::vector<cv::Point3f>> obj_pts_single = {obj_pts_template};
        std::vector<std::vector<cv::Point2f>> img_pts_single = {f.corners};
        obj_pts_all.push_back(obj_pts_template);
        img_pts_all.push_back(f.corners);
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                cv::fisheye::CALIB_CHECK_COND |
                cv::fisheye::CALIB_FIX_SKEW;
    if (cfg_.fix_skew) flags |= cv::fisheye::CALIB_FIX_SKEW;

    double rms;
    try {
        rms = cv::fisheye::calibrate(
            obj_pts_all, img_pts_all,
            cv::Size(width, height),
            K, dist, rvecs, tvecs,
            flags,
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-7));
    } catch (const cv::Exception& e) {
        UNICALIB_ERROR("鱼眼标定异常: {}", e.what());
        // 尝试不含 CHECK_COND 重新标定
        flags &= ~cv::fisheye::CALIB_CHECK_COND;
        try {
            rms = cv::fisheye::calibrate(
                obj_pts_all, img_pts_all,
                cv::Size(width, height),
                K, dist, rvecs, tvecs, flags);
        } catch (...) {
            UNICALIB_ERROR("鱼眼标定失败");
            return std::nullopt;
        }
    }

    UNICALIB_CALC("相机内参 鱼眼标定 fisheye::calibrate 帧数={} rms={:.4f}px", static_cast<int>(frames.size()), rms);
    UNICALIB_INFO("鱼眼标定完成: RMS = {:.4f} px", rms);

    CameraIntrinsics intrin;
    intrin.model  = CameraIntrinsics::Model::FISHEYE;
    intrin.width  = width;
    intrin.height = height;
    intrin.fx = K.at<double>(0,0);
    intrin.fy = K.at<double>(1,1);
    intrin.cx = K.at<double>(0,2);
    intrin.cy = K.at<double>(1,2);
    intrin.rms_reproj_error = rms;
    intrin.num_images_used  = static_cast<int>(frames.size());

    for (int i = 0; i < 4; ++i) {
        intrin.dist_coeffs.push_back(dist.at<double>(i));
    }

    UNICALIB_INFO("  fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}",
                  intrin.fx, intrin.fy, intrin.cx, intrin.cy);
    UNICALIB_INFO("  畸变(kb4): k1={:.4f} k2={:.4f} k3={:.4f} k4={:.4f}",
                  intrin.dist_coeffs[0], intrin.dist_coeffs[1],
                  intrin.dist_coeffs[2], intrin.dist_coeffs[3]);

    return intrin;
}

// ===================================================================
// 计算重投影误差
// ===================================================================
double CameraIntrinsicCalibrator::compute_reproj_error(
    const CameraIntrinsics& intrin,
    const std::vector<cv::Point3f>& obj_pts,
    const std::vector<cv::Point2f>& img_pts,
    const cv::Mat& rvec, const cv::Mat& tvec) {

    cv::Mat K = (cv::Mat_<double>(3,3) <<
        intrin.fx, 0, intrin.cx,
        0, intrin.fy, intrin.cy,
        0, 0, 1);
    cv::Mat dist = cv::Mat(intrin.dist_coeffs).reshape(1, 1);

    std::vector<cv::Point2f> proj;
    if (intrin.model == CameraIntrinsics::Model::FISHEYE) {
        cv::fisheye::projectPoints(
            cv::Mat(obj_pts), rvec, tvec, K, dist, proj);
    } else {
        cv::projectPoints(obj_pts, rvec, tvec, K, dist, proj);
    }

    double rms = 0.0;
    for (size_t i = 0; i < img_pts.size(); ++i) {
        double dx = img_pts[i].x - proj[i].x;
        double dy = img_pts[i].y - proj[i].y;
        rms += dx*dx + dy*dy;
    }
    return std::sqrt(rms / img_pts.size());
}

// ===================================================================
// 从 cv::Mat 列表标定 (重载)
// ===================================================================
std::optional<CameraIntrinsics> CameraIntrinsicCalibrator::calibrate(
    const std::vector<cv::Mat>& images, int width, int height) {

    UNICALIB_INFO("=== 棋盘格角点标定 (cv::Mat) ===");
    UNICALIB_INFO("  图像数量: {}  分辨率: {}×{}  标定板: {}×{}",
                  images.size(), width, height, cfg_.target.cols, cfg_.target.rows);

    frames_.clear();
    int skip = std::max(1, cfg_.frame_skip);
    for (size_t i = 0; i < images.size(); i += skip) {
        if (progress_cb_)
            progress_cb_(static_cast<int>(i), static_cast<int>(images.size()));
        CalibFrame frame = detect_corners(images[i]);
        if (frame.detection_ok) {
            frames_.push_back(std::move(frame));
        }
        if (static_cast<int>(frames_.size()) >= cfg_.max_images) break;
    }

    UNICALIB_INFO("  棋盘格有效帧: {}/{} (需 ≥{} 才采用棋盘格结果)",
                  frames_.size(), images.size(), cfg_.min_images);

    if (static_cast<int>(frames_.size()) < cfg_.min_images) {
        const bool optional = cfg_.checkerboard_optional;
        if (optional) {
            UNICALIB_WARN("Camera intrinsic: only {} valid frames (need ≥{} for 棋盘格精化，将采用无棋盘格/先验结果)",
                          frames_.size(), cfg_.min_images);
        } else {
            UNICALIB_ERROR("Camera intrinsic: only {} valid frames (need ≥{})",
                           frames_.size(), cfg_.min_images);
        }
        if (frames_.size() == 0 && !images.empty()) {
            if (optional) {
                UNICALIB_INFO("  说明: 图像中未检测到标定板 (type={}, {}×{})，可能为 SLAM/外参数据；已启用 prefer_no_checkerboard，标定不失败",
                             cfg_.target.type == TargetConfig::Type::CHESSBOARD ? "chessboard" :
                             cfg_.target.type == TargetConfig::Type::CIRCLES_GRID ? "circles" : "asym_circles",
                             cfg_.target.cols, cfg_.target.rows);
            } else {
                UNICALIB_ERROR("  可能原因: 1) 图像中未包含标定板(棋盘格/圆点板)");
                UNICALIB_ERROR("  2) 配置的 target 与标定板不一致: 当前 type={}, cols={}, rows={}",
                              cfg_.target.type == TargetConfig::Type::CHESSBOARD ? "chessboard" :
                              cfg_.target.type == TargetConfig::Type::CIRCLES_GRID ? "circles" : "asym_circles",
                              cfg_.target.cols, cfg_.target.rows);
                UNICALIB_ERROR("  3) 若数据来自 bag，该 bag 可能为 SLAM/外参等场景数据，不含标定板");
                UNICALIB_ERROR("  建议: 配置 camera_intrinsic.prefer_no_checkerboard: true 可避免无棋盘格时失败 (DM-Calib/先验)；或使用含标定板数据");
            }
        }
        return std::nullopt;
    }

    if (cfg_.model == CameraIntrinsics::Model::FISHEYE) {
        return calibrate_fisheye(frames_, width, height);
    } else {
        return calibrate_pinhole(frames_, width, height);
    }
}

// ===================================================================
// 立体相机外参标定
// ===================================================================
std::optional<ExtrinsicSE3> StereoCameraCalibrator::calibrate(
    const std::vector<std::string>& images_cam0,
    const std::vector<std::string>& images_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    UNICALIB_INFO("=== 立体相机外参标定 ===");
    UNICALIB_INFO("  {} ↔ {}", cam0_id, cam1_id);

    if (images_cam0.size() != images_cam1.size()) {
        UNICALIB_ERROR("图像数量不匹配: {} vs {}", images_cam0.size(), images_cam1.size());
        return std::nullopt;
    }

    // 检测角点
    CameraIntrinsicCalibrator::Config det_cfg;
    det_cfg.target = cfg_.target;
    CameraIntrinsicCalibrator detector(det_cfg);

    std::vector<std::vector<cv::Point3f>> obj_pts_all;
    std::vector<std::vector<cv::Point2f>> img0_all, img1_all;
    auto obj_pts = cfg_.target.object_points();

    int width0 = 0, height0 = 0;
    for (size_t i = 0; i < images_cam0.size(); ++i) {
        cv::Mat img0 = cv::imread(images_cam0[i]);
        cv::Mat img1 = cv::imread(images_cam1[i]);
        if (img0.empty() || img1.empty()) continue;
        if (width0 == 0) { width0 = img0.cols; height0 = img0.rows; }

        auto f0 = detector.detect_corners(img0);
        auto f1 = detector.detect_corners(img1);
        if (f0.detection_ok && f1.detection_ok) {
            obj_pts_all.push_back(obj_pts);
            img0_all.push_back(f0.corners);
            img1_all.push_back(f1.corners);
        }
    }

    if (obj_pts_all.size() < 5) {
        UNICALIB_ERROR("立体标定: 有效帧太少 ({})", obj_pts_all.size());
        return std::nullopt;
    }

    // 构建内参矩阵
    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = cv::Mat(intrin0.dist_coeffs).reshape(1, 1);
    cv::Mat D1 = cv::Mat(intrin1.dist_coeffs).reshape(1, 1);

    cv::Mat R, T, E, F;
    int flags = cfg_.fix_intrinsics ?
        (cv::CALIB_FIX_INTRINSIC) : 0;

    double rms = cv::stereoCalibrate(
        obj_pts_all, img0_all, img1_all,
        K0, D0, K1, D1,
        cv::Size(width0, height0),
        R, T, E, F,
        flags,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-7));

    UNICALIB_CALC("立体内参 stereoCalibrate 帧数={} rms={:.4f}px converged={}",
                  static_cast<int>(obj_pts_all.size()), rms, (rms < cfg_.max_rms_px) ? "yes" : "no");
    UNICALIB_INFO("立体标定完成: RMS = {:.4f} px", rms);

    if (rms > cfg_.max_rms_px) {
        UNICALIB_WARN("RMS {:.4f} > threshold {:.4f} px", rms, cfg_.max_rms_px);
    }

    // 填充结果
    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;
    result.residual_rms     = rms;
    result.is_converged     = (rms < cfg_.max_rms_px);

    // R, T: cam1 在 cam0 坐标系下
    Eigen::Matrix3d R_eig;
    cv::cv2eigen(R, R_eig);
    Eigen::Vector3d T_eig;
    cv::cv2eigen(T, T_eig);
    result.SO3_TargetInRef  = Sophus::SO3d(R_eig);
    result.POS_TargetInRef  = T_eig;

    UNICALIB_INFO("  R (RPY deg): [{:.2f}, {:.2f}, {:.2f}]",
        result.SO3_TargetInRef.log()[0]*180/M_PI,
        result.SO3_TargetInRef.log()[1]*180/M_PI,
        result.SO3_TargetInRef.log()[2]*180/M_PI);
    UNICALIB_INFO("  T (m): [{:.4f}, {:.4f}, {:.4f}]",
        T_eig[0], T_eig[1], T_eig[2]);

    return result;
}

}  // namespace ns_unicalib
