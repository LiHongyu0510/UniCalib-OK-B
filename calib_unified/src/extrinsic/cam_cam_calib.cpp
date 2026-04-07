/**
 * UniCalib Unified — Camera-Camera 外参标定实现
 * 方法:
 *   A: 棋盘格立体标定 (OpenCV stereoCalibrate)
 *   B: 本质矩阵法 (ORB + findEssentialMat)
 *   C: Bundle Adjustment (Ceres 两视图)
 */

#include "unicalib/extrinsic/cam_cam_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/math_safety.h"  // 修复：添加数学安全工具库
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>

namespace ns_unicalib {

// 返回可用于 OpenCV 的畸变系数 Mat；空时用 1x5 零矩阵避免未定义行为
static cv::Mat safe_dist_coeffs(const std::vector<double>& d) {
    if (!d.empty()) return cv::Mat(d).reshape(1, 1);
    return cv::Mat::zeros(1, 5, CV_64F);
}

// 尺度约束代价函数（需在命名空间/文件作用域，Ceres 局部类不能有 template 成员）
struct ScaleConstraintCost {
    double target_scale, weight;
    ScaleConstraintCost(double s, double w) : target_scale(s), weight(w) {}
    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        T tx = pose[3], ty = pose[4], tz = pose[5];
        T translation_norm = ceres::sqrt(tx*tx + ty*ty + tz*tz);
        residual[0] = T(weight) * (translation_norm - T(target_scale));
        return true;
    }
    static ceres::CostFunction* Create(double target_scale, double weight) {
        return new ceres::AutoDiffCostFunction<ScaleConstraintCost, 1, 6>(
            new ScaleConstraintCost(target_scale, weight));
    }
};

// ===================================================================
// 特征点提取与匹配
// ===================================================================
std::vector<FeatureMatch> CamCamCalibrator::match_features(
    const cv::Mat& img0,
    const cv::Mat& img1,
    const CameraIntrinsics& /*intrin0*/,
    const CameraIntrinsics& /*intrin1*/) {

    UNICALIB_DEBUG("[Cam-Cam] 步骤: 特征提取与匹配");
    std::vector<FeatureMatch> matches;

    cv::Mat gray0, gray1;
    if (img0.channels() == 3) cv::cvtColor(img0, gray0, cv::COLOR_BGR2GRAY);
    else gray0 = img0.clone();
    if (img1.channels() == 3) cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    else gray1 = img1.clone();

    auto orb = cv::ORB::create(cfg_.num_features, 1.2f, 8, 31, 0, 2,
                                cv::ORB::HARRIS_SCORE, 31, 20);
    std::vector<cv::KeyPoint> kp0, kp1;
    cv::Mat desc0, desc1;
    orb->detectAndCompute(gray0, cv::noArray(), kp0, desc0);
    orb->detectAndCompute(gray1, cv::noArray(), kp1, desc1);
    if (desc0.empty() || desc1.empty()) return matches;

    cv::BFMatcher bf(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> raw;
    bf.knnMatch(desc0, desc1, raw, 2);

    for (const auto& m : raw) {
        if (m.size() < 2) continue;
        if (m[0].distance < cfg_.match_ratio * m[1].distance) {
            FeatureMatch fm;
            fm.pt0 = kp0[m[0].queryIdx].pt;
            fm.pt1 = kp1[m[0].trainIdx].pt;
            fm.pt_cam0 = Eigen::Vector2d(fm.pt0.x, fm.pt0.y);
            fm.pt_cam1 = Eigen::Vector2d(fm.pt1.x, fm.pt1.y);
            fm.score = 1.0 - m[0].distance / 256.0;
            matches.push_back(fm);
        }
    }
    return matches;
}

// ===================================================================
// 从本质矩阵恢复 R, t
// ===================================================================
bool CamCamCalibrator::recover_pose_from_E(
    const std::vector<FeatureMatch>& matches,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    Sophus::SO3d& rot, Eigen::Vector3d& trans) {

    if (matches.size() < 8) return false;

    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = safe_dist_coeffs(intrin0.dist_coeffs);
    cv::Mat D1 = safe_dist_coeffs(intrin1.dist_coeffs);

    std::vector<cv::Point2f> raw0, raw1;
    for (const auto& m : matches) {
        raw0.push_back(m.pt0);
        raw1.push_back(m.pt1);
    }
    std::vector<cv::Point2f> pts0_ud, pts1_ud;
    cv::undistortPoints(raw0, pts0_ud, K0, D0);
    cv::undistortPoints(raw1, pts1_ud, K1, D1);

    cv::Mat inliers;
    double confidence = std::max(0.0, std::min(1.0, cfg_.ransac_confidence));
    double threshold  = std::max(1e-6, cfg_.ransac_threshold_essential);
    cv::Mat E = cv::findEssentialMat(pts0_ud, pts1_ud,
        cv::Mat::eye(3,3,CV_64F), cv::RANSAC, confidence, threshold, inliers);
    
    // 修复 3.1: 检查本质矩阵是否有效
    if (E.empty()) {
        UNICALIB_ERROR("[RecoverPoseE] 本质矩阵为空");
        return false;
    }
    
    // 修复 3.2: 检查内点数量是否充足
    if (inliers.rows < 8) {
        UNICALIB_ERROR("[RecoverPoseE] 内点数不足: {} (需要≥8)", inliers.rows);
        return false;
    }
    
    // 修复 3.3: 检查内点占比是否合理
    double inlier_ratio = static_cast<double>(inliers.rows) / matches.size();
    UNICALIB_CALC("本质矩阵 recover_pose 内点数={} 总匹配={} 内点比={:.3f}",
                  inliers.rows, static_cast<int>(matches.size()), inlier_ratio);
    if (inlier_ratio < 0.3) {  // 至少 30% 的匹配是内点
        UNICALIB_WARN("[RecoverPoseE] 内点占比过低: {:.2f}", inlier_ratio);
        // 不直接返回 false，但继续尝试恢复
    }

    cv::Mat R_cv, t_cv;
    cv::recoverPose(E, pts0_ud, pts1_ud,
                    cv::Mat::eye(3,3,CV_64F), R_cv, t_cv, inliers);
    
    // 修复 3.4: 检查旋转矩阵的有效性
    // 旋转矩阵应该满足: det(R) = 1, R^T * R = I
    double det = cv::determinant(R_cv);
    if (std::abs(det - 1.0) > 0.1) {
        UNICALIB_WARN("[RecoverPoseE] 旋转矩阵行列式异常: det={:.3f}", det);
    }
    
    // 检查旋转矩阵是否接近奇异
    cv::Mat RTR = R_cv.t() * R_cv;
    cv::Mat identity = cv::Mat::eye(3,3,CV_64F);
    double ortho_error = cv::norm(RTR, identity, cv::NORM_L2);
    if (ortho_error > 0.1) {
        UNICALIB_WARN("[RecoverPoseE] 旋转矩阵不正交: error={:.3f}", ortho_error);
    }

    Eigen::Matrix3d R_eig;
    Eigen::Vector3d t_eig;
    cv::cv2eigen(R_cv, R_eig);
    cv::cv2eigen(t_cv, t_eig);
    rot   = Sophus::SO3d(R_eig);
    trans = t_eig;
    UNICALIB_INFO("[Cam-Cam] 步骤: 本质矩阵恢复位姿完成 (内点 {})", inliers.rows);
    return true;
}

// ===================================================================
// 方法 A: 棋盘格立体标定
// ===================================================================
std::optional<ExtrinsicSE3> CamCamCalibrator::calibrate_stereo(
    const std::vector<std::string>& images_cam0,
    const std::vector<std::string>& images_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    UNICALIB_INFO("=== Camera-Camera 棋盘格立体标定 ===");
    UNICALIB_INFO("  {} ↔ {}", cam0_id, cam1_id);

    // 验证相机内参有效性
    auto validate_intrinsics = [](const CameraIntrinsics& intrin, const std::string& name) -> bool {
        if (intrin.fx <= 0 || intrin.fy <= 0) {
            UNICALIB_ERROR("[{}] 焦距无效: fx={}, fy={}", name, intrin.fx, intrin.fy);
            return false;
        }
        if (intrin.width <= 0 || intrin.height <= 0) {
            UNICALIB_ERROR("[{}] 图像尺寸无效: {}x{}", name, intrin.width, intrin.height);
            return false;
        }
        return true;
    };

    if (!validate_intrinsics(intrin0, cam0_id) || !validate_intrinsics(intrin1, cam1_id)) {
        return std::nullopt;
    }

    if (images_cam0.size() != images_cam1.size()) {
        UNICALIB_ERROR("图像数量不匹配: {} vs {}", images_cam0.size(), images_cam1.size());
        return std::nullopt;
    }

    CameraIntrinsicCalibrator::Config det_cfg;
    det_cfg.target = cfg_.target;
    CameraIntrinsicCalibrator detector(det_cfg);

    std::vector<std::vector<cv::Point3f>> obj_all;
    std::vector<std::vector<cv::Point2f>> img0_all, img1_all;
    auto obj = cfg_.target.object_points();

    int W = 0, H = 0;
    for (size_t i = 0; i < images_cam0.size(); ++i) {
        cv::Mat im0 = cv::imread(images_cam0[i]);
        cv::Mat im1 = cv::imread(images_cam1[i]);
        if (im0.empty() || im1.empty()) continue;
        if (W == 0) { W = im0.cols; H = im0.rows; }

        auto f0 = detector.detect_corners(im0);
        auto f1 = detector.detect_corners(im1);
        if (f0.detection_ok && f1.detection_ok) {
            obj_all.push_back(obj);
            img0_all.push_back(f0.corners);
            img1_all.push_back(f1.corners);
        }
    }

    if (obj_all.size() < 5) {
        UNICALIB_ERROR("有效帧对不足 (需≥5, 实有{})", obj_all.size());
        return std::nullopt;
    }

    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = safe_dist_coeffs(intrin0.dist_coeffs);
    cv::Mat D1 = safe_dist_coeffs(intrin1.dist_coeffs);

    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(
        obj_all, img0_all, img1_all, K0, D0, K1, D1,
        cv::Size(W, H), R, T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-7));

    UNICALIB_INFO("  RMS={:.4f}px 有效帧={}", rms, obj_all.size());

    Eigen::Matrix3d R_eig;
    Eigen::Vector3d t_eig;
    cv::cv2eigen(R, R_eig);
    cv::cv2eigen(T, t_eig);

    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;
    result.SO3_TargetInRef  = Sophus::SO3d(R_eig);
    result.POS_TargetInRef  = t_eig;
    result.residual_rms     = rms;
    result.is_converged     = (rms < cfg_.max_rms_px);
    return result;
}

// 方法 A 重载: 从内存帧
std::optional<ExtrinsicSE3> CamCamCalibrator::calibrate_stereo(
    const std::vector<std::pair<double, cv::Mat>>& frames_cam0,
    const std::vector<std::pair<double, cv::Mat>>& frames_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    UNICALIB_INFO("=== Camera-Camera 棋盘格立体标定 (内存帧) ===");
    UNICALIB_INFO("  {} ↔ {}", cam0_id, cam1_id);

    // 验证相机内参有效性
    auto validate_intrinsics = [](const CameraIntrinsics& intrin, const std::string& name) -> bool {
        if (intrin.fx <= 0 || intrin.fy <= 0) {
            UNICALIB_ERROR("[{}] 焦距无效: fx={}, fy={}", name, intrin.fx, intrin.fy);
            return false;
        }
        if (intrin.width <= 0 || intrin.height <= 0) {
            UNICALIB_ERROR("[{}] 图像尺寸无效: {}x{}", name, intrin.width, intrin.height);
            return false;
        }
        return true;
    };

    if (!validate_intrinsics(intrin0, cam0_id) || !validate_intrinsics(intrin1, cam1_id)) {
        return std::nullopt;
    }

    size_t N = std::min(frames_cam0.size(), frames_cam1.size());
    if (N == 0) {
        UNICALIB_ERROR("帧数为 0");
        return std::nullopt;
    }

    CameraIntrinsicCalibrator::Config det_cfg;
    det_cfg.target = cfg_.target;
    CameraIntrinsicCalibrator detector(det_cfg);

    std::vector<std::vector<cv::Point3f>> obj_all;
    std::vector<std::vector<cv::Point2f>> img0_all, img1_all;
    auto obj = cfg_.target.object_points();

    int W = 0, H = 0;
    for (size_t i = 0; i < N; ++i) {
        const cv::Mat& im0 = frames_cam0[i].second;
        const cv::Mat& im1 = frames_cam1[i].second;
        if (im0.empty() || im1.empty()) continue;
        if (W == 0) { W = im0.cols; H = im0.rows; }

        auto f0 = detector.detect_corners(im0);
        auto f1 = detector.detect_corners(im1);
        if (f0.detection_ok && f1.detection_ok) {
            obj_all.push_back(obj);
            img0_all.push_back(f0.corners);
            img1_all.push_back(f1.corners);
        }
    }

    if (obj_all.size() < 5) {
        UNICALIB_ERROR("有效帧对不足 (需≥5, 实有{})", obj_all.size());
        return std::nullopt;
    }

    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = safe_dist_coeffs(intrin0.dist_coeffs);
    cv::Mat D1 = safe_dist_coeffs(intrin1.dist_coeffs);

    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(
        obj_all, img0_all, img1_all, K0, D0, K1, D1,
        cv::Size(W, H), R, T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-7));

    UNICALIB_INFO("  RMS={:.4f}px 有效帧={}", rms, obj_all.size());

    Eigen::Matrix3d R_eig;
    Eigen::Vector3d t_eig;
    cv::cv2eigen(R, R_eig);
    cv::cv2eigen(T, t_eig);

    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;
    result.SO3_TargetInRef  = Sophus::SO3d(R_eig);
    result.POS_TargetInRef  = t_eig;
    result.residual_rms     = rms;
    result.is_converged     = (rms < cfg_.max_rms_px);
    UNICALIB_CALC("立体标定 stereoCalibrate rms={:.4f}px converged={}", rms, result.is_converged ? "yes" : "no");
    return result;
}

// ===================================================================
// 两阶段标定 (推荐使用)
// ===================================================================
CamCamCalibrator::TwoStageResult CamCamCalibrator::calibrate_two_stage(
    const std::vector<std::pair<double, cv::Mat>>& frames_cam0,
    const std::vector<std::pair<double, cv::Mat>>& frames_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    bool prefer_targetfree,
    const std::string& cam0_id,
    const std::string& cam1_id,
    std::optional<ExtrinsicSE3> init_extrin) {

    TwoStageResult result;
    result.manual_threshold_px = cfg_.max_rms_px;

    UNICALIB_INFO("=== Camera-Camera 两阶段标定 ===");
    UNICALIB_INFO("  相机0 帧数: {}", frames_cam0.size());
    UNICALIB_INFO("  相机1 帧数: {}", frames_cam1.size());
    UNICALIB_INFO("  优先无目标: {}", prefer_targetfree);
    const bool has_initial = init_extrin.has_value() && cfg_.use_initial_if_available;
    UNICALIB_INFO("  使用初值: {} (skip_coarse={})", has_initial, has_initial && cfg_.skip_coarse_when_initial_given);

    size_t N = std::min(frames_cam0.size(), frames_cam1.size());
    if (N < 3) {
        UNICALIB_ERROR("图像对不足 (需≥3)");
        return result;
    }

    Sophus::SE3d init_T = Sophus::SE3d();

    // ─── Stage 1: 粗标定 (本质矩阵)，有初值且 skip_coarse 时可省略 ───
    const bool run_coarse = !(has_initial && cfg_.skip_coarse_when_initial_given);
    if (run_coarse) {
        auto coarse_opt = calibrate_essential(
            frames_cam0, frames_cam1, intrin0, intrin1, cam0_id, cam1_id);
        if (coarse_opt.has_value()) {
            result.coarse = coarse_opt;
            result.coarse_method = "ESSENTIAL_MATRIX";
            result.coarse_rms = coarse_opt->residual_rms >= 0 ? coarse_opt->residual_rms : -1.0;
            init_T = result.coarse->SE3_TargetInRef();
        }
    }
    if (has_initial)
        init_T = init_extrin->SE3_TargetInRef();
    else if (result.coarse.has_value())
        init_T = result.coarse->SE3_TargetInRef();

    // ─── Stage 2: 精标定 ───
    std::optional<ExtrinsicSE3> fine_result;
    if (prefer_targetfree) {
        std::vector<FeatureMatch> all_matches;
        for (size_t fi = 0; fi < N; ++fi) {
            if (frames_cam0[fi].second.empty() || frames_cam1[fi].second.empty())
                continue;
            auto fm = match_features(frames_cam0[fi].second, frames_cam1[fi].second,
                                     intrin0, intrin1);
            for (auto& m : fm) all_matches.push_back(m);
        }
        UNICALIB_INFO("[Fine] 匹配点数: {}, 执行 Bundle Adjustment", all_matches.size());
        if (all_matches.size() >= 10) {
            fine_result = bundle_adjustment_two_views(
                all_matches, intrin0, intrin1, init_T, cam0_id, cam1_id);
            result.fine_method = "BUNDLE_ADJUSTMENT";
        }
    } else {
        UNICALIB_INFO("[Fine] 执行棋盘格立体标定");
        fine_result = calibrate_stereo(
            frames_cam0, frames_cam1, intrin0, intrin1, cam0_id, cam1_id);
        result.fine_method = "CHESSBOARD_STEREO";
    }

    if (fine_result.has_value()) {
        result.fine = fine_result;
        result.fine_rms = fine_result->residual_rms;
        result.needs_manual = (result.fine_rms > 0 && result.fine_rms > result.manual_threshold_px);
        UNICALIB_INFO("[Fine] 标定完成: RMS={:.3f}px converged={}",
                      result.fine_rms, fine_result->is_converged ? "yes" : "no");
        if (result.needs_manual) {
            UNICALIB_WARN("[Fine] RMS={:.3f}px > 阈值={:.3f}px，建议手动校准",
                          result.fine_rms, result.manual_threshold_px);
        }
    } else {
        UNICALIB_WARN("[Fine] 精标定未得到结果");
    }

    return result;
}

// ===================================================================
// 方法 B: 本质矩阵法
// ===================================================================
std::optional<ExtrinsicSE3> CamCamCalibrator::calibrate_essential(
    const std::vector<std::pair<double, cv::Mat>>& frames_cam0,
    const std::vector<std::pair<double, cv::Mat>>& frames_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    UNICALIB_INFO("=== Camera-Camera 本质矩阵法 ===");
    size_t N = std::min(frames_cam0.size(), frames_cam1.size());
    if (N < 3) { UNICALIB_ERROR("图像对不足 (需≥3)"); return std::nullopt; }

    std::vector<FeatureMatch> all_matches;
    for (size_t fi = 0; fi < N; ++fi) {
        auto fm = match_features(frames_cam0[fi].second, frames_cam1[fi].second,
                                 intrin0, intrin1);
        for (auto& m : fm) all_matches.push_back(m);
    }
    UNICALIB_INFO("  总匹配点: {}", all_matches.size());
    if (all_matches.size() < 20) {
        UNICALIB_ERROR("匹配点不足");
        return std::nullopt;
    }

    Sophus::SO3d rot;
    Eigen::Vector3d trans;
    if (!recover_pose_from_E(all_matches, intrin0, intrin1, rot, trans)) {
        UNICALIB_ERROR("本质矩阵分解失败");
        return std::nullopt;
    }

    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;
    result.SO3_TargetInRef  = rot;
    result.POS_TargetInRef  = trans;
    result.is_converged     = true;
    result.residual_rms     = 0.0;
    UNICALIB_CALC("本质矩阵法完成 平移 T=[{:.4f},{:.4f},{:.4f}] 匹配数={}",
                  trans[0], trans[1], trans[2], static_cast<int>(all_matches.size()));
    UNICALIB_INFO("本质矩阵法完成 T=[{:.4f},{:.4f},{:.4f}]",
                  trans[0], trans[1], trans[2]);
    return result;
}

// ===================================================================
// Bundle Adjustment 代价函数
// ===================================================================
namespace {
struct ReprojectCost {
    double obs_u, obs_v, fx, fy, cx, cy;
    double px, py, pz;
    ReprojectCost(double u, double v,
                  double _fx, double _fy, double _cx, double _cy,
                  double x, double y, double z)
        : obs_u(u), obs_v(v), fx(_fx), fy(_fy), cx(_cx), cy(_cy),
          px(x), py(y), pz(z) {}

    template <typename T>
    bool operator()(const T* const pose, T* res) const {
        T pt[3] = {T(px), T(py), T(pz)}, pt_c[3];
        ceres::AngleAxisRotatePoint(pose, pt, pt_c);
        pt_c[0] += pose[3]; pt_c[1] += pose[4]; pt_c[2] += pose[5];
        T zi = T(1.0) / pt_c[2];
        res[0] = T(fx) * pt_c[0] * zi + T(cx) - T(obs_u);
        res[1] = T(fy) * pt_c[1] * zi + T(cy) - T(obs_v);
        return true;
    }
    static ceres::CostFunction* Create(
        double u, double v, double fx, double fy, double cx, double cy,
        double x, double y, double z) {
        return new ceres::AutoDiffCostFunction<ReprojectCost, 2, 6>(
            new ReprojectCost(u, v, fx, fy, cx, cy, x, y, z));
    }
};
}  // anonymous namespace

// ===================================================================
// 方法 C: Bundle Adjustment (两视图)
// ===================================================================
ExtrinsicSE3 CamCamCalibrator::bundle_adjustment_two_views(
    const std::vector<FeatureMatch>& matches,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const Sophus::SE3d& init_T,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;

    if (matches.size() < 10) {
        result.set_SE3(init_T);
        result.residual_rms = -1.0;
        result.is_converged = false;
        return result;
    }
    // 零基线初值导致三角化退化，无法得到有效 3D 点
    if (init_T.translation().norm() < 1e-6) {
        UNICALIB_WARN("[Cam-Cam] BA 初值平移接近零（零基线），三角化退化，跳过优化");
        result.set_SE3(init_T);
        result.residual_rms = -1.0;
        result.is_converged = false;
        return result;
    }

    UNICALIB_INFO("[Cam-Cam] 步骤: Bundle Adjustment 开始 (匹配点 {})", matches.size());
    // 三角化
    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = safe_dist_coeffs(intrin0.dist_coeffs);
    cv::Mat D1 = safe_dist_coeffs(intrin1.dist_coeffs);

    std::vector<cv::Point2f> raw0, raw1;
    for (const auto& m : matches) { raw0.push_back(m.pt0); raw1.push_back(m.pt1); }

    // 三角化投影矩阵
    cv::Mat R_cv, t_cv;
    cv::eigen2cv(Eigen::Matrix3d(init_T.rotationMatrix()), R_cv);
    cv::eigen2cv(Eigen::Vector3d(init_T.translation()), t_cv);
    cv::Mat P0 = cv::Mat::zeros(3,4,CV_64F);
    K0.copyTo(P0(cv::Rect(0,0,3,3)));
    cv::Mat RT = cv::Mat::zeros(3,4,CV_64F);
    R_cv.copyTo(RT.colRange(0,3)); t_cv.copyTo(RT.col(3));
    cv::Mat P1 = K1 * RT;

    cv::Mat pts4d;
    cv::triangulatePoints(P0, P1, raw0, raw1, pts4d);

    // 初始化 pose 参数 [rx,ry,rz,tx,ty,tz]
    Eigen::Vector3d rv = init_T.so3().log();
    Eigen::Vector3d tv = init_T.translation();
    double pose[6] = {rv[0], rv[1], rv[2], tv[0], tv[1], tv[2]};

    ceres::Problem problem;
    int n_valid = 0;
    int n_invalid_depth = 0;
    int n_invalid_proj = 0;
    
    const bool pts4d_is_f64 = (pts4d.type() == CV_64FC1);
    for (int i = 0; i < pts4d.cols; ++i) {
        double w = pts4d_is_f64 ? pts4d.at<double>(3, i) : static_cast<double>(pts4d.at<float>(3, i));
        if (std::abs(w) < 1e-6) {
            ++n_invalid_depth;
            continue;
        }
        double x = (pts4d_is_f64 ? pts4d.at<double>(0, i) : static_cast<double>(pts4d.at<float>(0, i))) / w;
        double y = (pts4d_is_f64 ? pts4d.at<double>(1, i) : static_cast<double>(pts4d.at<float>(1, i))) / w;
        double z = (pts4d_is_f64 ? pts4d.at<double>(2, i) : static_cast<double>(pts4d.at<float>(2, i))) / w;
        
        // 深度范围检查
        if (z < 0.1 || z > 200.0) {
            ++n_invalid_depth;
            continue;
        }
        
        // 检查 3D 点是否在相机前方 (视锥体约束)
        // 假设相机朝向 +Z 方向，需要 z > 0
        if (z <= 0) {
            ++n_invalid_depth;
            continue;
        }
        
        // 检查重投影是否在图像范围内
        double proj_u = intrin1.fx * x / z + intrin1.cx;
        double proj_v = intrin1.fy * y / z + intrin1.cy;
        
        // 允许一定边界外 (5% 边距)
        double margin = std::max(10.0, 0.05 * std::min(intrin1.width, intrin1.height));
        if (proj_u < -margin || proj_u > intrin1.width + margin) {
            ++n_invalid_proj;
            continue;
        }
        if (proj_v < -margin || proj_v > intrin1.height + margin) {
            ++n_invalid_proj;
            continue;
        }
        
        // 检查 x, y 合理性 (避免过大的横向偏移)
        if (std::abs(x) > 100.0 || std::abs(y) > 100.0) {
            ++n_invalid_proj;
            continue;
        }
        
        problem.AddResidualBlock(
            ReprojectCost::Create(raw1[i].x, raw1[i].y,
                                  intrin1.fx, intrin1.fy, intrin1.cx, intrin1.cy,
                                  x, y, z),
            new ceres::HuberLoss(1.0), pose);
        ++n_valid;
    }
    
    if (n_valid > 0) {
        UNICALIB_DEBUG("  BA: 有效点={}, 无效深度={}, 无效投影={}",
                      n_valid, n_invalid_depth, n_invalid_proj);
    }

    ceres::Solver::Summary summary;
    if (n_valid >= 10) {
        ceres::Solver::Options opt;
        opt.max_num_iterations = cfg_.ba_max_iter;
        opt.linear_solver_type = ceres::DENSE_SCHUR;
        opt.minimizer_progress_to_stdout = false;

        // 添加尺度约束 (参考: "Solving for Relative Pose with Constraints", CVPR 2014)
        // 通过已知距离消除尺度不确定性
        // 约束: ||t|| = known_baseline_scale (已知的基线长度)
        if (cfg_.enable_scale_constraint && cfg_.known_baseline_scale > 0.001) {
            problem.AddResidualBlock(
                ScaleConstraintCost::Create(cfg_.known_baseline_scale, cfg_.scale_constraint_weight),
                nullptr, pose);

            UNICALIB_INFO("  BA: 启用尺度约束 scale={:.4f}m weight={:.1f}",
                         cfg_.known_baseline_scale, cfg_.scale_constraint_weight);
        }

        try {
            ceres::Solve(opt, &problem, &summary);
            UNICALIB_INFO("  BA: n_valid={} {}", n_valid, summary.BriefReport());
            UNICALIB_CALC("Cam-Cam BA Ceres n_valid={} initial_cost={:.6f} final_cost={:.6f} iter={} converged={}",
                          n_valid, summary.initial_cost, summary.final_cost,
                          static_cast<int>(summary.iterations.size()),
                          summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");
        } catch (const std::exception& e) {
            UNICALIB_ERROR("[Cam-Cam] Ceres BA 求解异常: {}", e.what());
            result.set_SE3(init_T);
            result.residual_rms = -1.0;
            result.is_converged = false;
            return result;
        }
    } else {
        result.residual_rms = -1.0;
    }

    Eigen::Vector3d rv2(pose[0], pose[1], pose[2]);
    if (rv2.norm() < 1e-10)
        result.SO3_TargetInRef = Sophus::SO3d();
    else
        result.SO3_TargetInRef = Sophus::SO3d(Eigen::AngleAxisd(rv2.norm(), rv2.normalized()).toRotationMatrix());
    result.POS_TargetInRef  = Eigen::Vector3d(pose[3], pose[4], pose[5]);
    result.is_converged     = (n_valid >= 10) && (summary.termination_type == ceres::CONVERGENCE);
    // 近似重投影 RMS（Ceres cost = 0.5*sum rho(r^2)，Huber 下为近似）
    if (n_valid >= 10) {
        if (summary.IsSolutionUsable() && summary.final_cost >= 0.0) {
            int num_residuals = n_valid * 2;
            result.residual_rms = std::sqrt(std::max(0.0, 2.0 * summary.final_cost / num_residuals));
        } else {
            result.residual_rms = -1.0;
        }
    }
    UNICALIB_CALC("Cam-Cam BA 完成 有效点={} converged={} residual_rms={:.4f}px",
                  n_valid, result.is_converged ? "yes" : "no", result.residual_rms);
    UNICALIB_INFO("[Cam-Cam] 步骤: Bundle Adjustment 完成 (有效点 {} converged={} rms={:.3f}px)",
                  n_valid, result.is_converged ? "yes" : "no", result.residual_rms);
    return result;
}

// ===================================================================
// 方法 C: 多相机 Bundle Adjustment
// ===================================================================
std::vector<ExtrinsicSE3> CamCamCalibrator::calibrate_bundle_adjustment(
    const std::vector<std::vector<std::pair<double, cv::Mat>>>& frames_per_cam,
    const std::vector<CameraIntrinsics>& intrinsics,
    const std::vector<std::string>& cam_ids) {

    UNICALIB_INFO("=== Camera-Camera Bundle Adjustment ({} 相机) ===",
                  cam_ids.size());

    std::vector<ExtrinsicSE3> results;
    if (cam_ids.size() < 2) return results;

    // 多相机且启用时，优先完整多视图 BA（全局联合优化）
    if (cfg_.use_full_multiview_ba && cam_ids.size() >= 2) {
        auto full = calibrate_bundle_adjustment_full_multiview(
            frames_per_cam, intrinsics, cam_ids);
        if (!full.extrinsics.empty()) {
            UNICALIB_INFO("  [多视图 BA] 使用全局优化结果 (点 {} 观测 {} converged={})",
                          full.num_points, full.num_observations, full.converged ? "yes" : "no");
            return full.extrinsics;
        }
        UNICALIB_WARN("  [多视图 BA] 未得到结果，退回两两标定");
    }

    return calibrate_bundle_adjustment_pairwise_only(
        frames_per_cam, intrinsics, cam_ids);
}

std::vector<ExtrinsicSE3> CamCamCalibrator::calibrate_bundle_adjustment_pairwise_only(
    const std::vector<std::vector<std::pair<double, cv::Mat>>>& frames_per_cam,
    const std::vector<CameraIntrinsics>& intrinsics,
    const std::vector<std::string>& cam_ids) {

    std::vector<ExtrinsicSE3> results;
    if (cam_ids.size() < 2) return results;

    for (size_t i = 1; i < cam_ids.size(); ++i) {
        const auto& f0 = frames_per_cam[0];
        const auto& f1 = frames_per_cam[i];
        size_t N = std::min(f0.size(), f1.size());

        std::vector<FeatureMatch> all_matches;
        for (size_t fi = 0; fi < N; ++fi) {
            if (f0[fi].second.empty() || f1[fi].second.empty()) continue;
            auto fm = match_features(f0[fi].second, f1[fi].second,
                                     intrinsics[0], intrinsics[i]);
            for (auto& m : fm) all_matches.push_back(m);
        }
        UNICALIB_INFO("  cam0 <-> cam{}: {} 匹配", i, all_matches.size());

        Sophus::SO3d rot;
        Eigen::Vector3d trans;
        Sophus::SE3d init_T;
        if (recover_pose_from_E(all_matches, intrinsics[0], intrinsics[i], rot, trans)) {
            init_T = Sophus::SE3d(rot, trans);
        }

        auto ext = bundle_adjustment_two_views(
            all_matches, intrinsics[0], intrinsics[i],
            init_T, cam_ids[0], cam_ids[i]);
        results.push_back(ext);
    }
    return results;
}

// ===================================================================
// 完整多视图 BA：跨相机轨迹 + 全局 Ceres 优化
// ===================================================================
namespace {
// 单次观测：(相机 id, 像素 u, v)
struct Obs {
    int cam_id;
    double u, v;
};
// 轨迹：同一 3D 点在多相机中的观测
using Track = std::vector<Obs>;

// 多视图重投影代价：X 在 cam0 系，T_cam0_cami 为 cam_i 在 cam0 下的位姿，则 p_cami = T_cam0_cami^{-1} * X = R^T*(X-t)
struct MultiViewReprojectCost {
    double obs_u, obs_v;
    double fx, fy, cx, cy;
    MultiViewReprojectCost(double u, double v, double _fx, double _fy, double _cx, double _cy)
        : obs_u(u), obs_v(v), fx(_fx), fy(_fy), cx(_cx), cy(_cy) {}
    template <typename T>
    bool operator()(const T* const T_cam0_cami, const T* const X, T* res) const {
        const T* aa = T_cam0_cami;
        const T* t = T_cam0_cami + 3;
        T d[3] = { X[0] - t[0], X[1] - t[1], X[2] - t[2] };
        T inv_aa[3] = { -aa[0], -aa[1], -aa[2] };
        T p_cami[3];
        ceres::AngleAxisRotatePoint(inv_aa, d, p_cami);
        T zi = T(1.0) / p_cami[2];
        res[0] = T(fx) * p_cami[0] * zi + T(cx) - T(obs_u);
        res[1] = T(fy) * p_cami[1] * zi + T(cy) - T(obs_v);
        return true;
    }
    static ceres::CostFunction* Create(double u, double v,
                                      double fx, double fy, double cx, double cy) {
        return new ceres::AutoDiffCostFunction<MultiViewReprojectCost, 2, 6, 3>(
            new MultiViewReprojectCost(u, v, fx, fy, cx, cy));
    }
};

// cam0 系下 3D 点重投影代价（单参数块 3，供 Ceres AutoDiff 使用；须在文件作用域以支持 template 成员）
struct ReprojectCam0Cost {
    double obs_u, obs_v, fx, fy, cx, cy;
    ReprojectCam0Cost(double u, double v, double _fx, double _fy, double _cx, double _cy)
        : obs_u(u), obs_v(v), fx(_fx), fy(_fy), cx(_cx), cy(_cy) {}
    template <typename T>
    bool operator()(const T* const X, T* res) const {
        T zi = T(1.0) / X[2];
        res[0] = T(fx) * X[0] * zi + T(cx) - T(obs_u);
        res[1] = T(fy) * X[1] * zi + T(cy) - T(obs_v);
        return true;
    }
};
}  // namespace

CamCamCalibrator::MultiViewBAResult CamCamCalibrator::calibrate_bundle_adjustment_full_multiview(
    const std::vector<std::vector<std::pair<double, cv::Mat>>>& frames_per_cam,
    const std::vector<CameraIntrinsics>& intrinsics,
    const std::vector<std::string>& cam_ids) {

    MultiViewBAResult out;
    if (cam_ids.size() < 2) return out;
    if (intrinsics.size() < cam_ids.size()) {
        UNICALIB_WARN("[FullMultiViewBA] intrinsics.size() ({}) < cam_ids.size() ({})",
                      intrinsics.size(), cam_ids.size());
        return out;
    }

    UNICALIB_INFO("=== Camera-Camera 完整多视图 BA ({} 相机) ===", cam_ids.size());

    const size_t num_cams = cam_ids.size();
    const size_t num_frames = frames_per_cam.empty() ? 0 : frames_per_cam[0].size();

    if (num_frames == 0) {
        UNICALIB_WARN("[FullMultiViewBA] 无帧数据");
        return out;
    }
    for (size_t c = 0; c < num_cams; ++c) {
        if (frames_per_cam[c].size() < num_frames) {
            UNICALIB_WARN("[FullMultiViewBA] cam{} 帧数不足", c);
            return out;
        }
    }

    // 1) 提取每相机关键点与描述子（使用第一帧或中间帧）
    const size_t frame_idx = num_frames / 2;
    std::vector<std::vector<cv::KeyPoint>> kps(num_cams);
    std::vector<cv::Mat> descs(num_cams);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(cfg_.num_features, 1.2f, 8, 31, 0, 2,
                                           cv::ORB::HARRIS_SCORE, 31, 20);
    for (size_t c = 0; c < num_cams; ++c) {
        cv::Mat gray;
        if (frames_per_cam[c][frame_idx].second.channels() == 3)
            cv::cvtColor(frames_per_cam[c][frame_idx].second, gray, cv::COLOR_BGR2GRAY);
        else
            gray = frames_per_cam[c][frame_idx].second;
        orb->detectAndCompute(gray, cv::noArray(), kps[c], descs[c]);
    }

    // 2) 两两匹配并构建轨迹（cam0 与 cam_i 匹配，再按 cam0 的 idx 合并）
    std::vector<std::map<int, int>> cam0_to_cami(num_cams);
    for (size_t i = 1; i < num_cams; ++i) {
        if (descs[0].empty() || descs[i].empty()) continue;
        cv::BFMatcher bf(cv::NORM_HAMMING, false);
        std::vector<cv::DMatch> matches;
        bf.match(descs[0], descs[i], matches);
        for (const auto& m : matches) {
            if (m.distance > 80) continue;
            cam0_to_cami[i][m.queryIdx] = m.trainIdx;
        }
    }
    std::vector<Track> tracks;
    for (const auto& pair01 : cam0_to_cami[1]) {
        int idx0 = pair01.first;
        int idx1 = pair01.second;
        Track tr;
        tr.push_back({0, static_cast<double>(kps[0][idx0].pt.x), static_cast<double>(kps[0][idx0].pt.y)});
        tr.push_back({1, static_cast<double>(kps[1][idx1].pt.x), static_cast<double>(kps[1][idx1].pt.y)});
        for (size_t i = 2; i < num_cams; ++i) {
            auto it = cam0_to_cami[i].find(idx0);
            if (it != cam0_to_cami[i].end()) {
                int idxi = it->second;
                tr.push_back({static_cast<int>(i),
                    static_cast<double>(kps[i][idxi].pt.x), static_cast<double>(kps[i][idxi].pt.y)});
            }
        }
        if (tr.size() >= 2u) tracks.push_back(std::move(tr));
    }

    const size_t min_tracks = static_cast<size_t>(cfg_.ba_min_tracks_multiview);
    UNICALIB_INFO("[FullMultiViewBA] 轨迹数 {} (至少 {} 条)", tracks.size(), min_tracks);
    if (tracks.size() < min_tracks) {
        UNICALIB_WARN("[FullMultiViewBA] 轨迹不足 ({} < {})，退回两两 BA", tracks.size(), min_tracks);
        out.extrinsics = calibrate_bundle_adjustment_pairwise_only(
            frames_per_cam, intrinsics, cam_ids);
        return out;
    }

    // 3) 初始化外参：cam0-cam1, cam0-cam2, ...（从轨迹构造匹配并恢复位姿）
    std::vector<Sophus::SE3d> T_cam0_cami(num_cams);
    T_cam0_cami[0] = Sophus::SE3d();
    for (size_t i = 1; i < num_cams; ++i) {
        std::vector<FeatureMatch> m0i;
        for (const auto& tr : tracks) {
            double u0 = 0, v0 = 0, ui = 0, vi = 0;
            bool has0 = false, has_i = false;
            for (const auto& o : tr) {
                if (o.cam_id == 0) { u0 = o.u; v0 = o.v; has0 = true; }
                if (o.cam_id == static_cast<int>(i)) { ui = o.u; vi = o.v; has_i = true; }
            }
            if (has0 && has_i) {
                FeatureMatch fm;
                fm.pt0 = cv::Point2f(static_cast<float>(u0), static_cast<float>(v0));
                fm.pt1 = cv::Point2f(static_cast<float>(ui), static_cast<float>(vi));
                m0i.push_back(fm);
            }
        }
        Sophus::SO3d rot;
        Eigen::Vector3d trans;
        if (m0i.size() >= 8 && recover_pose_from_E(m0i, intrinsics[0], intrinsics[i], rot, trans)) {
            T_cam0_cami[i] = Sophus::SE3d(rot, trans);
        } else {
            T_cam0_cami[i] = Sophus::SE3d();
            if (i == 1) {
                UNICALIB_WARN("[FullMultiViewBA] cam0-cam1 位姿恢复失败，退回两两 BA");
                out.extrinsics = calibrate_bundle_adjustment_pairwise_only(
                    frames_per_cam, intrinsics, cam_ids);
                return out;
            }
        }
    }

    // 4) 三角化：用 cam0 和 cam1 得到 3D 点（cam0 系），并保留对应轨迹
    // 投影矩阵: P0 = K0*[I|0], P1 = K1*[R1^T|-R1^T*t1] (点 X_cam0 在 cam1 中为 R1^T*(X-t1))
    std::vector<Eigen::Vector3d> points_3d;
    std::vector<Track> tracks_for_points;
    cv::Mat K0 = (cv::Mat_<double>(3,3) << intrinsics[0].fx, 0, intrinsics[0].cx,
                 0, intrinsics[0].fy, intrinsics[0].cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) << intrinsics[1].fx, 0, intrinsics[1].cx,
                 0, intrinsics[1].fy, intrinsics[1].cy, 0, 0, 1);
    cv::Mat P0 = cv::Mat::eye(3, 4, CV_64F);
    K0.copyTo(P0(cv::Rect(0,0,3,3)));
    Sophus::SE3d T_cam1_cam0 = T_cam0_cami[1].inverse();
    Eigen::Matrix3d R1t = T_cam1_cam0.rotationMatrix();
    Eigen::Vector3d t1 = T_cam1_cam0.translation();
    cv::Mat P1 = (cv::Mat_<double>(3,4) <<
        R1t(0,0), R1t(0,1), R1t(0,2), t1(0),
        R1t(1,0), R1t(1,1), R1t(1,2), t1(1),
        R1t(2,0), R1t(2,1), R1t(2,2), t1(2));
    P1 = K1 * P1;
    for (const auto& tr : tracks) {
        cv::Point2f pt0(tr[0].u, tr[0].v);
        cv::Point2f pt1(tr[1].u, tr[1].v);
        cv::Mat pts4d;
        cv::triangulatePoints(P0, P1, std::vector<cv::Point2f>{pt0}, std::vector<cv::Point2f>{pt1}, pts4d);
        const bool pt4_f64 = (pts4d.type() == CV_64FC1);
        double w = pt4_f64 ? pts4d.at<double>(3, 0) : static_cast<double>(pts4d.at<float>(3, 0));
        if (std::abs(w) < 1e-8) continue;
        double xw = pt4_f64 ? pts4d.at<double>(0,0) : static_cast<double>(pts4d.at<float>(0,0));
        double yw = pt4_f64 ? pts4d.at<double>(1,0) : static_cast<double>(pts4d.at<float>(1,0));
        double zw = pt4_f64 ? pts4d.at<double>(2,0) : static_cast<double>(pts4d.at<float>(2,0));
        Eigen::Vector3d X(xw/w, yw/w, zw/w);
        if (X.z() < 0.1 || X.z() > 500.0) continue;
        points_3d.push_back(X);
        tracks_for_points.push_back(tr);
    }

    const size_t min_pts = static_cast<size_t>(cfg_.ba_min_points_multiview);
    if (points_3d.size() < min_pts) {
        UNICALIB_WARN("[FullMultiViewBA] 三角化有效点过少 ({} < {})，退回两两 BA",
                      points_3d.size(), min_pts);
        out.extrinsics = calibrate_bundle_adjustment_pairwise_only(
            frames_per_cam, intrinsics, cam_ids);
        return out;
    }

    // 5) 全局 BA：参数 = [T_cam0_cam1(6), T_cam0_cam2(6), ...], [X1(3), X2(3), ...]
    ceres::Problem problem;
    std::vector<double*> extr_params(num_cams);
    std::vector<double> extr_storage((num_cams - 1) * 6);
    for (size_t i = 1; i < num_cams; ++i) {
        extr_params[i] = &extr_storage[(i - 1) * 6];
        Eigen::Vector3d rv = T_cam0_cami[i].so3().log();
        Eigen::Vector3d tv = T_cam0_cami[i].translation();
        extr_storage[(i-1)*6+0] = rv(0); extr_storage[(i-1)*6+1] = rv(1); extr_storage[(i-1)*6+2] = rv(2);
        extr_storage[(i-1)*6+3] = tv(0); extr_storage[(i-1)*6+4] = tv(1); extr_storage[(i-1)*6+5] = tv(2);
    }
    std::vector<double*> point_params(points_3d.size());
    std::vector<std::vector<double>> point_storage(points_3d.size());
    for (size_t j = 0; j < points_3d.size(); ++j) {
        point_storage[j] = {points_3d[j].x(), points_3d[j].y(), points_3d[j].z()};
        point_params[j] = point_storage[j].data();
    }

    int obs_count = 0;
    for (size_t j = 0; j < points_3d.size(); ++j) {
        const auto& tr = tracks_for_points[j];
        for (const auto& o : tr) {
            const CameraIntrinsics& K = intrinsics[o.cam_id];
            if (o.cam_id == 0) {
                ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<ReprojectCam0Cost, 2, 3>(
                    new ReprojectCam0Cost{o.u, o.v, K.fx, K.fy, K.cx, K.cy});
                problem.AddResidualBlock(cost, new ceres::HuberLoss(1.0), point_params[j]);
            } else {
                ceres::CostFunction* cost = MultiViewReprojectCost::Create(
                    o.u, o.v, K.fx, K.fy, K.cx, K.cy);
                problem.AddResidualBlock(cost, new ceres::HuberLoss(1.0),
                                         extr_params[o.cam_id], point_params[j]);
            }
            ++obs_count;
        }
    }

    ceres::Solver::Options opt;
    opt.max_num_iterations = cfg_.ba_max_iter * 2;
    opt.linear_solver_type = ceres::SPARSE_SCHUR;
    opt.minimizer_progress_to_stdout = cfg_.verbose;
    ceres::Solver::Summary summary;
    ceres::Solve(opt, &problem, &summary);
    UNICALIB_INFO("[FullMultiViewBA] {} (cost={:.4f})",
                  summary.BriefReport(), summary.final_cost);
    UNICALIB_CALC("Cam-Cam 多视图 BA Ceres initial_cost={:.6f} final_cost={:.6f} iter={} 观测数={} converged={}",
                  summary.initial_cost, summary.final_cost, static_cast<int>(summary.iterations.size()),
                  obs_count, summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");

    for (size_t i = 1; i < num_cams; ++i) {
        double* e = extr_params[i];
        ExtrinsicSE3 ext;
        ext.ref_sensor_id = cam_ids[0];
        ext.target_sensor_id = cam_ids[i];
        ext.SO3_TargetInRef = Sophus::SO3d::exp(Eigen::Vector3d(e[0], e[1], e[2]));
        ext.POS_TargetInRef = Eigen::Vector3d(e[3], e[4], e[5]);
        ext.is_converged = (summary.termination_type == ceres::CONVERGENCE);
        out.extrinsics.push_back(ext);
    }
    out.final_cost = summary.final_cost;
    out.num_points = static_cast<int>(points_3d.size());
    out.num_observations = obs_count;
    out.converged = (summary.termination_type == ceres::CONVERGENCE);
    return out;
}

// ===================================================================
// 立体矫正可视化
// ===================================================================
void CamCamCalibrator::visualize_stereo_rectification(
    const cv::Mat& img0, const cv::Mat& img1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin,
    const std::string& output_path) {

    if (img0.empty() || img1.empty()) return;

    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = safe_dist_coeffs(intrin0.dist_coeffs);
    cv::Mat D1 = safe_dist_coeffs(intrin1.dist_coeffs);

    cv::Mat R_cv, t_cv;
    cv::eigen2cv(Eigen::Matrix3d(extrin.SO3_TargetInRef.matrix()), R_cv);
    cv::eigen2cv(Eigen::Vector3d(extrin.POS_TargetInRef), t_cv);

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K0, D0, K1, D1, img0.size(),
                      R_cv, t_cv, R1, R2, P1, P2, Q);

    cv::Mat map00, map01, map10, map11;
    cv::initUndistortRectifyMap(K0, D0, R1, P1, img0.size(), CV_16SC2, map00, map01);
    cv::initUndistortRectifyMap(K1, D1, R2, P2, img1.size(), CV_16SC2, map10, map11);

    cv::Mat rect0, rect1;
    cv::remap(img0, rect0, map00, map01, cv::INTER_LINEAR);
    cv::remap(img1, rect1, map10, map11, cv::INTER_LINEAR);

    // 拼接显示
    cv::Mat vis(rect0.rows, rect0.cols + rect1.cols, rect0.type());
    rect0.copyTo(vis(cv::Rect(0, 0, rect0.cols, rect0.rows)));
    rect1.copyTo(vis(cv::Rect(rect0.cols, 0, rect1.cols, rect1.rows)));

    // 水平线验证立体矫正质量
    for (int y = 0; y < vis.rows; y += 30)
        cv::line(vis, {0,y}, {vis.cols,y}, cv::Scalar(0,255,0), 1);

    cv::imwrite(output_path, vis);
    UNICALIB_INFO("立体矫正可视化保存: {}", output_path);
}

// ===================================================================
// 评估外参
// ===================================================================
double CamCamCalibrator::evaluate(
    const std::vector<std::string>& images_cam0,
    const std::vector<std::string>& images_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin) {

    (void)extrin;
    // 通过重投影误差评估
    UNICALIB_INFO("Camera-Camera evaluate: 计算重投影误差");

    double total_rms = 0.0;
    int cnt = 0;
    size_t N = std::min(images_cam0.size(), images_cam1.size());
    for (size_t fi = 0; fi < N; fi += std::max(size_t(1), N/10)) {
        cv::Mat im0 = cv::imread(images_cam0[fi]);
        cv::Mat im1 = cv::imread(images_cam1[fi]);
        if (im0.empty() || im1.empty()) continue;
        auto fm = match_features(im0, im1, intrin0, intrin1);
        if (fm.size() < 8) continue;

        Sophus::SO3d rot;
        Eigen::Vector3d trans;
        if (recover_pose_from_E(fm, intrin0, intrin1, rot, trans)) {
            // Compare recovered rotation vs extrin
            Sophus::SO3d err = extrin.SO3_TargetInRef.inverse() * rot;
            total_rms += err.log().norm();
            ++cnt;
        }
    }
    double rms = (cnt > 0) ? total_rms / cnt : -1.0;
    UNICALIB_INFO("  Camera-Camera 评估 RMS 旋转差: {:.4f} rad", rms);
    return rms;
}

}  // namespace ns_unicalib
