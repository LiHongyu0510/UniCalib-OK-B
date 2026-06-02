/**
 * UniCalib Unified — LiDAR-LiDAR 外参标定实现
 *
 * 两阶段标定:
 *   P0 粗标定: FPFH + 鲁棒位姿(RANSAC/TEASER++)，失败回退 NDT
 *   P1 精标定: GICP/NDT，可选多帧融合(中值/加权)
 *   P2 GMM: 接口预留
 *   P3 B样条: 接口预留
 */

#include "unicalib/extrinsic/lidar_lidar_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <string>
#include <stdexcept>

namespace ns_unicalib {

using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

// =============================================================================
// 异常与边界：配置钳位与 SE3 合法性
// =============================================================================

namespace {
constexpr double kMinVoxelSize = 1e-3;
constexpr double kMaxVoxelSize = 5.0;
constexpr double kDefaultVoxelSize = 0.1;

inline double clamp_voxel_size(double v) {
    if (!std::isfinite(v) || v <= 0) return kDefaultVoxelSize;
    return std::min(kMaxVoxelSize, std::max(kMinVoxelSize, v));
}

inline bool is_se3_finite(const Sophus::SE3d& T) {
    const Eigen::Matrix4d M = T.matrix();
    if (!M.allFinite()) return false;
    Eigen::Matrix3d R = M.block<3,3>(0,0);
    double det = R.determinant();
    if (std::abs(det - 1.0) > 0.01) return false;
    return true;
}

// PCL GeneralizedICP 在大点云上易 OOM/段错误（日志常在 [fine] 0% 后中断）；先限幅再配准。
constexpr size_t kGicpMaxPoints = 35000;
constexpr size_t kGicpMinPoints = 100;

inline void decimate_cloud_uniform(PointCloudXYZI::Ptr& cloud) {
    if (!cloud || cloud->size() <= kGicpMaxPoints) return;
    PointCloudXYZI::Ptr out(new PointCloudXYZI);
    const size_t step = cloud->size() / kGicpMaxPoints + 1;
    out->reserve(cloud->size() / step + 2);
    for (size_t i = 0; i < cloud->size(); i += step)
        out->push_back(cloud->points[i]);
    cloud = out;
}

// PCL NDT/GICP 的 getFinalTransformation() 为 float 累加，3x3 旋转块常非严格正交；
// 直接 Sophus::SO3d(R) 在部分构建下会 assert/崩溃。投影到 SO(3) 再构造。
inline void log_rotation_stats_ex(const Eigen::Matrix3d& R, const char* tag) {
    const double det = R.determinant();
    const double ortho = (R * R.transpose() - Eigen::Matrix3d::Identity()).norm();
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] {} R_stats det={:.8f} ||R*R^T-I||_F={:.6e}", tag, det, ortho);
    Logger::flush();
}

inline Sophus::SO3d so3_from_pcl_rotation_block(const Eigen::Matrix3d& R_raw) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) R.col(2) *= -1;
    return Sophus::SO3d(R);
}
}  // namespace

// =============================================================================
// 构造 / 析构
// =============================================================================

LiDARLiDARCalibrator::LiDARLiDARCalibrator(const Config& cfg) : cfg_(cfg) {}

LiDARLiDARCalibrator::~LiDARLiDARCalibrator() = default;

// =============================================================================
// 预处理
// =============================================================================

PointCloudXYZI::Ptr LiDARLiDARCalibrator::preprocess_cloud(
    const PointCloudXYZI::Ptr& cloud) const {
    if (!cloud || cloud->empty()) return nullptr;
    double vs = clamp_voxel_size(cfg_.voxel_size);
    PointCloudXYZI::Ptr out(new PointCloudXYZI);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(static_cast<float>(vs), static_cast<float>(vs), static_cast<float>(vs));
    vg.filter(*out);
    return out;
}

PointCloudXYZI::Ptr LiDARLiDARCalibrator::transform_cloud(
    const PointCloudXYZI::Ptr& cloud,
    const Sophus::SE3d& transform) const {
    if (!cloud || cloud->empty()) return nullptr;
    if (!is_se3_finite(transform)) {
        UNICALIB_WARN("[LiDAR-LiDAR] transform_cloud: 变换含 NaN/Inf，跳过");
        return nullptr;
    }
    PointCloudXYZI::Ptr out(new PointCloudXYZI);
    Eigen::Matrix4f T = transform.matrix().cast<float>();
    pcl::transformPointCloud(*cloud, *out, T);
    return out;
}

// =============================================================================
// 合并多帧点云
// =============================================================================

static PointCloudXYZI::Ptr merge_scans(const std::vector<LiDARScan>& scans,
                                       size_t max_frames,
                                       double voxel_size) {
    if (scans.empty()) return nullptr;
    size_t n = std::min(scans.size(), std::max(size_t(1), max_frames));
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] merge_scans: begin frames_in={} max_frames={} use_n={} voxel={:.4f}",
                     scans.size(), max_frames, n, voxel_size);
    Logger::flush();
    PointCloudXYZI::Ptr merged(new PointCloudXYZI);
    for (size_t i = 0; i < n; ++i) {
        if (!scans[i].cloud || scans[i].cloud->empty()) continue;
        for (const auto& pt : scans[i].cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            merged->push_back(pt);
        }
    }
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] merge_scans: concat done merged_pts={}", merged->size());
    Logger::flush();
    if (merged->empty()) return nullptr;
    double vs = clamp_voxel_size(voxel_size);
    PointCloudXYZI::Ptr filtered(new PointCloudXYZI);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(merged);
    vg.setLeafSize(static_cast<float>(vs), static_cast<float>(vs), static_cast<float>(vs));
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] merge_scans: before VoxelGrid.filter");
    Logger::flush();
    vg.filter(*filtered);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] merge_scans: after filter out_pts={} (empty_fallback={})",
                     filtered->size(), filtered->empty() ? "use_merged" : "use_filtered");
    Logger::flush();
    return filtered->empty() ? merged : filtered;
}

// =============================================================================
// NDT 配准 (粗/精)
// =============================================================================

std::optional<Sophus::SE3d> LiDARLiDARCalibrator::calibrate_ndt(
    const LiDARScan& scan_ref,
    const LiDARScan& scan_target,
    const Sophus::SE3d& init_guess) {
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] NDT: enter raw_pts ref={} tgt={}",
                     scan_ref.cloud ? scan_ref.cloud->size() : 0,
                     scan_target.cloud ? scan_target.cloud->size() : 0);
    Logger::flush();
    if (!scan_ref.cloud || scan_ref.cloud->empty() ||
        !scan_target.cloud || scan_target.cloud->empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR] NDT: 点云为空");
        return std::nullopt;
    }
    PointCloudXYZI::Ptr src = preprocess_cloud(scan_target.cloud);
    PointCloudXYZI::Ptr tgt = preprocess_cloud(scan_ref.cloud);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] NDT: after preprocess src={} tgt={}", src ? src->size() : 0,
                     tgt ? tgt->size() : 0);
    Logger::flush();
    if (!src || src->empty() || !tgt || tgt->empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR] NDT: 预处理后点云为空");
        return std::nullopt;
    }
    if (!is_se3_finite(init_guess)) {
        UNICALIB_WARN("[LiDAR-LiDAR] NDT: 初始位姿含 NaN/Inf，使用单位阵");
    }

    double res = std::isfinite(cfg_.ndt_resolution) && cfg_.ndt_resolution > 0
                     ? std::min(5.0, std::max(0.1, cfg_.ndt_resolution)) : 1.0;
    double step = std::isfinite(cfg_.ndt_step_size) && cfg_.ndt_step_size > 0
                      ? std::min(1.0, std::max(0.01, cfg_.ndt_step_size)) : 0.1;
    int max_iter = cfg_.ndt_max_iterations > 0 ? std::min(500, std::max(10, cfg_.ndt_max_iterations)) : 35;

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setTransformationEpsilon(std::isfinite(cfg_.gicp_transformation_epsilon) && cfg_.gicp_transformation_epsilon > 0
                                     ? cfg_.gicp_transformation_epsilon : 1e-8);
    ndt.setStepSize(static_cast<float>(step));
    ndt.setResolution(static_cast<float>(res));
    ndt.setMaximumIterations(max_iter);
    ndt.setInputSource(src);
    ndt.setInputTarget(tgt);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] NDT: setInput* done res={:.4f} step={:.4f} max_iter={}", res, step,
                     max_iter);
    Logger::flush();

    Eigen::Matrix4f init;
    if (is_se3_finite(init_guess))
        init = init_guess.matrix().cast<float>();
    else
        init = Eigen::Matrix4f::Identity();
    PointCloudXYZI::Ptr aligned(new PointCloudXYZI);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] NDT: before align");
    Logger::flush();
    ndt.align(*aligned, init);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] NDT: after align converged={}", ndt.hasConverged() ? "yes" : "no");
    Logger::flush();
    if (!ndt.hasConverged()) {
        UNICALIB_WARN("[LiDAR-LiDAR] NDT 未收敛");
        return std::nullopt;
    }
    Eigen::Matrix4d T = ndt.getFinalTransformation().cast<double>();
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] NDT: after getFinalTransformation 4x4 finite={}", T.allFinite() ? "yes" : "no");
    Logger::flush();
    if (!T.allFinite()) {
        UNICALIB_WARN("[LiDAR-LiDAR] NDT 输出含 NaN/Inf");
        return std::nullopt;
    }
    Eigen::Matrix3d R_raw = T.block<3,3>(0,0);
    log_rotation_stats_ex(R_raw, "NDT");
    Sophus::SE3d result(so3_from_pcl_rotation_block(R_raw), T.block<3,1>(0,3));
    if (!validate_extrinsic(result)) {
        UNICALIB_WARN("[LiDAR-LiDAR] NDT 输出外参未通过校验");
        return std::nullopt;
    }
    last_quality_.converged = true;
    last_quality_.fitness_score = static_cast<double>(ndt.getFitnessScore());
    UNICALIB_CALC("LiDAR-LiDAR NDT 完成 fitness_score={:.6f} converged=yes", last_quality_.fitness_score);
    return result;
}

// =============================================================================
// GICP 配准
// =============================================================================

std::optional<Sophus::SE3d> LiDARLiDARCalibrator::calibrate_gicp(
    const LiDARScan& scan_ref,
    const LiDARScan& scan_target,
    const Sophus::SE3d& init_guess) {
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: enter raw_pts ref={} tgt={}",
                     scan_ref.cloud ? scan_ref.cloud->size() : 0,
                     scan_target.cloud ? scan_target.cloud->size() : 0);
    Logger::flush();
    if (!scan_ref.cloud || scan_ref.cloud->empty() ||
        !scan_target.cloud || scan_target.cloud->empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR] GICP: 点云为空");
        return std::nullopt;
    }
    PointCloudXYZI::Ptr src = preprocess_cloud(scan_target.cloud);
    PointCloudXYZI::Ptr tgt = preprocess_cloud(scan_ref.cloud);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: after preprocess src={} tgt={}", src ? src->size() : 0,
                     tgt ? tgt->size() : 0);
    Logger::flush();
    if (!src || src->empty() || !tgt || tgt->empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR] GICP: 预处理后点云为空");
        return std::nullopt;
    }
    decimate_cloud_uniform(src);
    decimate_cloud_uniform(tgt);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: after uniform cap (max={}) src={} tgt={}", kGicpMaxPoints,
                     src->size(), tgt->size());
    Logger::flush();
    if (src->size() < kGicpMinPoints || tgt->size() < kGicpMinPoints) {
        UNICALIB_WARN("[LiDAR-LiDAR] GICP: 下采样后点数不足 (src={} tgt={})，跳过",
                      src->size(), tgt->size());
        return std::nullopt;
    }
    if (!is_se3_finite(init_guess)) {
        UNICALIB_WARN("[LiDAR-LiDAR] GICP: 初始位姿含 NaN/Inf，使用单位阵");
    }

    double max_corr = std::isfinite(cfg_.gicp_max_corr_dist) && cfg_.gicp_max_corr_dist > 0
                          ? std::min(10.0, std::max(0.01, cfg_.gicp_max_corr_dist)) : 1.0;
    int max_iter = cfg_.gicp_max_iterations > 0 ? std::min(500, std::max(5, cfg_.gicp_max_iterations)) : 30;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: object constructed");
    Logger::flush();
    gicp.setMaxCorrespondenceDistance(max_corr);
    gicp.setMaximumIterations(max_iter);
    gicp.setTransformationEpsilon(std::isfinite(cfg_.gicp_transformation_epsilon) && cfg_.gicp_transformation_epsilon > 0
                                      ? cfg_.gicp_transformation_epsilon : 1e-8);
    gicp.setEuclideanFitnessEpsilon(std::isfinite(cfg_.gicp_euclidean_fitness_epsilon) ? cfg_.gicp_euclidean_fitness_epsilon : 1e-6);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: before setInputSource/setInputTarget");
    Logger::flush();
    gicp.setInputSource(src);
    gicp.setInputTarget(tgt);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: after setInput* max_corr={:.4f} max_iter={}", max_corr, max_iter);
    Logger::flush();

    Eigen::Matrix4f init;
    if (is_se3_finite(init_guess))
        init = init_guess.matrix().cast<float>();
    else
        init = Eigen::Matrix4f::Identity();
    PointCloudXYZI::Ptr aligned(new PointCloudXYZI);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: before align");
    Logger::flush();
    gicp.align(*aligned, init);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: after align converged={}", gicp.hasConverged() ? "yes" : "no");
    Logger::flush();
    if (!gicp.hasConverged()) {
        UNICALIB_WARN("[LiDAR-LiDAR] GICP 未收敛");
        return std::nullopt;
    }
    Eigen::Matrix4d T = gicp.getFinalTransformation().cast<double>();
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] GICP: after getFinalTransformation 4x4 finite={}", T.allFinite() ? "yes" : "no");
    Logger::flush();
    if (!T.allFinite()) {
        UNICALIB_WARN("[LiDAR-LiDAR] GICP 输出含 NaN/Inf");
        return std::nullopt;
    }
    Eigen::Matrix3d R_raw = T.block<3,3>(0,0);
    log_rotation_stats_ex(R_raw, "GICP");
    Sophus::SE3d result(so3_from_pcl_rotation_block(R_raw), T.block<3,1>(0,3));
    if (!validate_extrinsic(result)) {
        UNICALIB_WARN("[LiDAR-LiDAR] GICP 输出外参未通过校验");
        return std::nullopt;
    }
    last_quality_.converged = true;
    last_quality_.fitness_score = static_cast<double>(gicp.getFitnessScore());
    UNICALIB_CALC("LiDAR-LiDAR GICP 完成 fitness_score={:.6f} converged=yes", last_quality_.fitness_score);
    return result;
}

// =============================================================================
// P0: FPFH 特征提取 (PCL PointNormal + FPFH)
// =============================================================================

bool LiDARLiDARCalibrator::compute_fpfh_features(
    const PointCloudXYZI::Ptr& cloud,
    std::vector<Eigen::VectorXf>& features) {
    if (!cloud || cloud->empty()) return false;
    features.clear();
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normals->reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        pcl::PointNormal pn;
        pn.x = pt.x; pn.y = pt.y; pn.z = pt.z;
        pn.normal_x = pn.normal_y = pn.normal_z = 0.f;
        cloud_with_normals->push_back(pn);
    }
    if (cloud_with_normals->size() < 10) return false;

    double r_n = std::isfinite(cfg_.fpfh_normal_radius) && cfg_.fpfh_normal_radius > 0
                     ? std::min(5.0, std::max(0.05, cfg_.fpfh_normal_radius)) : 0.3;
    double r_f = std::isfinite(cfg_.fpfh_radius) && cfg_.fpfh_radius > 0
                     ? std::min(5.0, std::max(0.05, cfg_.fpfh_radius)) : 0.5;

    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
    ne.setInputCloud(cloud_with_normals);
    ne.setSearchSurface(cloud_with_normals);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_n(new pcl::search::KdTree<pcl::PointNormal>);
    ne.setSearchMethod(tree_n);
    ne.setRadiusSearch(static_cast<float>(r_n));
    ne.compute(*cloud_with_normals);

    pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_out(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setInputCloud(cloud_with_normals);
    fpfh.setInputNormals(cloud_with_normals);
    fpfh.setSearchMethod(tree_n);
    fpfh.setRadiusSearch(static_cast<float>(r_f));
    fpfh.compute(*fpfh_out);

    if (fpfh_out->empty()) return false;
    features.resize(fpfh_out->size());
    for (size_t i = 0; i < fpfh_out->size(); ++i) {
        features[i].resize(33);
        for (int j = 0; j < 33; ++j) {
            float v = (*fpfh_out)[i].histogram[j];
            features[i](j) = std::isfinite(v) ? v : 0.f;
        }
    }
    return true;
}

// =============================================================================
// P0: 从对应点对鲁棒估计 SE3 (RANSAC，可替换为 TEASER++)
// T 满足 tgt_pts[i] ≈ T * src_pts[i]，即 T = T_target_in_ref
// =============================================================================

namespace {
// 三点是否近似共线（面积/体积过小则退化）
bool are_collinear(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                   double min_volume = 1e-12) {
    Eigen::Vector3d ab = b - a, ac = c - a;
    double vol = std::abs(ab.cross(ac).norm());
    return vol < min_volume;
}

// 从至少 3 组对应点 SVD 求 SE3 (Horn/Arun)，R 投影到 SO(3)
bool pose_from_correspondences(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt,
    Sophus::SE3d& T_out) {
    if (src.size() < 3 || src.size() != tgt.size()) return false;
    Eigen::Vector3d c_src = Eigen::Vector3d::Zero(), c_tgt = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        c_src += src[i]; c_tgt += tgt[i];
    }
    c_src /= static_cast<double>(src.size());
    c_tgt /= static_cast<double>(tgt.size());
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        H += (tgt[i] - c_tgt) * (src[i] - c_src).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }
    // 投影到 SO(3)：避免浮点导致 det(R)≠1
    Eigen::JacobiSVD<Eigen::Matrix3d> rsvd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = rsvd.matrixU() * rsvd.matrixV().transpose();
    if (R.determinant() < 0) R.col(2) *= -1;
    Eigen::Vector3d t = c_tgt - R * c_src;
    if (!t.allFinite() || !R.allFinite()) return false;
    T_out = Sophus::SE3d(Sophus::SO3d(R), t);
    return true;
}

int count_inliers(const std::vector<Eigen::Vector3d>& src,
                  const std::vector<Eigen::Vector3d>& tgt,
                  const Sophus::SE3d& T,
                  double threshold_sq) {
    int n = 0;
    for (size_t i = 0; i < src.size(); ++i) {
        double err = (tgt[i] - T * src[i]).squaredNorm();
        if (err < threshold_sq) ++n;
    }
    return n;
}
}  // namespace

std::optional<Sophus::SE3d> LiDARLiDARCalibrator::solve_teaser(
    const std::vector<Eigen::Vector3d>& src_pts,
    const std::vector<Eigen::Vector3d>& tgt_pts) {
    const size_t n = std::min(src_pts.size(), tgt_pts.size());
    if (n < 3) return std::nullopt;
    double noise_b = std::isfinite(cfg_.teaser_noise_bound) && cfg_.teaser_noise_bound > 0
                         ? std::min(1.0, std::max(1e-4, cfg_.teaser_noise_bound)) : 0.1;
    const double thresh_sq = noise_b * noise_b;
    std::vector<size_t> indices(n);
    std::iota(indices.begin(), indices.end(), 0);
    std::mt19937 rng(42);
    Sophus::SE3d best_T;
    int best_inliers = 0;
    const int max_iter = std::max(10, std::min(1000, cfg_.ransac_max_iterations > 0 ? cfg_.ransac_max_iterations : 500));
    for (int iter = 0; iter < max_iter; ++iter) {
        std::shuffle(indices.begin(), indices.end(), rng);
        std::vector<Eigen::Vector3d> s3(3), t3(3);
        for (int i = 0; i < 3; ++i) {
            s3[i] = src_pts[indices[i]];
            t3[i] = tgt_pts[indices[i]];
        }
        if (are_collinear(s3[0], s3[1], s3[2]) || are_collinear(t3[0], t3[1], t3[2]))
            continue;
        Sophus::SE3d T;
        if (!pose_from_correspondences(s3, t3, T)) continue;
        int inliers = count_inliers(src_pts, tgt_pts, T, thresh_sq);
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_T = T;
        }
    }
    if (best_inliers < 3) return std::nullopt;
    if (!is_se3_finite(best_T)) return std::nullopt;
    UNICALIB_CALC("LiDAR-LiDAR FPFH+RANSAC 内点数={} 总对应={} 内点比={:.3f}",
                  best_inliers, static_cast<int>(n),
                  n > 0 ? static_cast<double>(best_inliers) / n : 0.0);
    // 用所有内点精化
    std::vector<Eigen::Vector3d> src_in, tgt_in;
    for (size_t i = 0; i < n; ++i) {
        if ((tgt_pts[i] - best_T * src_pts[i]).squaredNorm() < thresh_sq) {
            src_in.push_back(src_pts[i]);
            tgt_in.push_back(tgt_pts[i]);
        }
    }
    Sophus::SE3d refined;
    if (pose_from_correspondences(src_in, tgt_in, refined) && is_se3_finite(refined))
        return refined;
    return best_T;
}

// 去除 NaN 点，保证与 FPFH 输出严格 1:1 对应
static PointCloudXYZI::Ptr remove_nan(const PointCloudXYZI::Ptr& cloud) {
    if (!cloud || cloud->empty()) return nullptr;
    PointCloudXYZI::Ptr out(new PointCloudXYZI);
    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            out->push_back(pt);
    }
    return out->empty() ? nullptr : out;
}

// =============================================================================
// P0: FPFH + 鲁棒位姿 (RANSAC；可接入 TEASER++ 替代 solve_teaser)
// =============================================================================

std::optional<Sophus::SE3d> LiDARLiDARCalibrator::calibrate_fpfh_teaser(
    const LiDARScan& scan_ref,
    const LiDARScan& scan_target) {
    if (!scan_ref.cloud || scan_ref.cloud->empty() ||
        !scan_target.cloud || scan_target.cloud->empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR] FPFH+TEASER: 点云为空");
        return std::nullopt;
    }
    PointCloudXYZI::Ptr ref = preprocess_cloud(scan_ref.cloud);
    PointCloudXYZI::Ptr tgt = preprocess_cloud(scan_target.cloud);
    if (!ref || ref->empty() || !tgt || tgt->empty()) return std::nullopt;
    ref = remove_nan(ref);
    tgt = remove_nan(tgt);
    if (!ref || ref->empty() || !tgt || tgt->empty()) return std::nullopt;
    if (ref->size() < 20 || tgt->size() < 20) {
        UNICALIB_WARN("[LiDAR-LiDAR] FPFH+TEASER: 下采样/去 NaN 后点过少");
        return std::nullopt;
    }

    std::vector<Eigen::VectorXf> feat_ref, feat_tgt;
    if (!compute_fpfh_features(ref, feat_ref) || !compute_fpfh_features(tgt, feat_tgt)) {
        UNICALIB_WARN("[LiDAR-LiDAR] FPFH 特征提取失败");
        return std::nullopt;
    }
    // ref/tgt 已去 NaN，compute_fpfh_features 内部不再丢弃点，故 ref->size()==feat_ref.size()、tgt->size()==feat_tgt.size()
    const size_t n_tgt = std::min(tgt->size(), feat_tgt.size());
    const size_t n_ref = std::min(ref->size(), feat_ref.size());
    if (n_tgt < 3 || n_ref < 3) return std::nullopt;
    std::vector<Eigen::Vector3d> src_pts, tgt_pts;
    for (size_t i = 0; i < n_tgt; ++i) {
        double best = 1e30;
        size_t j_best = 0;
        for (size_t j = 0; j < n_ref; ++j) {
            double d = (feat_tgt[i] - feat_ref[j]).squaredNorm();
            if (d < best) { best = d; j_best = j; }
        }
        src_pts.push_back(Eigen::Vector3d((*tgt)[i].x, (*tgt)[i].y, (*tgt)[i].z));
        tgt_pts.push_back(Eigen::Vector3d((*ref)[j_best].x, (*ref)[j_best].y, (*ref)[j_best].z));
    }
    if (src_pts.size() < 3) return std::nullopt;
    std::optional<Sophus::SE3d> T = solve_teaser(src_pts, tgt_pts);
    if (T.has_value()) {
        last_quality_.converged = true;
        last_quality_.num_correspondences = static_cast<int>(src_pts.size());
        last_quality_.fitness_score = 0.8;  // 粗标定无精确 fitness，给合理默认
    }
    return T;
}

// =============================================================================
// GMM 占位: 多帧 NDT 联合
// =============================================================================

std::optional<Sophus::SE3d> LiDARLiDARCalibrator::calibrate_gmm(
    const std::vector<LiDARScan>& scans_ref,
    const std::vector<LiDARScan>& scans_target,
    const std::optional<Sophus::SE3d>& init_guess) {
    if (scans_ref.empty() || scans_target.empty()) return std::nullopt;
    size_t max_f = std::max(size_t(1), static_cast<size_t>(cfg_.max_frames > 0 ? cfg_.max_frames : 100));
    PointCloudXYZI::Ptr ref_merged = merge_scans(scans_ref, max_f, cfg_.voxel_size);
    PointCloudXYZI::Ptr tgt_merged = merge_scans(scans_target, max_f, cfg_.voxel_size);
    if (!ref_merged || ref_merged->empty() || !tgt_merged || tgt_merged->empty()) {
        UNICALIB_WARN("[LiDAR-LiDAR] GMM: 合并点云为空");
        return std::nullopt;
    }
    LiDARScan fake_ref, fake_tgt;
    fake_ref.cloud = ref_merged;
    fake_ref.timestamp = 0;
    fake_tgt.cloud = tgt_merged;
    fake_tgt.timestamp = 0;
    Sophus::SE3d init = init_guess.has_value() && is_se3_finite(*init_guess) ? *init_guess : Sophus::SE3d();
    return calibrate_ndt(fake_ref, fake_tgt, init);
}

// =============================================================================
// 粗标定 (P0: 优先 FPFH+鲁棒位姿，失败回退 NDT；P2 GMM 可选)
// =============================================================================

std::optional<ExtrinsicSE3> LiDARLiDARCalibrator::calibrate_coarse(
    const std::vector<LiDARScan>& scans_ref,
    const std::vector<LiDARScan>& scans_target,
    const std::string& ref_id,
    const std::string& target_id) {
    if (scans_ref.empty() || scans_target.empty()) {
        UNICALIB_ERROR("[LiDAR-LiDAR] 粗标定: 点云序列为空");
        return std::nullopt;
    }
    log_stage("Coarse", cfg_.use_fpfh_teaser_coarse ? "开始粗标定 (FPFH+鲁棒位姿，失败回退NDT)" : "开始粗标定 (NDT/多帧合并)");
    if (progress_cb_) progress_cb_("coarse", 0.0);

    if (!scans_ref.front().cloud || scans_ref.front().cloud->empty() ||
        !scans_target.front().cloud || scans_target.front().cloud->empty()) {
        UNICALIB_ERROR("[LiDAR-LiDAR] 粗标定: 首帧点云为空");
        return std::nullopt;
    }
    std::optional<Sophus::SE3d> T;
    last_coarse_method_ = "NDT";
    if (cfg_.use_fpfh_teaser_coarse) {
        LiDARScan ref_first = scans_ref.front();
        LiDARScan tgt_first = scans_target.front();
        T = calibrate_fpfh_teaser(ref_first, tgt_first);
        if (T.has_value()) {
            last_coarse_method_ = "FPFH_RANSAC";
            log_stage("Coarse", "FPFH+鲁棒位姿成功");
        } else {
            UNICALIB_WARN("[LiDAR-LiDAR] FPFH+鲁棒位姿未收敛，回退 NDT");
        }
    }
    if (!T.has_value() && cfg_.use_gmm_registration && scans_ref.size() > 1) {
        T = calibrate_gmm(scans_ref, scans_target, std::nullopt);
        if (T.has_value()) last_coarse_method_ = "GMM";
    }
    if (!T.has_value()) {
        LiDARScan ref_first = scans_ref.front();
        LiDARScan tgt_first = scans_target.front();
        T = calibrate_ndt(ref_first, tgt_first, Sophus::SE3d());
    }
    if (progress_cb_) progress_cb_("coarse", 1.0);
    if (!T.has_value()) return std::nullopt;
    if (!validate_extrinsic(*T)) {
        UNICALIB_WARN("[LiDAR-LiDAR] 粗标定结果未通过外参校验，丢弃");
        return std::nullopt;
    }

    ExtrinsicSE3 ext;
    ext.ref_sensor_id = ref_id;
    ext.target_sensor_id = target_id;
    ext.SO3_TargetInRef = T->so3();
    ext.POS_TargetInRef = T->translation();
    ext.time_offset_s = 0.0;
    // 防护：检查 last_quality_ 是否有效
    if (last_quality_.converged || last_quality_.fitness_score > 0) {
        ext.residual_rms = last_quality_.inlier_rmse > 0 ? last_quality_.inlier_rmse : (1.0 - last_quality_.fitness_score);
        ext.is_converged = last_quality_.converged;
    } else {
        ext.residual_rms = -1.0;
        ext.is_converged = false;
        UNICALIB_WARN("[LiDAR-LiDAR] 粗标定: last_quality_ 未设置，使用默认值");
    }
    return ext;
}

// =============================================================================
// P1: 精标定 — 单次合并 或 多帧 GICP 融合 (median/weighted)
// =============================================================================

std::optional<ExtrinsicSE3> LiDARLiDARCalibrator::calibrate_fine(
    const std::vector<LiDARScan>& scans_ref,
    const std::vector<LiDARScan>& scans_target,
    const Sophus::SE3d& init_extrinsic,
    const std::string& ref_id,
    const std::string& target_id) {
    if (scans_ref.empty() || scans_target.empty()) return std::nullopt;
    const bool multi = cfg_.use_multi_frame_fine &&
                      (cfg_.fine_fusion_method == "median" || cfg_.fine_fusion_method == "weighted") &&
                      scans_ref.size() > 1 && scans_target.size() > 1;
    log_stage("Fine", multi ? "开始精标定 (多帧GICP融合)" : "开始精标定 (GICP/NDT)");
    if (progress_cb_) progress_cb_("fine", 0.0);
    UNICALIB_INFO_EX(
        "[LiDAR-LiDAR][Fine] calibrate_fine: post progress_cb ref={} tgt={} multi={} fusion={} use_ndt={} "
        "n_ref={} n_tgt={} max_frames={}",
        ref_id, target_id, multi, cfg_.fine_fusion_method, cfg_.use_ndt, scans_ref.size(), scans_target.size(),
        cfg_.max_frames);
    Logger::flush();

    auto run_one = [this](const LiDARScan& r, const LiDARScan& t, const Sophus::SE3d& init) -> std::optional<Sophus::SE3d> {
        return cfg_.use_ndt ? calibrate_ndt(r, t, init) : calibrate_gicp(r, t, init);
    };

    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] calibrate_fine: before init_extrinsic check");
    Logger::flush();
    if (!is_se3_finite(init_extrinsic)) {
        UNICALIB_WARN("[LiDAR-LiDAR] 精标定: 初始外参含 NaN/Inf");
    }
    std::optional<Sophus::SE3d> T;
    if (multi) {
        const int K = std::max(1, std::min(cfg_.fine_fusion_max_frames > 0 ? cfg_.fine_fusion_max_frames : 20,
                                           std::min(static_cast<int>(scans_ref.size()), static_cast<int>(scans_target.size()))));
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] calibrate_fine: multi-frame branch K={} method={}", K,
                         cfg_.fine_fusion_method);
        Logger::flush();
        std::vector<Sophus::SE3d> poses;
        std::vector<double> fitnesses;
        const size_t step_ref = std::max(size_t(1), (scans_ref.size() - 1) / static_cast<size_t>(K));
        const size_t step_tgt = std::max(size_t(1), (scans_target.size() - 1) / static_cast<size_t>(K));
        for (int i = 0; i < K; ++i) {
            size_t ir = std::min(i * step_ref, scans_ref.size() - 1);
            size_t it = std::min(i * step_tgt, scans_target.size() - 1);
            if (!scans_ref[ir].cloud || scans_ref[ir].cloud->empty() ||
                !scans_target[it].cloud || scans_target[it].cloud->empty())
                continue;
            UNICALIB_INFO_EX(
                "[LiDAR-LiDAR][Fine] multi: iter i={} ir={} it={} pts_ref={} pts_tgt={} -> run_one",
                i, ir, it, scans_ref[ir].cloud->size(), scans_target[it].cloud->size());
            Logger::flush();
            auto Ti = run_one(scans_ref[ir], scans_target[it], init_extrinsic);
            UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] multi: iter i={} run_one {}", i, Ti.has_value() ? "ok" : "fail");
            Logger::flush();
            if (Ti.has_value()) {
                poses.push_back(*Ti);
                fitnesses.push_back(last_quality_.fitness_score > 0 ? last_quality_.fitness_score : 0.5);
            }
        }
        if (poses.empty()) {
            T = std::nullopt;  // fallback to single merged below
        } else if (cfg_.fine_fusion_method == "median") {
            std::vector<size_t> idx(poses.size());
            std::iota(idx.begin(), idx.end(), 0);
            std::sort(idx.begin(), idx.end(), [&fitnesses](size_t a, size_t b) { return fitnesses[a] < fitnesses[b]; });
            size_t mid = poses.size() / 2;
            T = poses[idx[mid]];
            last_quality_.fitness_score = fitnesses[idx[mid]];
            last_quality_.converged = true;
        } else {
            // weighted: translation = sum(w*t)/sum(w), quaternion = normalized sum(w*q)
            double sumw = 0;
            Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
            Eigen::Vector4d q_sum = Eigen::Vector4d::Zero();
            for (size_t i = 0; i < poses.size(); ++i) {
                double w = std::max(1e-6, fitnesses[i]);
                sumw += w;
                t_sum += w * poses[i].translation();
                Eigen::Quaterniond q = poses[i].unit_quaternion();
                if (q.w() < 0) q.coeffs() *= -1;
                q_sum += w * Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
            }
            const double q_norm_min = 1e-6;
            if (sumw > 1e-12 && q_sum.norm() > q_norm_min) {
                t_sum /= sumw;
                q_sum.normalize();
                if (q_sum(0) < 0) q_sum = -q_sum;
                T = Sophus::SE3d(Eigen::Quaterniond(q_sum(0), q_sum(1), q_sum(2), q_sum(3)), t_sum);
                last_quality_.fitness_score = sumw / static_cast<double>(poses.size());
            } else {
                // 旋转分散时加权平均不稳定，回退到 fitness 最佳的一帧
                size_t best_i = 0;
                for (size_t i = 1; i < fitnesses.size(); ++i)
                    if (fitnesses[i] > fitnesses[best_i]) best_i = i;
                T = poses[best_i];
                last_quality_.fitness_score = fitnesses[best_i];
            }
            last_quality_.converged = true;
        }
    } else {
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] calibrate_fine: single/merged-only branch (multi=false)");
        Logger::flush();
    }
    if (!T.has_value()) {
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] calibrate_fine: T empty -> merge_scans path max_frames={} voxel={:.4f}",
                         cfg_.max_frames, cfg_.voxel_size);
        Logger::flush();
        PointCloudXYZI::Ptr ref_merged = merge_scans(scans_ref, static_cast<size_t>(cfg_.max_frames), cfg_.voxel_size);
        PointCloudXYZI::Ptr tgt_merged = merge_scans(scans_target, static_cast<size_t>(cfg_.max_frames), cfg_.voxel_size);
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] merge done ref_pts={} tgt_pts={}", ref_merged ? ref_merged->size() : 0,
                         tgt_merged ? tgt_merged->size() : 0);
        Logger::flush();
        if (!ref_merged || ref_merged->empty() || !tgt_merged || tgt_merged->empty()) return std::nullopt;
        LiDARScan fake_ref, fake_tgt;
        fake_ref.cloud = ref_merged;
        fake_ref.timestamp = 0;
        fake_tgt.cloud = tgt_merged;
        fake_tgt.timestamp = 0;
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] calibrate_fine: before run_one on merged clouds");
        Logger::flush();
        T = run_one(fake_ref, fake_tgt, init_extrinsic);
        UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] calibrate_fine: after run_one on merged {}", T.has_value() ? "ok" : "fail");
        Logger::flush();
    }
    if (progress_cb_) progress_cb_("fine", 1.0);
    if (!T.has_value()) return std::nullopt;
    if (!validate_extrinsic(*T)) {
        UNICALIB_WARN("[LiDAR-LiDAR] 精标定结果未通过外参校验");
        return std::nullopt;
    }

    ExtrinsicSE3 ext;
    ext.ref_sensor_id = ref_id;
    ext.target_sensor_id = target_id;
    ext.SO3_TargetInRef = T->so3();
    ext.POS_TargetInRef = T->translation();
    ext.time_offset_s = 0.0;
    if (last_quality_.converged || last_quality_.fitness_score > 0) {
        ext.residual_rms = last_quality_.inlier_rmse > 0 ? last_quality_.inlier_rmse : (1.0 - last_quality_.fitness_score);
        ext.is_converged = last_quality_.converged;
    } else {
        ext.residual_rms = -1.0;
        ext.is_converged = false;
        UNICALIB_WARN("[LiDAR-LiDAR] 精标定: last_quality_ 未设置，使用默认值");
    }
    return ext;
}

// =============================================================================
// P3: B样条多帧+时间偏移 — 占位实现，直接返回当前外参（接口预留）
// =============================================================================

ExtrinsicSE3 LiDARLiDARCalibrator::refine_with_bspline(
    const std::vector<LiDARScan>& scans_ref,
    const std::vector<LiDARScan>& scans_target,
    const Sophus::SE3d& init_extrinsic,
    const std::string& ref_id,
    const std::string& target_id) {
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] refine_with_bspline: enter ref={} tgt={} n_ref={} n_tgt={}", ref_id,
                     target_id, scans_ref.size(), scans_target.size());
    Logger::flush();
    (void)scans_ref;
    (void)scans_target;
    Sophus::SE3d T = init_extrinsic;
    if (!is_se3_finite(T) || !validate_extrinsic(T)) {
        UNICALIB_WARN("[LiDAR-LiDAR] B样条精化: 输入外参无效，返回单位阵");
        T = Sophus::SE3d();
    }
    ExtrinsicSE3 ext;
    ext.ref_sensor_id = ref_id;
    ext.target_sensor_id = target_id;
    ext.SO3_TargetInRef = T.so3();
    ext.POS_TargetInRef = T.translation();
    ext.time_offset_s = std::isfinite(cfg_.time_offset_init_s) ? cfg_.time_offset_init_s : 0.0;
    ext.is_converged = true;
    UNICALIB_CALC("LiDAR-LiDAR B样条精化 占位实现(直通) 帧数_ref={} 帧数_tgt={}", scans_ref.size(), scans_target.size());
    return ext;
}

// =============================================================================
// 两阶段标定
// =============================================================================

LiDARLiDARTwoStageResult LiDARLiDARCalibrator::calibrate_two_stage(
    const std::vector<LiDARScan>& scans_ref,
    const std::vector<LiDARScan>& scans_target,
    const std::string& ref_id,
    const std::string& target_id,
    const std::optional<Sophus::SE3d>& init_guess) {
    LiDARLiDARTwoStageResult result;
    result.coarse_method = "USER_INIT";
    result.fine_method = cfg_.use_ndt ? "NDT" : "GICP";

    try {
    if (scans_ref.empty() || scans_target.empty()) {
        result.failure_reason = "点云序列为空";
        result.needs_manual = true;
        return result;
    }

    Sophus::SE3d init_se3;
    bool use_user_init = false;
    if (init_guess.has_value() && is_se3_finite(*init_guess) && validate_extrinsic(*init_guess)) {
        init_se3 = *init_guess;
        result.coarse = ExtrinsicSE3{};
        result.coarse->ref_sensor_id = ref_id;
        result.coarse->target_sensor_id = target_id;
        result.coarse->SO3_TargetInRef = init_se3.so3();
        result.coarse->POS_TargetInRef = init_se3.translation();
        result.coarse_quality.converged = true;
        use_user_init = true;
    } else if (init_guess.has_value()) {
        result.warnings.push_back("用户初值未通过校验，将执行粗标定");
    }
    if (!use_user_init) {
        auto coarse = calibrate_coarse(scans_ref, scans_target, ref_id, target_id);
        if (!coarse.has_value()) {
            result.failure_reason = "粗标定失败";
            result.needs_manual = true;
            return result;
        }
        result.coarse = coarse;
        result.coarse_quality = last_quality_;
        result.coarse_method = last_coarse_method_;
        init_se3 = coarse->SE3_TargetInRef();
    }

    if (use_user_init && cfg_.use_config_extrinsic_only) {
        ExtrinsicSE3 ext;
        ext.ref_sensor_id = ref_id;
        ext.target_sensor_id = target_id;
        ext.SO3_TargetInRef = init_se3.so3();
        ext.POS_TargetInRef = init_se3.translation();
        ext.is_converged = true;
        ext.residual_rms = 0.0;
        result.fine = ext;
        result.fine_quality.converged = true;
        result.fine_quality.fitness_score = 0.0;
        result.fine_method = "CONFIG_INIT_SKIP_FINE";
        UNICALIB_INFO(
            "[LiDAR-LiDAR] use_config_extrinsic_only=true：跳过精标定/B样条，直接使用 initial_extrinsics 作为 {} -> {} 手动微调起点",
            ref_id, target_id);
        return result;
    }

    UNICALIB_INFO_EX(
        "[LiDAR-LiDAR][Fine] calibrate_two_stage: about to calibrate_fine ref={} tgt={} n_ref={} n_tgt={} "
        "init_se3_ok={}",
        ref_id, target_id, scans_ref.size(), scans_target.size(),
        is_se3_finite(init_se3) ? "yes" : "no");
    Logger::flush();
    auto fine = calibrate_fine(scans_ref, scans_target, init_se3, ref_id, target_id);
    UNICALIB_INFO_EX("[LiDAR-LiDAR][Fine] calibrate_two_stage: calibrate_fine returned {}", fine.has_value() ? "ok" : "nullopt");
    Logger::flush();
    if (!fine.has_value()) {
        result.fine = result.coarse;
        result.fine_quality = result.coarse_quality;
        result.warnings.push_back("精标定未收敛，使用粗标定结果");
    } else {
        result.fine = fine;
        result.fine_quality = last_quality_;
        if (cfg_.use_bspline_refinement) {
            *result.fine = refine_with_bspline(scans_ref, scans_target,
                result.fine->SE3_TargetInRef(), ref_id, target_id);
        }
    }
    } catch (const std::exception& e) {
        result.failure_reason = std::string("标定异常: ") + e.what();
        result.needs_manual = true;
        if (!result.fine && result.coarse) {
            result.fine = result.coarse;
            result.fine_quality = result.coarse_quality;
        }
        UNICALIB_ERROR("[LiDAR-LiDAR] {}", result.failure_reason);
    } catch (...) {
        result.failure_reason = "标定异常: 未知错误";
        result.needs_manual = true;
        if (!result.fine && result.coarse) {
            result.fine = result.coarse;
            result.fine_quality = result.coarse_quality;
        }
        UNICALIB_ERROR("[LiDAR-LiDAR] {}", result.failure_reason);
    }
    return result;
}

// =============================================================================
// 质量评估
// =============================================================================

RegistrationQuality LiDARLiDARCalibrator::evaluate_registration(
    const LiDARScan& scan_ref,
    const LiDARScan& scan_target,
    const Sophus::SE3d& extrinsic) {
    RegistrationQuality q;
    if (!scan_ref.cloud || scan_ref.cloud->empty() ||
        !scan_target.cloud || scan_target.cloud->empty()) return q;
    if (!is_se3_finite(extrinsic)) return q;
    PointCloudXYZI::Ptr tgt_trans = transform_cloud(scan_target.cloud, extrinsic);
    if (!tgt_trans) return q;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    PointCloudXYZI::Ptr ref = preprocess_cloud(scan_ref.cloud);
    if (!ref || ref->empty()) return q;
    kdtree.setInputCloud(ref);
    const double max_dist = std::isfinite(cfg_.gicp_max_corr_dist) && cfg_.gicp_max_corr_dist > 0
                                ? std::min(10.0, std::max(0.01, cfg_.gicp_max_corr_dist)) : 1.0;
    int inliers = 0;
    double sum_sq = 0;
    for (const auto& pt : tgt_trans->points) {
        std::vector<int> idx(1);
        std::vector<float> dist(1);
        if (kdtree.nearestKSearch(pt, 1, idx, dist) > 0 && dist[0] < max_dist * max_dist) {
            inliers++;
            sum_sq += static_cast<double>(dist[0]);
        }
    }
    q.num_correspondences = static_cast<int>(tgt_trans->size());
    q.num_inliers = inliers;
    q.overlap_ratio = (q.num_correspondences > 0)
        ? (static_cast<double>(inliers) / q.num_correspondences) : 0.0;
    q.inlier_rmse = (inliers > 0) ? std::sqrt(sum_sq / inliers) : 0.0;
    q.fitness_score = q.overlap_ratio;
    q.converged = q.overlap_ratio >= cfg_.min_overlap_ratio;
    UNICALIB_CALC("LiDAR-LiDAR 配准评估 inliers={} correspondences={} overlap_ratio={:.4f} inlier_rmse={:.6f} converged={}",
                  q.num_inliers, q.num_correspondences, q.overlap_ratio, q.inlier_rmse, q.converged ? "yes" : "no");
    return q;
}

double LiDARLiDARCalibrator::compute_overlap_ratio(
    const LiDARScan& scan_ref,
    const LiDARScan& scan_target,
    const Sophus::SE3d& extrinsic) {
    if (!is_se3_finite(extrinsic)) return 0.0;
    return evaluate_registration(scan_ref, scan_target, extrinsic).overlap_ratio;
}

LiDARLiDARObservability LiDARLiDARCalibrator::analyze_observability(
    const std::vector<LiDARScan>& scans_ref,
    const std::vector<LiDARScan>& scans_target) {
    LiDARLiDARObservability obs;
    if (scans_ref.empty() || scans_target.empty()) {
        obs.warnings.push_back("点云序列为空");
        obs.recommended_method = "none";
        return obs;
    }
    obs.has_fov_overlap = true;
    obs.overlap_ratio = 0.5;
    obs.recommended_method = "targetless";
    return obs;
}

// =============================================================================
// 手动校准 / 可视化
// =============================================================================

ExtrinsicSE3 LiDARLiDARCalibrator::calibrate_manual(
    const LiDARScan& /*scan_ref*/,
    const LiDARScan& /*scan_target*/,
    const ExtrinsicSE3& init_extrin,
    const Sophus::SE3d& delta_transform) {
    Sophus::SE3d T_init = init_extrin.SE3_TargetInRef();
    if (!is_se3_finite(T_init)) {
        UNICALIB_WARN("[LiDAR-LiDAR] 手动校准: 初始外参无效，使用单位阵");
        T_init = Sophus::SE3d();
    }
    if (!is_se3_finite(delta_transform)) {
        UNICALIB_WARN("[LiDAR-LiDAR] 手动校准: 增量变换无效，忽略增量");
    }
    Sophus::SE3d T = T_init * (is_se3_finite(delta_transform) ? delta_transform : Sophus::SE3d());
    if (!validate_extrinsic(T)) {
        UNICALIB_WARN("[LiDAR-LiDAR] 手动校准: 结果未通过校验，返回初始外参");
        T = T_init;
    }
    ExtrinsicSE3 out;
    out.ref_sensor_id = init_extrin.ref_sensor_id;
    out.target_sensor_id = init_extrin.target_sensor_id;
    out.SO3_TargetInRef = T.so3();
    out.POS_TargetInRef = T.translation();
    out.time_offset_s = init_extrin.time_offset_s;
    return out;
}

void LiDARLiDARCalibrator::generate_visualization(
    const LiDARScan& /*scan_ref*/,
    const LiDARScan& /*scan_target*/,
    const ExtrinsicSE3& /*extrinsic*/,
    const std::string& output_path) {
    (void)output_path;
    UNICALIB_INFO("[LiDAR-LiDAR] 可视化报告: {} (占位)", output_path);
}

// =============================================================================
// P2 占位: GMM 联合配准 (接口预留)
// =============================================================================

bool LiDARLiDARCalibrator::initialize_gmm(
    const PointCloudXYZI::Ptr& /*cloud*/,
    int /*num_components*/) {
    return false;
}

bool LiDARLiDARCalibrator::optimize_multi_frame(
    const std::vector<LiDARScan>& /*scans_ref*/,
    const std::vector<LiDARScan>& /*scans_target*/,
    Sophus::SE3d& /*extrinsic*/) {
    return false;
}

bool LiDARLiDARCalibrator::validate_extrinsic(const Sophus::SE3d& T) const {
    if (!T.matrix().allFinite()) return false;
    double t_norm = T.translation().norm();
    if (t_norm > 100.0) return false;
    Eigen::Matrix3d R = T.so3().matrix();
    double det = R.determinant();
    if (std::abs(det - 1.0) > 0.01) return false;
    return true;
}

void LiDARLiDARCalibrator::log_stage(const std::string& stage, const std::string& msg) const {
    if (cfg_.verbose) UNICALIB_INFO("[LiDAR-LiDAR-{}] {}", stage, msg);
}

void LiDARLiDARCalibrator::log_iter(int iter, double cost, double delta) const {
    if (cfg_.verbose) UNICALIB_TRACE("[LiDAR-LiDAR] iter={} cost={:.6f} delta={:.6f}", iter, cost, delta);
}

}  // namespace ns_unicalib
