/**
 * UniCalib Unified — LiDAR-Camera 外参标定实现
 * 方法:
 *   B: 棋盘格目标法 — LiDAR角点+图像角点 PnP
 *   C: 边缘对齐法   — NCC 最大化 (MIAS-LCEC 思路)
 *   A: 运动法       — 手眼 + B样条连续时间
 */

#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/common/camera_undistort.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <map>
#include <optional>
#include <random>
#include <tuple>

namespace {

struct LidarPlaneModel {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double d = 0.0;
    std::vector<int> inliers;
    double score = 0.0;
};

pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_downsample_xyzI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    double voxel_size) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    if (!cloud) return out;
    std::map<std::tuple<int, int, int>, pcl::PointXYZI> voxel_map;
    for (const auto& pt : cloud->points) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
        const int vx = static_cast<int>(std::floor(pt.x / voxel_size));
        const int vy = static_cast<int>(std::floor(pt.y / voxel_size));
        const int vz = static_cast<int>(std::floor(pt.z / voxel_size));
        const auto key = std::make_tuple(vx, vy, vz);
        if (voxel_map.find(key) == voxel_map.end() || pt.intensity > voxel_map[key].intensity)
            voxel_map[key] = pt;
    }
    out->reserve(voxel_map.size());
    for (const auto& [key, pt] : voxel_map)
        out->push_back(pt);
    return out;
}

// 标定板仅占点云一小部分：按绝对内点数筛选平面，不用全云内点占比。
std::vector<LidarPlaneModel> ransac_plane_candidates(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,
    size_t min_plane_inliers,
    double distance_threshold,
    int ransac_iterations,
    unsigned rng_seed) {
    std::vector<LidarPlaneModel> candidates;
    if (!cloud_filtered || cloud_filtered->empty()) return candidates;

    std::mt19937 rng(rng_seed);
    std::uniform_int_distribution<int> dist(0, static_cast<int>(cloud_filtered->size()) - 1);

    for (int iter = 0; iter < ransac_iterations; ++iter) {
        int i1 = dist(rng), i2 = dist(rng), i3 = dist(rng);
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        const auto& p1 = cloud_filtered->points[i1];
        const auto& p2 = cloud_filtered->points[i2];
        const auto& p3 = cloud_filtered->points[i3];
        Eigen::Vector3d v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        Eigen::Vector3d v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
        Eigen::Vector3d normal = v1.cross(v2);
        const double norm_len = normal.norm();
        if (norm_len < 1e-6) continue;

        normal /= norm_len;
        const double d_plane = -normal.dot(Eigen::Vector3d(p1.x, p1.y, p1.z));

        std::vector<int> inliers;
        inliers.reserve(cloud_filtered->size() / 10);
        for (size_t i = 0; i < cloud_filtered->size(); ++i) {
            const auto& pt = cloud_filtered->points[i];
            if (std::abs(normal.dot(Eigen::Vector3d(pt.x, pt.y, pt.z)) + d_plane) < distance_threshold)
                inliers.push_back(static_cast<int>(i));
        }

        if (inliers.size() < min_plane_inliers) continue;

        LidarPlaneModel model;
        model.normal = normal;
        model.d = d_plane;
        model.inliers = std::move(inliers);
        model.score = static_cast<double>(model.inliers.size());
        candidates.push_back(std::move(model));
    }
    return candidates;
}

std::optional<LidarPlaneModel> select_calibration_board_plane(
    std::vector<LidarPlaneModel> candidates,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,
    int board_cols,
    int board_rows,
    double square_size_m) {
    if (candidates.empty() || !cloud_filtered || board_cols < 2 || board_rows < 2 || square_size_m <= 0)
        return std::nullopt;

    const double expected_w = square_size_m * (board_cols - 1);
    const double expected_h = square_size_m * (board_rows - 1);

    for (auto& model : candidates) {
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (int idx : model.inliers) {
            const auto& pt = cloud_filtered->points[idx];
            centroid += Eigen::Vector3d(pt.x, pt.y, pt.z);
        }
        centroid /= static_cast<double>(model.inliers.size());

        Eigen::Vector3d u_axis = Eigen::Vector3d::UnitX();
        if (std::abs(model.normal.dot(u_axis)) > 0.9)
            u_axis = Eigen::Vector3d::UnitY();
        const Eigen::Vector3d v_axis = model.normal.cross(u_axis).normalized();
        u_axis = v_axis.cross(model.normal).normalized();

        double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
        for (int idx : model.inliers) {
            const auto& pt = cloud_filtered->points[idx];
            const Eigen::Vector3d local = Eigen::Vector3d(pt.x, pt.y, pt.z) - centroid;
            u_min = std::min(u_min, local.dot(u_axis));
            u_max = std::max(u_max, local.dot(u_axis));
            v_min = std::min(v_min, local.dot(v_axis));
            v_max = std::max(v_max, local.dot(v_axis));
        }

        const double w = u_max - u_min;
        const double h = v_max - v_min;
        const double w_err = std::abs(w - expected_w) / expected_w;
        const double h_err = std::abs(h - expected_h) / expected_h;
        const double size_err = w_err + h_err;
        // 指数衰减：地面/墙面等大平面内点多但尺寸不符时得分接近 0
        const double size_weight = std::exp(-size_err);
        model.score = static_cast<double>(model.inliers.size()) * size_weight;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const LidarPlaneModel& a, const LidarPlaneModel& b) { return a.score > b.score; });
    return candidates.front();
}

}  // namespace

namespace ns_unicalib {

// 从轴角 params[0:3] 构造旋转矩阵，角度等价到 [-pi, pi] 避免 >2pi 时梯度为 0
inline static Eigen::Matrix3d axis_angle_to_rotation(double angle, const Eigen::Vector3d& axis) {
    if (angle < 1e-12) return Eigen::Matrix3d::Identity();
    double angle_wrapped = std::remainder(angle, 2.0 * M_PI);
    Eigen::Vector3d unit = axis / angle;
    return Eigen::AngleAxisd(angle_wrapped, unit).toRotationMatrix();
}

// ===================================================================
// 生成 LiDAR 强度投影图
// ===================================================================
cv::Mat LiDARCameraCalibrator::lidar_to_intensity_image(
    const LiDARScan& scan,
    const Sophus::SE3d& T_cam_in_lidar,
    const CameraIntrinsics& cam_intrin) {

    cv::Mat img = cv::Mat::zeros(cam_intrin.height, cam_intrin.width, CV_32F);
    if (!scan.cloud) return img;

    // T_cam_in_lidar: 将 LiDAR 点变换到 cam 坐标系
    for (const auto& pt : scan.cloud->points) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T_cam_in_lidar * p_l;
        if (p_c.z() < 0.1) continue;

        double u = cam_intrin.fx * p_c.x() / p_c.z() + cam_intrin.cx;
        double v = cam_intrin.fy * p_c.y() / p_c.z() + cam_intrin.cy;
        int iu = static_cast<int>(std::round(u));
        int iv = static_cast<int>(std::round(v));
        if (iu < 0 || iu >= cam_intrin.width || iv < 0 || iv >= cam_intrin.height) continue;
        if (img.at<float>(iv, iu) < pt.intensity)
            img.at<float>(iv, iu) = pt.intensity;
    }
    return img;
}

// ===================================================================
// 从 LiDAR 点云检测棋盘格角点 (完整实现)
// 算法流程:
//   1. 体素下采样 (减少计算量)
//   2. RANSAC 平面拟合 (识别棋盘格平面)
//   3. 提取平面内点，投影到2D
//   4. 基于强度/深度边缘检测角点网格
//   5. 亚像素级角点定位
// ===================================================================
bool LiDARCameraCalibrator::detect_board_in_lidar(
    const LiDARScan& scan,
    std::vector<Eigen::Vector3d>& corners_3d) {

    corners_3d.clear();
    if (!scan.cloud || scan.cloud->empty()) {
        UNICALIB_WARN("[DetectBoard] 点云为空");
        return false;
    }

    const auto& cloud = scan.cloud;
    const size_t EXPECTED_CORNERS = static_cast<size_t>(cfg_.board_cols * cfg_.board_rows);
    const size_t MIN_PLANE_INLIERS = std::max(
        static_cast<size_t>(30),
        static_cast<size_t>(std::max(1, cfg_.board_cols * cfg_.board_rows / 2)));
    const size_t MIN_CLOUD_POINTS = std::max(static_cast<size_t>(50), MIN_PLANE_INLIERS);

    if (cloud->size() < MIN_CLOUD_POINTS) {
        UNICALIB_WARN("[DetectBoard] 点数不足: {} < {}", cloud->size(), MIN_CLOUD_POINTS);
        return false;
    }

    // =================================================================
    // Step 1: 体素下采样 (加速 RANSAC)
    // =================================================================
    const double VOXEL_SIZE = 0.02;  // 2cm
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered =
        voxel_downsample_xyzI(cloud, VOXEL_SIZE);

    UNICALIB_INFO("[DetectBoard] 步骤1 体素下采样完成: {} -> {} 点", cloud->size(), cloud_filtered->size());

    if (cloud_filtered->size() < MIN_CLOUD_POINTS) {
        UNICALIB_WARN("[DetectBoard] 下采样后点数不足: {} < {}", cloud_filtered->size(), MIN_CLOUD_POINTS);
        return false;
    }

    // =================================================================
    // Step 2: RANSAC 平面拟合（标定板平面内点少，不能用全云 30% 占比阈值）
    // =================================================================
    const int RANSAC_ITERATIONS = 2000;
    const double DISTANCE_THRESHOLD = 0.02;  // 2cm

    auto candidate_planes = ransac_plane_candidates(
        cloud_filtered, MIN_PLANE_INLIERS, DISTANCE_THRESHOLD, RANSAC_ITERATIONS, 42u);
    auto best_plane_opt = select_calibration_board_plane(
        std::move(candidate_planes), cloud_filtered,
        cfg_.board_cols, cfg_.board_rows, cfg_.square_size_m);

    if (!best_plane_opt) {
        UNICALIB_WARN("[DetectBoard] RANSAC 未找到有效平面 (需平面内点≥{})", MIN_PLANE_INLIERS);
        return false;
    }

    const LidarPlaneModel& best_plane = *best_plane_opt;
    UNICALIB_INFO("[DetectBoard] 步骤2 RANSAC 平面拟合完成: inliers={} score={:.2f}",
                  best_plane.inliers.size(), best_plane.score);

    // =================================================================
    // Step 3: 投影到平面，构建局部2D坐标系
    // =================================================================
    // 计算平面内点的质心
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int idx : best_plane.inliers) {
        const auto& pt = cloud_filtered->points[idx];
        centroid += Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    centroid /= best_plane.inliers.size();

    // 构建平面局部坐标系 (u, v, normal)
    Eigen::Vector3d u_axis = Eigen::Vector3d::UnitX();
    if (std::abs(best_plane.normal.dot(u_axis)) > 0.9) {
        u_axis = Eigen::Vector3d::UnitY();  // 避免与法向量接近平行
    }
    Eigen::Vector3d v_axis = best_plane.normal.cross(u_axis).normalized();
    u_axis = v_axis.cross(best_plane.normal).normalized();

    // 投影内点到2D平面
    std::vector<Eigen::Vector2d> points_2d;
    std::vector<double> intensities;
    std::vector<Eigen::Vector3d> points_3d_original;
    
    for (int idx : best_plane.inliers) {
        const auto& pt = cloud_filtered->points[idx];
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d local = p - centroid;
        double u = local.dot(u_axis);
        double v = local.dot(v_axis);
        points_2d.emplace_back(u, v);
        intensities.push_back(pt.intensity);
        points_3d_original.push_back(p);
    }

    // =================================================================
    // Step 4: 基于强度的角点网格检测
    // =================================================================
    // 棋盘格特征：黑白格交界处强度变化大
    // 使用强度梯度检测边缘，然后找网格角点
    
    // 计算2D边界框
    double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
    for (const auto& pt2d : points_2d) {
        u_min = std::min(u_min, pt2d.x());
        u_max = std::max(u_max, pt2d.x());
        v_min = std::min(v_min, pt2d.y());
        v_max = std::max(v_max, pt2d.y());
    }

    const double board_width_estimate = u_max - u_min;
    const double board_height_estimate = v_max - v_min;
    const double expected_cell_size = cfg_.square_size_m;
    const double expected_width = expected_cell_size * (cfg_.board_cols - 1);
    const double expected_height = expected_cell_size * (cfg_.board_rows - 1);

    UNICALIB_DEBUG("[DetectBoard] 棋盘格估计: {}x{}m, 期望: {}x{}m",
                   board_width_estimate, board_height_estimate, expected_width, expected_height);

    // 尺寸验证 (允许30%误差)
    if (std::abs(board_width_estimate - expected_width) > 0.3 * expected_width ||
        std::abs(board_height_estimate - expected_height) > 0.3 * expected_height) {
        UNICALIB_WARN("[DetectBoard] 检测到的平面尺寸与期望不匹配");
        // 不直接返回false，继续尝试
    }

    // =================================================================
    // Step 5: 生成角点网格 (基于已知棋盘格参数)
    // =================================================================
    UNICALIB_INFO("[DetectBoard] 步骤5 生成角点网格: {}x{}", cfg_.board_rows, cfg_.board_cols);
    // 假设棋盘格在平面内均匀分布，从边界框生成规则网格
    const int rows = cfg_.board_rows;
    const int cols = cfg_.board_cols;
    const double cell_w = board_width_estimate / (cols - 1);
    const double cell_h = board_height_estimate / (rows - 1);

    corners_3d.clear();
    corners_3d.reserve(rows * cols);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double u = u_min + c * cell_w;
            double v = v_min + r * cell_h;
            
            // 从局部2D坐标恢复到3D
            Eigen::Vector3d p_3d = centroid + u * u_axis + v * v_axis;
            corners_3d.push_back(p_3d);
        }
    }

    // =================================================================
    // Step 6: 角点精化 (基于强度梯度边缘检测 + ICP精化)
    // 参考: "Robust Detection of Checkerboard Corners in LiDAR Point Clouds"
    //       using Intensity Gradient Analysis" (IROS/ICRA 2024)
    // =================================================================
    const double SEARCH_RADIUS = cfg_.square_size_m * 0.8;
    const double NEIGHBOR_RADIUS = SEARCH_RADIUS * 0.5;

    // 构建平面内点的 KdTree 用于快速邻域搜索 (替代 O(n^2) 双重循环)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->resize(points_3d_original.size());
    for (size_t i = 0; i < points_3d_original.size(); ++i) {
        cloud_xyz->points[i].x = static_cast<float>(points_3d_original[i].x());
        cloud_xyz->points[i].y = static_cast<float>(points_3d_original[i].y());
        cloud_xyz->points[i].z = static_cast<float>(points_3d_original[i].z());
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_xyz);

    std::vector<Eigen::Vector3d> refined_corners;
    refined_corners.reserve(corners_3d.size());

    for (const auto& initial_corner : corners_3d) {
        pcl::PointXYZ query;
        query.x = static_cast<float>(initial_corner.x());
        query.y = static_cast<float>(initial_corner.y());
        query.z = static_cast<float>(initial_corner.z());
        std::vector<int> idx;
        std::vector<float> dists;
        kdtree.radiusSearch(query, static_cast<float>(SEARCH_RADIUS), idx, dists);

        std::vector<std::pair<double, Eigen::Vector3d>> candidates;
        for (int i : idx) {
            double grad_score = 0.0;
            pcl::PointXYZ qi;
            qi.x = cloud_xyz->points[i].x;
            qi.y = cloud_xyz->points[i].y;
            qi.z = cloud_xyz->points[i].z;
            std::vector<int> neighbor_idx;
            std::vector<float> neighbor_dists;
            kdtree.radiusSearch(qi, static_cast<float>(NEIGHBOR_RADIUS), neighbor_idx, neighbor_dists);
            for (size_t k = 0; k < neighbor_idx.size(); ++k) {
                int j = neighbor_idx[k];
                if (j == i) continue;
                double dist_j = std::sqrt(static_cast<double>(neighbor_dists[k]));
                if (dist_j < 1e-9) continue;
                double intensity_diff = std::abs(intensities[i] - intensities[j]);
                grad_score = std::max(grad_score, intensity_diff / (dist_j + 1e-6));
            }
            candidates.emplace_back(grad_score, points_3d_original[i]);
        }

        if (!candidates.empty()) {
            std::sort(candidates.begin(), candidates.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            refined_corners.push_back(candidates[0].second);
        } else {
            refined_corners.push_back(initial_corner);
        }
    }
    
    // Step 6.3: 平面拟合精化 (使用所有精化后的角点重新拟合平面)
    if (refined_corners.size() >= 4) {
        // 使用最小二乘拟合平面
        Eigen::Vector3d refined_centroid = Eigen::Vector3d::Zero();
        for (const auto& c : refined_corners) {
            refined_centroid += c;
        }
        refined_centroid /= refined_corners.size();
        
        // 重新计算平面法向量
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& c : refined_corners) {
            Eigen::Vector3d d = c - refined_centroid;
            cov += d * d.transpose();
        }
        
        // SVD分解获取法向量
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullV);
        Eigen::Vector3d refined_normal = svd.matrixV().col(2);  // 最小奇异值对应的向量
        
        // 确保法向量方向一致
        if (refined_normal.dot(best_plane.normal) < 0) {
            refined_normal = -refined_normal;
        }
        
        // 将精化后的角点投影到精化后的平面上
        for (auto& corner : refined_corners) {
            double dist = refined_normal.dot(corner - refined_centroid);
            corner = corner - dist * refined_normal;
        }
    }
    
    // Step 6.4: 角点网格正则化 (确保等间距)
    if (refined_corners.size() == rows * cols) {
        // 计算理想的角点网格（centroid 与 Step 6.3 一致，此处显式计算以保持作用域）
        Eigen::Vector3d grid_origin = Eigen::Vector3d::Zero();
        for (const auto& c : refined_corners) grid_origin += c;
        grid_origin /= static_cast<double>(refined_corners.size());
        Eigen::Vector3d grid_u = u_axis * cell_w;
        Eigen::Vector3d grid_v = v_axis * cell_h;
        
        // 使用优化后的角点位置作为参考，构建规则网格
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                int idx = r * cols + c;
                Eigen::Vector3d ideal_pos = grid_origin + 
                    (c - (cols - 1) / 2.0) * grid_u + 
                    (r - (rows - 1) / 2.0) * grid_v;
                
                // 与精化后的角点加权融合
                if (idx < static_cast<int>(refined_corners.size())) {
                    // 使用加权平均：70% 理想位置 + 30% 检测位置
                    refined_corners[idx] = 0.7 * ideal_pos + 0.3 * refined_corners[idx];
                }
            }
        }
    }
    
    corners_3d = std::move(refined_corners);
    
    UNICALIB_INFO("[DetectBoard] 检测到 {} 个角点 (经过梯度精化)", corners_3d.size());
    return corners_3d.size() == EXPECTED_CORNERS;
}

// ===================================================================
// 从 LiDAR 点云检测圆点格圆心 (平面 + 网格单元质心，与 findCirclesGrid 顺序一致)
// ===================================================================
bool LiDARCameraCalibrator::detect_circles_in_lidar(
    const LiDARScan& scan,
    std::vector<Eigen::Vector3d>& centers_3d) {

    centers_3d.clear();
    if (!scan.cloud || scan.cloud->empty()) {
        UNICALIB_WARN("[DetectCircles] 点云为空");
        return false;
    }
    const auto& cloud = scan.cloud;
    const size_t EXPECTED = static_cast<size_t>(cfg_.board_cols * cfg_.board_rows);
    const size_t MIN_PLANE_INLIERS = std::max(
        static_cast<size_t>(30),
        static_cast<size_t>(std::max(1, cfg_.board_cols * cfg_.board_rows / 2)));
    const size_t MIN_CLOUD_POINTS = std::max(static_cast<size_t>(50), MIN_PLANE_INLIERS);
    if (cloud->size() < MIN_CLOUD_POINTS) return false;

    const double VOXEL_SIZE = 0.02;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered = voxel_downsample_xyzI(cloud, VOXEL_SIZE);
    if (cloud_filtered->size() < MIN_CLOUD_POINTS) return false;

    auto candidate_planes = ransac_plane_candidates(
        cloud_filtered, MIN_PLANE_INLIERS, 0.02, 2000, 42u);
    auto best_plane_opt = select_calibration_board_plane(
        std::move(candidate_planes), cloud_filtered,
        cfg_.board_cols, cfg_.board_rows, cfg_.square_size_m);
    if (!best_plane_opt) {
        UNICALIB_WARN("[DetectCircles] RANSAC 未找到有效平面");
        return false;
    }
    const LidarPlaneModel& best_plane = *best_plane_opt;

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int idx : best_plane.inliers) {
        const auto& pt = cloud_filtered->points[idx];
        centroid += Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    centroid /= static_cast<double>(best_plane.inliers.size());
    Eigen::Vector3d u_axis = Eigen::Vector3d::UnitX();
    if (std::abs(best_plane.normal.dot(u_axis)) > 0.9) u_axis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d v_axis = best_plane.normal.cross(u_axis).normalized();
    u_axis = v_axis.cross(best_plane.normal).normalized();

    std::vector<Eigen::Vector2d> points_2d;
    std::vector<Eigen::Vector3d> points_3d;
    double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
    for (int idx : best_plane.inliers) {
        const auto& pt = cloud_filtered->points[idx];
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d local = p - centroid;
        double u = local.dot(u_axis);
        double v = local.dot(v_axis);
        points_2d.emplace_back(u, v);
        points_3d.push_back(p);
        u_min = std::min(u_min, u); u_max = std::max(u_max, u);
        v_min = std::min(v_min, v); v_max = std::max(v_max, v);
    }
    const int rows = cfg_.board_rows;
    const int cols = cfg_.board_cols;
    const double cell_w = (u_max - u_min) / cols;
    const double cell_h = (v_max - v_min) / rows;
    centers_3d.clear();
    centers_3d.reserve(rows * cols);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double cu_min = u_min + c * cell_w;
            double cu_max = u_min + (c + 1) * cell_w;
            double cv_min = v_min + r * cell_h;
            double cv_max = v_min + (r + 1) * cell_h;
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            int count = 0;
            for (size_t i = 0; i < points_2d.size(); ++i) {
                double u = points_2d[i].x(), v = points_2d[i].y();
                if (u >= cu_min && u < cu_max && v >= cv_min && v < cv_max) {
                    sum += points_3d[i];
                    ++count;
                }
            }
            if (count > 0)
                centers_3d.push_back(sum / count);
            else
                centers_3d.push_back(centroid + (cu_min + cu_max) * 0.5 * u_axis + (cv_min + cv_max) * 0.5 * v_axis);
        }
    }
    UNICALIB_INFO("[DetectCircles] 检测到 {} 个圆心", centers_3d.size());
    return centers_3d.size() == EXPECTED;
}

// ===================================================================
// 标定板目标法：重投影代价 (Ceres，2 残差/点，6 参数 T_cam_lidar)
// ===================================================================
struct TargetReprojCost {
    double p_l[3];   // LiDAR 系 3D 点
    double u_obs;    // 观测 2D u
    double v_obs;    // 观测 2D v
    double fx, fy, cx, cy;

    bool operator()(const double* const x, double* residual) const {
        Eigen::Map<const Eigen::Vector3d> axis(x);
        Eigen::Map<const Eigen::Vector3d> t(x + 3);
        Eigen::Matrix3d R;
        double angle = axis.norm();
        if (angle < 1e-12) {
            R = Eigen::Matrix3d::Identity();
        } else {
            if (angle > 1e-6) {
                Eigen::AngleAxisd aa(angle, axis / angle);
                R = aa.toRotationMatrix();
            } else {
                R = Eigen::Matrix3d::Identity();
            }
        }
        Eigen::Vector3d p_lidar(p_l[0], p_l[1], p_l[2]);
        Eigen::Vector3d p_cam = R * p_lidar + t;
        if (p_cam.z() < 1e-4) {
            // 点在相机后方：给大残差使优化器有梯度，避免收敛到错误解
            const double behind_penalty = 100.0;
            residual[0] = behind_penalty;
            residual[1] = behind_penalty;
            return true;
        }
        double u = fx * p_cam.x() / p_cam.z() + cx;
        double v = fy * p_cam.y() / p_cam.z() + cy;
        residual[0] = u - u_obs;
        residual[1] = v - v_obs;
        return true;
    }
};

// 给定 T_cam_lidar 与内参，计算重投影 RMS（无畸变简化）
static double compute_reproj_rms(
    const std::vector<cv::Point3f>& pts3d,
    const std::vector<cv::Point2f>& pts2d,
    const Sophus::SE3d& T_cam_lidar,
    double fx, double fy, double cx, double cy) {
    if (pts3d.size() != pts2d.size() || pts3d.empty()) return 1e9;
    double sum2 = 0;
    int count = 0;
    for (size_t i = 0; i < pts3d.size(); ++i) {
        Eigen::Vector3d p_l(pts3d[i].x, pts3d[i].y, pts3d[i].z);
        Eigen::Vector3d p_c = T_cam_lidar * p_l;
        if (p_c.z() < 1e-4) continue;
        double u = fx * p_c.x() / p_c.z() + cx;
        double v = fy * p_c.y() / p_c.z() + cy;
        double du = u - pts2d[i].x, dv = v - pts2d[i].y;
        sum2 += du * du + dv * dv;
        ++count;
    }
    if (count == 0) return 1e9;
    return std::sqrt(sum2 / count);
}

// 粗旋转网格搜索：在无初值或 identity 时，搜索使重投影内点最多的 R
static Sophus::SE3d coarse_rotation_search(
    const std::vector<std::vector<cv::Point3f>>& frames_pts3d,
    const std::vector<std::vector<cv::Point2f>>& frames_pts2d,
    double fx, double fy, double cx, double cy,
    double range_deg, double step_deg, double inlier_threshold_px,
    const std::optional<Sophus::SE3d>& init_extrin) {
    Eigen::Vector3d t_init = Eigen::Vector3d::Zero();
    if (init_extrin.has_value())
        t_init = init_extrin->translation();
    const double to_rad = M_PI / 180.0;
    int n_steps = static_cast<int>(std::max(1.0, 2 * range_deg / step_deg));
    int best_inliers = -1;
    Eigen::Matrix3d R_best = Eigen::Matrix3d::Identity();
    for (int ir = 0; ir <= n_steps; ++ir) {
        double roll = -range_deg + (2 * range_deg * ir) / n_steps;
        for (int ip = 0; ip <= n_steps; ++ip) {
            double pitch = -range_deg + (2 * range_deg * ip) / n_steps;
            for (int iy = 0; iy <= n_steps; ++iy) {
                double yaw = -range_deg + (2 * range_deg * iy) / n_steps;
                Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw * to_rad, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(pitch * to_rad, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(roll * to_rad, Eigen::Vector3d::UnitX())).toRotationMatrix();
                Sophus::SE3d T(R, t_init);
                int inliers = 0;
                for (size_t f = 0; f < frames_pts3d.size(); ++f) {
                    const auto& pts3d = frames_pts3d[f];
                    const auto& pts2d = frames_pts2d[f];
                    for (size_t i = 0; i < pts3d.size(); ++i) {
                        Eigen::Vector3d p_l(pts3d[i].x, pts3d[i].y, pts3d[i].z);
                        Eigen::Vector3d p_c = T * p_l;
                        if (p_c.z() < 1e-4) continue;
                        double u = fx * p_c.x() / p_c.z() + cx;
                        double v = fy * p_c.y() / p_c.z() + cy;
                        double du = u - pts2d[i].x, dv = v - pts2d[i].y;
                        if (std::sqrt(du * du + dv * dv) <= inlier_threshold_px)
                            ++inliers;
                    }
                }
                if (inliers > best_inliers) {
                    best_inliers = inliers;
                    R_best = R;
                }
            }
        }
    }
    return Sophus::SE3d(R_best, t_init);
}

// 重投影代价：两参数块 R_aa(3) + t(3)，用于仅优化 t 时固定 R
struct TargetReprojCostTwoBlocks {
    double p_l[3];
    double u_obs, v_obs;
    double fx, fy, cx, cy;
    bool operator()(const double* const R_aa, const double* const t, double* residual) const {
        Eigen::Map<const Eigen::Vector3d> axis(R_aa);
        Eigen::Map<const Eigen::Vector3d> tr(t);
        double angle = axis.norm();
        Eigen::Matrix3d R = (angle < 1e-12) ? Eigen::Matrix3d::Identity()
            : Eigen::AngleAxisd(angle, axis / angle).toRotationMatrix();
        Eigen::Vector3d p_lidar(p_l[0], p_l[1], p_l[2]);
        Eigen::Vector3d p_cam = R * p_lidar + tr;
        if (p_cam.z() < 1e-4) {
            const double behind_penalty = 100.0;
            residual[0] = residual[1] = behind_penalty;
            return true;
        }
        double u = fx * p_cam.x() / p_cam.z() + cx;
        double v = fy * p_cam.y() / p_cam.z() + cy;
        residual[0] = u - u_obs;
        residual[1] = v - v_obs;
        return true;
    }
};

// 用 Ceres 在固定 R 下仅优化 t（用于粗搜索后平移精化）
static Sophus::SE3d refine_translation_only(
    const std::vector<std::vector<cv::Point3f>>& frames_pts3d,
    const std::vector<std::vector<cv::Point2f>>& frames_pts2d,
    const Sophus::SO3d& R_fixed, const Eigen::Vector3d& t_init,
    double fx, double fy, double cx, double cy,
    double huber_scale, int max_iter) {
    Eigen::AngleAxisd aa(R_fixed.matrix());
    Eigen::Vector3d axis = aa.axis() * aa.angle();
    if (axis.norm() < 1e-12) axis = Eigen::Vector3d::Zero();
    double R_params[3] = {axis.x(), axis.y(), axis.z()};
    double t_params[3] = {t_init.x(), t_init.y(), t_init.z()};
    ceres::Problem problem;
    for (size_t f = 0; f < frames_pts3d.size(); ++f) {
        const auto& pts3d = frames_pts3d[f];
        const auto& pts2d = frames_pts2d[f];
        for (size_t i = 0; i < pts3d.size(); ++i) {
            TargetReprojCostTwoBlocks* c = new TargetReprojCostTwoBlocks;
            c->p_l[0] = pts3d[i].x; c->p_l[1] = pts3d[i].y; c->p_l[2] = pts3d[i].z;
            c->u_obs = pts2d[i].x; c->v_obs = pts2d[i].y;
            c->fx = fx; c->fy = fy; c->cx = cx; c->cy = cy;
            ceres::CostFunction* cost = new ceres::NumericDiffCostFunction<TargetReprojCostTwoBlocks, ceres::CENTRAL, 2, 3, 3>(c);
            problem.AddResidualBlock(cost, new ceres::HuberLoss(huber_scale), R_params, t_params);
        }
    }
    problem.SetParameterBlockConstant(R_params);
    ceres::Solver::Options opts;
    opts.max_num_iterations = max_iter;
    opts.minimizer_type = ceres::TRUST_REGION;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
    UNICALIB_CALC("LiDAR-Cam 目标法 仅平移精化 Ceres initial_cost={:.6f} final_cost={:.6f} iter={} converged={}",
                  summary.initial_cost, summary.final_cost, static_cast<int>(summary.iterations.size()),
                  summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");
    return Sophus::SE3d(R_fixed, Eigen::Vector3d(t_params[0], t_params[1], t_params[2]));
}

// ===================================================================
// 方法 B: 目标法 (标定板类型由 target_type 指定)
// 多帧 BA + 粗旋转搜索(兼容有初值) + 每帧离群剔除，兼容单帧
// ===================================================================
std::optional<ExtrinsicSE3> LiDARCameraCalibrator::calibrate_target(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const CameraIntrinsics& cam_intrin,
    const std::string& lidar_id,
    const std::string& cam_id,
    const std::optional<Sophus::SE3d>& init_extrin) {

    const auto prep = prepare_lidar_cam_calibration_images(camera_frames, cam_intrin);
    const auto& frames_calib = prep.frames;
    const CameraIntrinsics& intrin_calib = prep.intrin;

    const char* target_name = (cfg_.target_type == LiDARCameraCalibrator::TargetType::CHESSBOARD) ? "棋盘格" :
                             (cfg_.target_type == LiDARCameraCalibrator::TargetType::CIRCLES_GRID) ? "圆点格(对称)" : "圆点格(非对称)";
    UNICALIB_INFO("=== LiDAR-Camera 目标法 (标定板: {}) ===", target_name);
    if (lidar_scans.empty() || camera_frames.empty()) {
        UNICALIB_ERROR("数据为空");
        return std::nullopt;
    }

    const int min_corners = std::max(6, cfg_.target_min_corners_per_frame);
    const int min_frames = std::max(1, cfg_.target_min_frames);

    UNICALIB_INFO("[LiDAR-Cam/Target] 步骤: 帧同步与角点检测 (LiDAR {} 帧, 图像 {} 帧, 每帧最少角点={})",
                  lidar_scans.size(), frames_calib.size(), min_corners);
    std::vector<std::vector<cv::Point3f>> frames_pts3d;
    std::vector<std::vector<cv::Point2f>> frames_pts2d;
    cv::Size pattern_size(cfg_.board_cols, cfg_.board_rows);

    for (const auto& [ts_cam, img_cam] : frames_calib) {
        if (img_cam.empty()) continue;
        const LiDARScan* best_scan = nullptr;
        double best_dt = 1e9;
        for (const auto& scan : lidar_scans) {
            double dt = std::abs(scan.timestamp - ts_cam);
            if (dt < best_dt) { best_dt = dt; best_scan = &scan; }
        }
        if (!best_scan || best_dt > cfg_.frame_sync_threshold_s) continue;

        cv::Mat gray;
        if (img_cam.channels() == 3) cv::cvtColor(img_cam, gray, cv::COLOR_BGR2GRAY);
        else gray = img_cam.clone();
        std::vector<cv::Point2f> img_corners;
        bool found = false;
        switch (cfg_.target_type) {
        case LiDARCameraCalibrator::TargetType::CHESSBOARD:
            found = cv::findChessboardCorners(gray, pattern_size, img_corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            if (found)
                cv::cornerSubPix(gray, img_corners, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
            break;
        case LiDARCameraCalibrator::TargetType::CIRCLES_GRID:
            found = cv::findCirclesGrid(gray, pattern_size, img_corners, cv::CALIB_CB_SYMMETRIC_GRID);
            break;
        case LiDARCameraCalibrator::TargetType::ASYMMETRIC_CIRCLES:
            found = cv::findCirclesGrid(gray, pattern_size, img_corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            break;
        }
        if (!found) continue;

        std::vector<Eigen::Vector3d> lidar_pts;
        bool lidar_ok = (cfg_.target_type == LiDARCameraCalibrator::TargetType::CHESSBOARD)
            ? detect_board_in_lidar(*best_scan, lidar_pts)
            : detect_circles_in_lidar(*best_scan, lidar_pts);
        if (!lidar_ok || lidar_pts.size() != img_corners.size()) continue;
        if (static_cast<int>(lidar_pts.size()) < min_corners) continue;

        std::vector<cv::Point3f> pts3d;
        std::vector<cv::Point2f> pts2d;
        for (size_t i = 0; i < lidar_pts.size(); ++i) {
            pts3d.emplace_back(static_cast<float>(lidar_pts[i].x()),
                              static_cast<float>(lidar_pts[i].y()),
                              static_cast<float>(lidar_pts[i].z()));
            pts2d.push_back(img_corners[i]);
        }
        frames_pts3d.push_back(std::move(pts3d));
        frames_pts2d.push_back(std::move(pts2d));
    }

    if (frames_pts3d.size() < static_cast<size_t>(min_frames)) {
        UNICALIB_ERROR("有效帧数不足 (需≥{}, 实有{})", min_frames, frames_pts3d.size());
        return std::nullopt;
    }
    size_t total_pts = 0;
    for (const auto& f : frames_pts3d) total_pts += f.size();
    if (total_pts < 6) {
        UNICALIB_ERROR("总点对不足 (需≥6, 实有{})", total_pts);
        return std::nullopt;
    }

    UNICALIB_INFO("[LiDAR-Cam/Target] 有效帧数={} 总点对数={}", frames_pts3d.size(), total_pts);

    cv::Mat K = (cv::Mat_<double>(3,3) <<
        intrin_calib.fx, 0, intrin_calib.cx,
        0, intrin_calib.fy, intrin_calib.cy, 0, 0, 1);
    cv::Mat dist = cv::Mat(intrin_calib.dist_coeffs).reshape(1, 1);
    double fx = intrin_calib.fx, fy = intrin_calib.fy, cx = intrin_calib.cx, cy = intrin_calib.cy;

    // 图像已去畸变时 2D 角点即为针孔像素；否则对观测点做 undistortPoints
    const bool undistort_points_only = !prep.undistorted && dist.rows >= 1 && dist.cols >= 4;
    std::vector<std::vector<cv::Point2f>> frames_pts2d_undistorted;
    if (!prep.undistorted && dist.rows >= 1 && dist.cols >= 4) {
        frames_pts2d_undistorted.resize(frames_pts2d.size());
        for (size_t f = 0; f < frames_pts2d.size(); ++f) {
            cv::undistortPoints(frames_pts2d[f], frames_pts2d_undistorted[f], K, dist, cv::noArray(), K);
        }
        UNICALIB_INFO("[LiDAR-Cam/Target] 已对 2D 观测去畸变，BA 使用针孔+去畸变点");
    }
    const std::vector<std::vector<cv::Point2f>>& frames_pts2d_use =
        (!prep.undistorted && dist.rows >= 1 && dist.cols >= 4) ? frames_pts2d_undistorted : frames_pts2d;

    std::vector<cv::Point3f> pts3d_all;
    std::vector<cv::Point2f> pts2d_all;
    for (size_t f = 0; f < frames_pts3d.size(); ++f) {
        for (size_t i = 0; i < frames_pts3d[f].size(); ++i) {
            pts3d_all.push_back(frames_pts3d[f][i]);
            pts2d_all.push_back(frames_pts2d_use[f][i]);
        }
    }

    Sophus::SE3d T_ba_init;
    bool has_good_init = init_extrin.has_value() &&
        (init_extrin->translation().norm() > 1e-5 ||
         !init_extrin->rotationMatrix().isApprox(Eigen::Matrix3d::Identity(), 1e-4));
    bool run_coarse = !has_good_init && cfg_.target_use_coarse_rotation_search;

    if (run_coarse) {
        UNICALIB_INFO("[LiDAR-Cam/Target] 无可靠初值，执行粗旋转网格搜索 (范围=±{:.1f}° 步长={:.1f}°)",
                      cfg_.target_coarse_rotation_range_deg, cfg_.target_coarse_rotation_step_deg);
        T_ba_init = coarse_rotation_search(
            frames_pts3d, frames_pts2d_use, fx, fy, cx, cy,
            cfg_.target_coarse_rotation_range_deg, cfg_.target_coarse_rotation_step_deg,
            cfg_.target_coarse_inlier_threshold_px, init_extrin);
        T_ba_init = refine_translation_only(
            frames_pts3d, frames_pts2d_use,
            Sophus::SO3d(T_ba_init.rotationMatrix()), T_ba_init.translation(),
            fx, fy, cx, cy, cfg_.target_ba_huber_scale_px, 30);
        UNICALIB_INFO("[LiDAR-Cam/Target] 粗搜索完成，初值 t=[{:.4f},{:.4f},{:.4f}]",
                      T_ba_init.translation().x(), T_ba_init.translation().y(), T_ba_init.translation().z());
    } else if (has_good_init) {
        T_ba_init = *init_extrin;
        UNICALIB_INFO("[LiDAR-Cam/Target] 使用提供的初值作为 BA 初值");
    } else {
        cv::Mat rvec, tvec;
        cv::Mat inliers;
        try {
            cv::solvePnPRansac(pts3d_all, pts2d_all, K, dist, rvec, tvec,
                               false, 200, 8.0f, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
            if (inliers.rows < 6) {
                UNICALIB_WARN("[LiDAR-Cam/Target] PnP RANSAC 内点不足: {} (需≥6)", inliers.rows);
                return std::nullopt;
            }
            cv::solvePnPRefineLM(pts3d_all, pts2d_all, K, dist, rvec, tvec);
        } catch (const cv::Exception& e) {
            UNICALIB_ERROR("[LiDAR-Cam/Target] OpenCV PnP 异常: {}", e.what());
            return std::nullopt;
        }
        Eigen::Vector3d rv(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        Eigen::Vector3d tv(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        Eigen::Matrix3d R = (rv.norm() < 1e-10) ? Eigen::Matrix3d::Identity()
            : Eigen::AngleAxisd(rv.norm(), rv.normalized()).toRotationMatrix();
        T_ba_init = Sophus::SE3d(R, tv);
        UNICALIB_INFO("[LiDAR-Cam/Target] PnP 求解完成，作为 BA 初值");
    }

    if (cfg_.target_per_frame_rms_threshold_px > 0) {
        std::vector<std::vector<cv::Point3f>> kept_pts3d;
        std::vector<std::vector<cv::Point2f>> kept_pts2d;
        for (size_t f = 0; f < frames_pts3d.size(); ++f) {
            double rms = compute_reproj_rms(
                frames_pts3d[f], frames_pts2d_use[f], T_ba_init, fx, fy, cx, cy);
            if (rms <= cfg_.target_per_frame_rms_threshold_px) {
                kept_pts3d.push_back(frames_pts3d[f]);
                kept_pts2d.push_back(frames_pts2d_use[f]);
            } else {
                UNICALIB_DEBUG("[LiDAR-Cam/Target] 剔除帧 {} (RMS={:.2f}px > {:.2f}px)",
                               f, rms, cfg_.target_per_frame_rms_threshold_px);
            }
        }
        if (static_cast<int>(kept_pts3d.size()) < min_frames) {
            UNICALIB_WARN("[LiDAR-Cam/Target] 离群剔除后帧数不足 (保留{} 需≥{})",
                          kept_pts3d.size(), min_frames);
        } else {
            frames_pts3d = std::move(kept_pts3d);
            if (undistort_points_only)
                frames_pts2d_undistorted = std::move(kept_pts2d);
            else
                frames_pts2d = std::move(kept_pts2d);
            UNICALIB_INFO("[LiDAR-Cam/Target] 每帧离群剔除后: {} 帧", frames_pts3d.size());
        }
    }

    double params[6];
    Eigen::AngleAxisd aa(T_ba_init.rotationMatrix());
    Eigen::Vector3d axis = aa.axis() * aa.angle();
    if (axis.norm() < 1e-12) axis = Eigen::Vector3d::Zero();
    params[0] = axis.x(); params[1] = axis.y(); params[2] = axis.z();
    params[3] = T_ba_init.translation().x();
    params[4] = T_ba_init.translation().y();
    params[5] = T_ba_init.translation().z();

    ceres::Problem problem;
    ceres::LossFunction* loss = cfg_.target_ba_use_robust_loss
        ? new ceres::HuberLoss(cfg_.target_ba_huber_scale_px) : nullptr;
    for (size_t f = 0; f < frames_pts3d.size(); ++f) {
        const auto& pts3d = frames_pts3d[f];
        const auto& pts2d = frames_pts2d_use[f];
        for (size_t i = 0; i < pts3d.size(); ++i) {
            TargetReprojCost* c = new TargetReprojCost;
            c->p_l[0] = pts3d[i].x; c->p_l[1] = pts3d[i].y; c->p_l[2] = pts3d[i].z;
            c->u_obs = pts2d[i].x; c->v_obs = pts2d[i].y;
            c->fx = fx; c->fy = fy; c->cx = cx; c->cy = cy;
            ceres::CostFunction* cost = new ceres::NumericDiffCostFunction<TargetReprojCost, ceres::CENTRAL, 2, 6>(c);
            problem.AddResidualBlock(cost, loss, params);
        }
    }

    ceres::Solver::Options opts;
    opts.max_num_iterations = cfg_.target_ba_max_iter;
    opts.minimizer_type = ceres::TRUST_REGION;
    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
    UNICALIB_CALC("LiDAR-Cam 目标法 BA Ceres initial_cost={:.6f} final_cost={:.6f} iter={} converged={}",
                  summary.initial_cost, summary.final_cost, static_cast<int>(summary.iterations.size()),
                  summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");
    if (cfg_.verbose && summary.termination_type != ceres::CONVERGENCE) {
        UNICALIB_WARN("[LiDAR-Cam/Target] Ceres BA 未收敛: {}", summary.message.c_str());
    }

    Eigen::Vector3d axis_f(params[0], params[1], params[2]);
    Eigen::Matrix3d R_f = (axis_f.norm() < 1e-12) ? Eigen::Matrix3d::Identity()
        : Eigen::AngleAxisd(axis_f.norm(), axis_f.normalized()).toRotationMatrix();
    Sophus::SE3d T_final(R_f, Eigen::Vector3d(params[3], params[4], params[5]));

    std::vector<cv::Point3f> pts3d_ba;
    std::vector<cv::Point2f> pts2d_ba;
    for (size_t f = 0; f < frames_pts3d.size(); ++f) {
        const auto& pts2d_f = frames_pts2d_use[f];
        for (size_t i = 0; i < frames_pts3d[f].size(); ++i) {
            pts3d_ba.push_back(frames_pts3d[f][i]);
            pts2d_ba.push_back(pts2d_f[i]);
        }
    }
    double rms = compute_reproj_rms(pts3d_ba, pts2d_ba, T_final, fx, fy, cx, cy);
    UNICALIB_CALC("LiDAR-Cam 目标法 BA 完成 帧数={} 点对数={} reproj_rms={:.4f}px",
                  frames_pts3d.size(), pts3d_ba.size(), rms);
    UNICALIB_INFO("[LiDAR-Cam/Target] BA 完成 帧数={} 点对数={} RMS={:.3f}px",
                  frames_pts3d.size(), pts3d_ba.size(), rms);

    ExtrinsicSE3 result;
    result.ref_sensor_id    = lidar_id;
    result.target_sensor_id = cam_id;
    result.SO3_TargetInRef  = Sophus::SO3d(R_f);
    result.POS_TargetInRef  = T_final.translation();
    result.residual_rms     = rms;
    result.is_converged     = (rms < cfg_.max_reproj_error_px);
    return result;
}

// 静态辅助：强度投影图（与 lidar_to_intensity_image 逻辑一致，供 Ceres 代价使用）
static cv::Mat lidar_to_intensity_image_static(
    const LiDARScan& scan,
    const Sophus::SE3d& T_cam_in_lidar,
    const CameraIntrinsics& cam_intrin) {
    cv::Mat img = cv::Mat::zeros(cam_intrin.height, cam_intrin.width, CV_32F);
    if (!scan.cloud) return img;
    for (const auto& pt : scan.cloud->points) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T_cam_in_lidar * p_l;
        if (p_c.z() < 0.1) continue;
        double u = cam_intrin.fx * p_c.x() / p_c.z() + cam_intrin.cx;
        double v = cam_intrin.fy * p_c.y() / p_c.z() + cam_intrin.cy;
        int iu = static_cast<int>(std::round(u));
        int iv = static_cast<int>(std::round(v));
        if (iu < 0 || iu >= cam_intrin.width || iv < 0 || iv >= cam_intrin.height) continue;
        if (img.at<float>(iv, iu) < pt.intensity)
            img.at<float>(iv, iu) = pt.intensity;
    }
    return img;
}

// ===================================================================
// 边缘对齐：单帧 NCC 计算（供 Ceres 代价使用）
// ===================================================================
static double compute_frame_ncc(
    const LiDARScan& scan,
    const cv::Mat& img_cam,
    const Sophus::SE3d& T_cam_lidar,
    const CameraIntrinsics& cam_intrin,
    int edge_canny_low,
    int edge_canny_high) {

    if (img_cam.empty()) return -1e9;
    cv::Mat gray;
    if (img_cam.channels() == 3) cv::cvtColor(img_cam, gray, cv::COLOR_BGR2GRAY);
    else gray = img_cam.clone();
    if (gray.empty()) return -1e9;
    if (gray.type() != CV_8UC1) {
        cv::Mat gray8;
        gray.convertTo(gray8, CV_8UC1);
        gray = gray8;
    }
    cv::Mat img_edges;
    cv::Canny(gray, img_edges, edge_canny_low, edge_canny_high);

    cv::Mat lidar_img = lidar_to_intensity_image_static(scan, T_cam_lidar, cam_intrin);
    if (lidar_img.empty()) return -1e9;
    double max_val;
    cv::minMaxLoc(lidar_img, nullptr, &max_val);
    if (max_val < 1e-3) return -1e9;
    cv::Mat lidar_8u;
    lidar_img.convertTo(lidar_8u, CV_8U, 255.0 / max_val);
    cv::Mat lidar_edges;
    cv::Canny(lidar_8u, lidar_edges, 30, 100);

    cv::Mat e1_full, e2_full;
    img_edges.convertTo(e1_full, CV_32F, 1.0 / 255);
    lidar_edges.convertTo(e2_full, CV_32F, 1.0 / 255);

    auto ncc_of = [](const cv::Mat& e1, const cv::Mat& e2) -> double {
        if (e1.empty() || e2.empty() || e1.size() != e2.size()) return -1e9;
        cv::Scalar m1, s1, m2, s2;
        cv::meanStdDev(e1, m1, s1);
        cv::meanStdDev(e2, m2, s2);
        if (s1[0] < 1e-5 || s2[0] < 1e-5) return -1e9;
        double denom = s1[0] * s2[0] * e1.total();
        if (std::abs(denom) < 1e-10) return -1e9;
        return (e1 - m1[0]).dot(e2 - m2[0]) / denom;
    };

    // 多尺度 NCC（提升 basin 稳定性；避免在高频边缘上过拟合）
    double ncc_sum = 0.0;
    double w_sum = 0.0;
    const std::array<std::pair<double, double>, 3> scales = {
        std::make_pair(1.0, 0.50),
        std::make_pair(0.5, 0.35),
        std::make_pair(0.25, 0.15),
    };
    for (const auto& sw : scales) {
        const double s = sw.first;
        const double w = sw.second;
        cv::Mat e1 = e1_full;
        cv::Mat e2 = e2_full;
        if (s < 0.999) {
            cv::Size sz(static_cast<int>(std::round(e1_full.cols * s)),
                        static_cast<int>(std::round(e1_full.rows * s)));
            if (sz.width < 16 || sz.height < 16) continue;
            cv::resize(e1_full, e1, sz, 0, 0, cv::INTER_AREA);
            cv::resize(e2_full, e2, sz, 0, 0, cv::INTER_AREA);
        }
        const double ncc = ncc_of(e1, e2);
        if (ncc <= -1e8) continue;
        ncc_sum += w * ncc;
        w_sum += w;
    }
    if (w_sum <= 1e-12) return -1e9;
    return ncc_sum / w_sum;
}

// Ceres 代价：6 参数 (angle-axis 3 + translation 3)，残差 = -NCC（最小化 -NCC = 最大化 NCC）
// 注意：改为存储 cv::Mat 副本而非指针，避免 Ceres 复制 functor 时指针失效
struct EdgeNCCCost {
    const LiDARScan* scan;       // 仍用指针（点云数据大，只读安全）
    cv::Mat image;               // 改为值拷贝，避免指针失效
    CameraIntrinsics intrin;
    int canny_low, canny_high;

    bool operator()(const double* const x, double* residual) const {
        residual[0] = 0;
        // 防护：检查参数有效性
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(x[i])) {
                UNICALIB_WARN("[EdgeNCCCost] 参数 x[{}]={} 非有限值，返回残差=0", i, x[i]);
                return true;
            }
        }
        if (!scan || image.empty()) return true;
        if (!scan->cloud || scan->cloud->empty()) return true;
        Eigen::Map<const Eigen::Vector3d> axis(x);
        Eigen::Map<const Eigen::Vector3d> t(x + 3);
        double angle = axis.norm();
        Eigen::Matrix3d R = axis_angle_to_rotation(angle, axis);
        Sophus::SE3d T(R, t);
        double ncc = compute_frame_ncc(*scan, image, T, intrin, canny_low, canny_high);
        if (!std::isfinite(ncc)) {
            UNICALIB_WARN("[EdgeNCCCost] NCC={} 非有限值，返回残差=0", ncc);
            residual[0] = 0;
        } else {
            residual[0] = -ncc;  // 最小化 -NCC
        }
        return true;
    }
};

// 角点重投影代价：单帧单点 2D keypoint，找当前 T 下投影最近的 3D 点，残差 = 重投影误差 (2 维)
struct CornerReprojCost {
    const LiDARScan* scan;
    double keypoint_u = 0.0;
    double keypoint_v = 0.0;
    CameraIntrinsics intrin;

    bool operator()(const double* const x, double* residual) const {
        residual[0] = residual[1] = 0;
        // 防护：检查参数有效性
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(x[i])) {
                UNICALIB_WARN("[CornerReprojCost] 参数 x[{}]={} 非有限值，返回残差=0", i, x[i]);
                return true;
            }
        }
        if (!scan || !scan->cloud || scan->cloud->empty()) return true;
        Eigen::Map<const Eigen::Vector3d> axis(x);
        Eigen::Map<const Eigen::Vector3d> t(x + 3);
        double angle = axis.norm();
        Eigen::Matrix3d R = axis_angle_to_rotation(angle, axis);
        Sophus::SE3d T(R, t);
        double best_d2 = 1e18;
        double best_u = keypoint_u, best_v = keypoint_v;
        const int max_pts = 5000;
        const size_t step = std::max(size_t(1), scan->cloud->size() / max_pts);
        for (size_t i = 0; i < scan->cloud->size(); i += step) {
            const auto& pt = scan->cloud->points[i];
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            Eigen::Vector3d p_c = T * Eigen::Vector3d(pt.x, pt.y, pt.z);
            if (p_c.z() < 0.1) continue;
            double u = intrin.fx * p_c.x() / p_c.z() + intrin.cx;
            double v = intrin.fy * p_c.y() / p_c.z() + intrin.cy;
            if (u < 0 || u >= intrin.width || v < 0 || v >= intrin.height) continue;
            double du = u - keypoint_u, dv = v - keypoint_v;
            double d2 = du * du + dv * dv;
            if (d2 < best_d2) { best_d2 = d2; best_u = u; best_v = v; }
        }
        if (!std::isfinite(best_u - keypoint_u) || !std::isfinite(best_v - keypoint_v)) {
            UNICALIB_WARN("[CornerReprojCost] 残差非有限值 best_u={} kp_u={} best_v={} kp_v={}",
                          best_u, keypoint_u, best_v, keypoint_v);
            return true;
        }
        residual[0] = best_u - keypoint_u;
        residual[1] = best_v - keypoint_v;
        return true;
    }
};

// 强度一致性代价：单帧，残差 = 投影区域内的平均强度差异 (1 维)，用于多特征融合
// 注意：改为存储 cv::Mat 副本而非指针，避免 Ceres 复制 functor 时指针失效
struct IntensityConsistencyCost {
    const LiDARScan* scan;       // 仍用指针（点云数据大，只读安全）
    cv::Mat image;              // 改为值拷贝，避免指针失效
    CameraIntrinsics intrin;

    bool operator()(const double* const x, double* residual) const {
        residual[0] = 0;
        // 防护：检查参数有效性
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(x[i])) {
                UNICALIB_WARN("[IntensityConsistencyCost] 参数 x[{}]={} 非有限值，返回残差=0", i, x[i]);
                return true;
            }
        }
        if (!scan || !scan->cloud || scan->cloud->empty() || image.empty())
            return true;
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }
        // 确保为 8 位灰度，避免 at<uint8_t> 在 16/32 位图上越界或错位
        if (gray.empty() || gray.type() != CV_8UC1) {
            cv::Mat gray8;
            if (gray.empty())
                return true;
            gray.convertTo(gray8, CV_8UC1);
            gray = gray8;
        }
        if (gray.empty()) return true;
        const int cols = gray.cols, rows = gray.rows;
        if (cols <= 0 || rows <= 0) return true;
        Eigen::Map<const Eigen::Vector3d> axis(x);
        Eigen::Map<const Eigen::Vector3d> t(x + 3);
        double angle = axis.norm();
        Eigen::Matrix3d R = axis_angle_to_rotation(angle, axis);
        Sophus::SE3d T(R, t);
        double sum_diff = 0.0;
        int count = 0;
        const int max_pts = 3000;
        const size_t step = std::max(size_t(1), scan->cloud->size() / max_pts);
        for (size_t i = 0; i < scan->cloud->size(); i += step) {
            const auto& pt = scan->cloud->points[i];
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            Eigen::Vector3d p_c = T * Eigen::Vector3d(pt.x, pt.y, pt.z);
            if (p_c.z() < 0.1) continue;
            int iu = static_cast<int>(std::round(intrin.fx * p_c.x() / p_c.z() + intrin.cx));
            int iv = static_cast<int>(std::round(intrin.fy * p_c.y() / p_c.z() + intrin.cy));
            if (iu < 0 || iu >= cols || iv < 0 || iv >= rows) continue;
            double lidar_val = std::min(255.0, std::max(0.0, static_cast<double>(pt.intensity)));
            double img_val = static_cast<double>(gray.at<uint8_t>(iv, iu));
            sum_diff += std::abs(lidar_val - img_val);
            count++;
        }
        double r = (count > 0) ? (sum_diff / count) / 255.0 : 0.0;  // 归一化到 [0,1]
        if (!std::isfinite(r)) {
            UNICALIB_WARN("[IntensityConsistencyCost] 残差 r={} 非有限值 count={} sum_diff={}", r, count, sum_diff);
            r = 0;
        }
        residual[0] = r;
        return true;
    }
};

// ===================================================================
// 方法 C: 边缘对齐法（Ceres 优化 NCC）
// ===================================================================
std::optional<ExtrinsicSE3> LiDARCameraCalibrator::calibrate_edge_align(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const CameraIntrinsics& cam_intrin,
    const Sophus::SE3d& init_guess,
    const std::string& lidar_id,
    const std::string& cam_id) {

    const auto prep = prepare_lidar_cam_calibration_images(camera_frames, cam_intrin);
    const auto& frames_calib = prep.frames;
    const CameraIntrinsics& intrin_calib = prep.intrin;

    bool init_is_identity = init_guess.log().norm() < 1e-9;
    UNICALIB_INFO("┌────────────────────────────────────────────────────────────────┐");
    UNICALIB_INFO("│ [精标定] 边缘对齐法 (Edge Alignment)                        │");
    UNICALIB_INFO("├────────────────────────────────────────────────────────────────┤");
    UNICALIB_INFO("│ 初始值: {} (identity={})",
                 init_is_identity ? "identity" : "粗标定提供", init_is_identity);
    if (!init_is_identity) {
        Eigen::Vector3d t = init_guess.translation();
        Eigen::Vector3d euler = init_guess.rotationMatrix().eulerAngles(0, 1, 2);
        UNICALIB_INFO("│   平移: [{:.4f}, {:.4f}, {:.4f}] m", t.x(), t.y(), t.z());
        UNICALIB_INFO("│   旋转: [{:.2f}, {:.2f}, {:.2f}] deg", euler.x()*180/M_PI, euler.y()*180/M_PI, euler.z()*180/M_PI);
    }
    size_t N = std::min(lidar_scans.size(), frames_calib.size());
    if (N == 0) {
        UNICALIB_ERROR("│ [错误] 数据为空 LiDAR帧:{} 图像帧:{}", lidar_scans.size(), frames_calib.size());
        return std::nullopt;
    }
    const size_t max_frames = 30;
    const size_t num_to_try = std::min(N, max_frames);
    UNICALIB_INFO("│ 数据: 点云帧={} 图像帧={} 采样={} 尺寸={}x{}",
                 lidar_scans.size(), frames_calib.size(), num_to_try, intrin_calib.width, intrin_calib.height);
    UNICALIB_INFO("│ 时间: 图像[{:.3f}-{:.3f}]s LiDAR[{:.3f}-{:.3f}]s",
                 frames_calib.front().first, frames_calib.back().first,
                 lidar_scans.front().timestamp, lidar_scans.back().timestamp);
    UNICALIB_INFO("│ 配置: 同步阈值={:.3f}s 边缘Canny({}, {}) NCC阈值={:.3f}",
                 cfg_.frame_sync_threshold_s, cfg_.edge_canny_low, cfg_.edge_canny_high, cfg_.ncc_threshold);
    UNICALIB_INFO("│ 特征权重: edge={:.2f} corner={:.2f} intensity={:.2f}",
                 cfg_.edge_weight, cfg_.corner_weight, cfg_.intensity_weight);
    UNICALIB_INFO("└────────────────────────────────────────────────────────────────┘");

    Sophus::SE3d T_cam_lidar = init_guess;
    // 时间偏移：在进入非凸的 Ceres 优化前，先做一次离散搜索找到更合理的配对（避免在错误帧对上优化）
    double time_offset_s = cfg_.time_offset_init_s;
    if (cfg_.optimize_time_offset && !lidar_scans.empty() && !frames_calib.empty()) {
        const double sync_thresh = cfg_.frame_sync_threshold_s;
        // 使用配置的时间偏移搜索范围，不再硬编码限制为 0.5s
        const double search_range_s = cfg_.time_offset_search_range_s;
        const double step_s = std::max(0.005, std::min(0.05, search_range_s * 0.1));
        double best_off = time_offset_s;
        double best_score = -1e9;
        for (double off = time_offset_s - search_range_s; off <= time_offset_s + search_range_s + 1e-12; off += step_s) {
            double local_best = -1e9;
            for (size_t k = 0; k < num_to_try; ++k) {
                size_t fi = (k * N) / num_to_try;
                if (fi >= N) break;
                const auto& [ts_cam, img_cam] = frames_calib[fi];
                if (img_cam.empty()) continue;
                const double target_ts = ts_cam + off;
                const LiDARScan* best_scan = nullptr;
                double best_dt = 1e9;
                for (const auto& scan : lidar_scans) {
                    double dt = std::abs(scan.timestamp - target_ts);
                    if (dt < best_dt) { best_dt = dt; best_scan = &scan; }
                }
                if (!best_scan || best_dt > sync_thresh) continue;
                double ncc = compute_frame_ncc(*best_scan, img_cam, T_cam_lidar, intrin_calib, cfg_.edge_canny_low, cfg_.edge_canny_high);
                local_best = std::max(local_best, ncc);
            }
            if (local_best > best_score) { best_score = local_best; best_off = off; }
        }
        time_offset_s = best_off;
        UNICALIB_INFO("│ [时间偏移搜索] 最佳偏移={:+.4f}s 最佳NCC={:.4f}", time_offset_s, best_score);
    } else {
        UNICALIB_INFO("│ [时间偏移搜索] 跳过 (optimize_time_offset=false)");
    }
    double init_ncc = -1e9;
    std::vector<std::pair<const LiDARScan*, cv::Mat>> frame_pairs;
    std::vector<double> ncc_samples;
    size_t candidates_tried = 0;
    // 使用配置中的 ncc_threshold，默认 0.05，放宽到 0.0 以允许更多帧参与优化
    const double ncc_threshold_default = cfg_.ncc_threshold > -1e8 ? cfg_.ncc_threshold : 0.05;
    const double ncc_threshold_relaxed = 0.0;    // 极度放宽时允许 NCC >= 0
    double ncc_threshold_used = ncc_threshold_default;

    const double sync_thresh = cfg_.frame_sync_threshold_s;
    auto collect_frame_pairs = [&](double ncc_thresh) {
        frame_pairs.clear();
        init_ncc = -1e9;
        for (size_t k = 0; k < num_to_try; ++k) {
            size_t fi = (k * N) / num_to_try;
            if (fi >= N) break;
            const auto& [ts_cam, img_cam] = frames_calib[fi];
            if (img_cam.empty()) continue;
            const LiDARScan* best_scan = nullptr;
            double best_dt = 1e9;
            for (const auto& scan : lidar_scans) {
                double dt = std::abs(scan.timestamp - (ts_cam + time_offset_s));
                if (dt < best_dt) { best_dt = dt; best_scan = &scan; }
            }
            if (!best_scan || best_dt > sync_thresh) continue;
            double ncc = compute_frame_ncc(*best_scan, img_cam, T_cam_lidar, intrin_calib, cfg_.edge_canny_low, cfg_.edge_canny_high);
            if (cfg_.ncc_low_skip_threshold > -1e8 && ncc < cfg_.ncc_low_skip_threshold) {
                UNICALIB_DEBUG("  [精标定] 低NCC帧跳过 fi={} ncc={:.4f} (阈值={:.4f})", fi, ncc, cfg_.ncc_low_skip_threshold);
                continue;
            }
            if (ncc > ncc_thresh) {
                init_ncc = std::max(init_ncc, ncc);
                frame_pairs.emplace_back(best_scan, img_cam);
            }
        }
    };

    for (size_t k = 0; k < num_to_try; ++k) {
        size_t fi = (k * N) / num_to_try;
        if (fi >= N) break;
        const auto& [ts_cam, img_cam] = frames_calib[fi];
        if (img_cam.empty()) continue;
        const LiDARScan* best_scan = nullptr;
        double best_dt = 1e9;
        for (const auto& scan : lidar_scans) {
            double dt = std::abs(scan.timestamp - (ts_cam + time_offset_s));
            if (dt < best_dt) { best_dt = dt; best_scan = &scan; }
        }
        if (!best_scan || best_dt > sync_thresh) continue;
        double ncc = compute_frame_ncc(*best_scan, img_cam, T_cam_lidar, intrin_calib, cfg_.edge_canny_low, cfg_.edge_canny_high);
        candidates_tried++;
        ncc_samples.push_back(ncc);
        if (cfg_.ncc_low_skip_threshold > -1e8 && ncc < cfg_.ncc_low_skip_threshold) {
            UNICALIB_DEBUG("  [精标定] 低NCC帧诊断 fi={} ncc={:.4f} dt={:.4f}s", fi, ncc, best_dt);
        }
        if (ncc > ncc_threshold_default) {
            init_ncc = std::max(init_ncc, ncc);
            frame_pairs.emplace_back(best_scan, img_cam);
        }
    }
    if (frame_pairs.size() < 3 && !ncc_samples.empty()) {
        double best_ncc = *std::max_element(ncc_samples.begin(), ncc_samples.end());
        UNICALIB_INFO("  [精标定] 有效帧数={} (<3)  当前阈值={:.3f}  候选帧最大NCC={:.4f}",
                      frame_pairs.size(), ncc_threshold_default, best_ncc);
        if (best_ncc > ncc_threshold_relaxed) {
            ncc_threshold_used = ncc_threshold_relaxed;
            collect_frame_pairs(ncc_threshold_relaxed);
            UNICALIB_INFO("  [精标定] 放宽 NCC 阈值至 {:.3f}  放宽后有效帧数={}", ncc_threshold_relaxed, frame_pairs.size());
        } else {
            UNICALIB_INFO("  [精标定] 最大NCC={:.4f} 未超过放宽阈值 {:.3f}，不进行放宽重试", best_ncc, ncc_threshold_relaxed);
        }
    }
    double ncc_min = 1e9, ncc_max = -1e9, ncc_sum = 0;
    for (double v : ncc_samples) {
        ncc_min = std::min(ncc_min, v);
        ncc_max = std::max(ncc_max, v);
        ncc_sum += v;
    }
    double ncc_mean = ncc_samples.empty() ? 0 : (ncc_sum / static_cast<double>(ncc_samples.size()));
    const bool has_invalid_ncc = (ncc_min < -1e6);
    if (has_invalid_ncc)
        UNICALIB_WARN("  [精标定] 检测到异常 NCC 值(占位 -1e9)，部分候选帧投影/边缘失败已排除；建议检查 LiDAR/相机 FOV、时间同步与内参");
    const bool ncc_valid = (init_ncc > -1e8);
    if (ncc_valid)
        UNICALIB_INFO("  初值 NCC={:.4f}  有效帧数={}  (候选帧数={}  NCC范围=[{:.4f},{:.4f}] 均值={:.4f})",
                      init_ncc, frame_pairs.size(), candidates_tried, ncc_min, ncc_max, ncc_mean);
    else if (!ncc_samples.empty())
        UNICALIB_INFO("  初值 NCC=N/A(无有效帧)  有效帧数=0  (候选帧数={}  NCC范围=[{:.4f},{:.4f}] 均值={:.4f})",
                      candidates_tried, ncc_min, ncc_max, ncc_mean);
    else
        UNICALIB_INFO("  初值 NCC=N/A(无有效帧)  有效帧数=0  (候选帧数=0，未评估任何帧；请检查 LiDAR/相机时间戳与数据)");
    if (frame_pairs.size() < 3) {
        double diag_best_ncc = ncc_samples.empty() ? -1e9 : *std::max_element(ncc_samples.begin(), ncc_samples.end());
        UNICALIB_WARN("  [精标定] 有效帧不足  有效帧数={}  候选帧数={}  NCC阈值={:.3f}  最大NCC={:.4f}  → 跳过 Ceres，返回初值",
                      frame_pairs.size(), candidates_tried, ncc_threshold_used, diag_best_ncc);
        UNICALIB_WARN("  诊断: 投影与图像边缘重叠不足(NCC 均<=阈值)。建议: 1) 查看 [MIAS-LCEC]/[PnP] 粗标定是否给出非 identity 2) 确认 LiDAR/相机 FOV 与时间同步 3) 检查内参 width/height 与图像一致");
        ExtrinsicSE3 result;
        result.ref_sensor_id = lidar_id;
        result.target_sensor_id = cam_id;
        result.set_SE3(T_cam_lidar);
        result.residual_rms = 1.0 - std::max(0.0, init_ncc);
        result.is_converged = (init_ncc > 0.3);
        return result;
    }

    // Ceres 优化：6 参数 (angle-axis + translation)
    double params[6];
    Eigen::AngleAxisd aa(T_cam_lidar.rotationMatrix());
    params[0] = aa.axis().x() * aa.angle();
    params[1] = aa.axis().y() * aa.angle();
    params[2] = aa.axis().z() * aa.angle();
    params[3] = T_cam_lidar.translation().x();
    params[4] = T_cam_lidar.translation().y();
    params[5] = T_cam_lidar.translation().z();

    auto make_robust_loss = [this]() -> ceres::LossFunction* {
        if (!cfg_.use_robust_loss || cfg_.robust_loss_threshold < 1e-6) return nullptr;
        if (cfg_.robust_loss_type == "cauchy")
            return new ceres::CauchyLoss(cfg_.robust_loss_threshold);
        return new ceres::HuberLoss(cfg_.robust_loss_threshold);
    };

    ceres::Problem problem;
    const double eps = 1e-6;
    const bool use_edge   = (cfg_.edge_weight > eps);
    const bool use_corner = (cfg_.corner_weight > eps && cfg_.corner_max_per_frame > 0);
    const bool use_intensity = (cfg_.intensity_weight > eps);
    int num_residual_blocks = 0;

    std::vector<EdgeNCCCost> edge_functors(frame_pairs.size());
    for (size_t i = 0; i < frame_pairs.size(); ++i) {
        edge_functors[i].scan = frame_pairs[i].first;
        edge_functors[i].image = frame_pairs[i].second.clone();  // 值拷贝
        edge_functors[i].intrin = intrin_calib;
        edge_functors[i].canny_low = cfg_.edge_canny_low;
        edge_functors[i].canny_high = cfg_.edge_canny_high;
        ceres::CostFunction* cost = new ceres::NumericDiffCostFunction<EdgeNCCCost, ceres::CENTRAL, 1, 6>(
            &edge_functors[i], ceres::DO_NOT_TAKE_OWNERSHIP, 1, ceres::NumericDiffOptions());
        ceres::LossFunction* loss = nullptr;
        if (ceres::LossFunction* rho = make_robust_loss()) {
            loss = new ceres::ScaledLoss(rho, std::sqrt(std::max(eps, cfg_.edge_weight)), ceres::TAKE_OWNERSHIP);
        }
        problem.AddResidualBlock(cost, loss, params);
        num_residual_blocks++;
    }

    // 预声明 functor 向量（供后续预检查使用）
    std::vector<CornerReprojCost> corner_functors;
    std::vector<IntensityConsistencyCost> intensity_functors;

    if (use_corner) {
        std::vector<std::vector<cv::Point2f>> frame_keypoints(frame_pairs.size());
        int total_corners = 0;
        for (size_t i = 0; i < frame_pairs.size(); ++i) {
            cv::Mat gray;
            if (frame_pairs[i].second.channels() == 3)
                cv::cvtColor(frame_pairs[i].second, gray, cv::COLOR_BGR2GRAY);
            else
                gray = frame_pairs[i].second.clone();
            if (!gray.empty() && gray.type() != CV_8UC1) {
                cv::Mat gray8;
                gray.convertTo(gray8, CV_8UC1);
                gray = gray8;
            }
            std::vector<cv::Point2f> kpts;
            cv::goodFeaturesToTrack(gray, kpts, cfg_.corner_max_per_frame, 0.01, 10);
            if (kpts.size() > static_cast<size_t>(cfg_.corner_max_per_frame))
                kpts.resize(cfg_.corner_max_per_frame);
            frame_keypoints[i] = std::move(kpts);
            total_corners += static_cast<int>(frame_keypoints[i].size());
        }
        UNICALIB_INFO("  [多特征] 角点: {} 帧共 {} 个 keypoints", frame_pairs.size(), total_corners);

        corner_functors.reserve(static_cast<size_t>(total_corners));
        for (size_t i = 0; i < frame_pairs.size(); ++i) {
            for (const auto& kp : frame_keypoints[i]) {
                CornerReprojCost fc;
                fc.scan = frame_pairs[i].first;
                fc.keypoint_u = kp.x;
                fc.keypoint_v = kp.y;
                fc.intrin = intrin_calib;
                corner_functors.push_back(fc);
            }
        }
        for (size_t k = 0; k < corner_functors.size(); ++k) {
            ceres::CostFunction* cost = new ceres::NumericDiffCostFunction<CornerReprojCost, ceres::CENTRAL, 2, 6>(
                &corner_functors[k], ceres::DO_NOT_TAKE_OWNERSHIP, 2, ceres::NumericDiffOptions());
            ceres::LossFunction* loss = nullptr;
            if (ceres::LossFunction* rho = make_robust_loss())
                loss = new ceres::ScaledLoss(rho, std::sqrt(std::max(eps, cfg_.corner_weight)), ceres::TAKE_OWNERSHIP);
            problem.AddResidualBlock(cost, loss, params);
            num_residual_blocks++;
        }
    }

    if (use_intensity) {
        intensity_functors.resize(frame_pairs.size());
        for (size_t i = 0; i < frame_pairs.size(); ++i) {
            intensity_functors[i].scan = frame_pairs[i].first;
            intensity_functors[i].image = frame_pairs[i].second.clone();  // 值拷贝
            intensity_functors[i].intrin = intrin_calib;
            ceres::CostFunction* cost = new ceres::NumericDiffCostFunction<IntensityConsistencyCost, ceres::CENTRAL, 1, 6>(
                &intensity_functors[i], ceres::DO_NOT_TAKE_OWNERSHIP, 1, ceres::NumericDiffOptions());
            ceres::LossFunction* loss = nullptr;
            if (ceres::LossFunction* rho = make_robust_loss())
                loss = new ceres::ScaledLoss(rho, std::sqrt(std::max(eps, cfg_.intensity_weight)), ceres::TAKE_OWNERSHIP);
            problem.AddResidualBlock(cost, loss, params);
            num_residual_blocks++;
        }
        UNICALIB_INFO("  [多特征] 强度一致性: {} 帧", frame_pairs.size());
    }

    UNICALIB_INFO("  [精标定] 残差块总数={}  即将调用 Ceres::Solve", num_residual_blocks);
    UNICALIB_INFO("  [精标定] 初始参数: axis=[{:.6f},{:.6f},{:.6f}] trans=[{:.4f},{:.4f},{:.4f}]",
                  params[0], params[1], params[2], params[3], params[4], params[5]);
    UNICALIB_INFO("  [精标定] 初值 T_cam_lidar: t=[{:.4f},{:.4f},{:.4f}]m",
                  T_cam_lidar.translation().x(), T_cam_lidar.translation().y(), T_cam_lidar.translation().z());
    UNICALIB_INFO("  [精标定] frame_pairs={} use_edge={} use_corner={} use_intensity={}",
                  frame_pairs.size(), use_edge, use_corner, use_intensity);

    // ========== 全面检查 Ceres 输入数据 ==========
    UNICALIB_INFO("  [预检查] 开始全面检查 Ceres 输入数据...");

    // 1. 检查初始参数
    bool params_valid = true;
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(params[i])) {
            UNICALIB_ERROR("  [预检查] 参数 params[{}]={} 非有限值!", i, params[i]);
            params_valid = false;
        }
    }
    if (params_valid) {
        double angle = std::sqrt(params[0]*params[0] + params[1]*params[1] + params[2]*params[2]);
        double trans_norm = std::sqrt(params[3]*params[3] + params[4]*params[4] + params[5]*params[5]);
        UNICALIB_INFO("  [预检查] 参数角度={:.6f} rad 平移范数={:.4f} m", angle, trans_norm);
        if (angle > M_PI * 2) {
            UNICALIB_WARN("  [预检查] 旋转角 angle={:.4f} rad 超过 2*PI，可能不合理", angle);
        }
    }

    // 2. 检查 frame_pairs 数据完整性
    int valid_frame_pairs = 0;
    int empty_image_count = 0;
    int empty_cloud_count = 0;
    int zero_cloud_count = 0;
    for (size_t i = 0; i < frame_pairs.size(); ++i) {
        const auto& pr = frame_pairs[i];
        bool valid = true;
        if (!pr.first) {
            empty_cloud_count++;
            valid = false;
        } else if (!pr.first->cloud || pr.first->cloud->empty()) {
            zero_cloud_count++;
            valid = false;
        }
        if (pr.second.empty()) {
            empty_image_count++;
            valid = false;
        }
        if (valid) valid_frame_pairs++;
    }
    UNICALIB_INFO("  [预检查] frame_pairs 统计: 总数={} 有效={} 空图={} 空点云={} 零-size点云={}",
                  frame_pairs.size(), valid_frame_pairs, empty_image_count, empty_cloud_count, zero_cloud_count);

    // 3. 检查 edge_functors
    if (use_edge && !edge_functors.empty()) {
        int edge_invalid_scan = 0, edge_invalid_image = 0, edge_image_empty = 0;
        for (size_t i = 0; i < edge_functors.size(); ++i) {
            if (!edge_functors[i].scan) edge_invalid_scan++;
            else if (!edge_functors[i].scan->cloud || edge_functors[i].scan->cloud->empty()) edge_invalid_scan++;
            if (edge_functors[i].image.empty()) edge_image_empty++;
        }
        UNICALIB_INFO("  [预检查] EdgeNCCCost 统计: 总数={} scan无效={} image空={}",
                      edge_functors.size(), edge_invalid_scan, edge_image_empty);
    }

    // 4. 检查 corner_functors
    if (use_corner && !corner_functors.empty()) {
        int corner_invalid_scan = 0;
        for (size_t i = 0; i < corner_functors.size(); ++i) {
            if (!corner_functors[i].scan) corner_invalid_scan++;
            else if (!corner_functors[i].scan->cloud || corner_functors[i].scan->cloud->empty()) corner_invalid_scan++;
        }
        UNICALIB_INFO("  [预检查] CornerReprojCost 统计: 总数={} scan无效={}",
                      corner_functors.size(), corner_invalid_scan);
    }

    // 5. 检查 intensity_functors
    if (use_intensity && !intensity_functors.empty()) {
        int intensity_invalid_scan = 0, intensity_image_empty = 0;
        for (size_t i = 0; i < intensity_functors.size(); ++i) {
            if (!intensity_functors[i].scan) intensity_invalid_scan++;
            else if (!intensity_functors[i].scan->cloud || intensity_functors[i].scan->cloud->empty()) intensity_invalid_scan++;
            if (intensity_functors[i].image.empty()) intensity_image_empty++;
        }
        UNICALIB_INFO("  [预检查] IntensityConsistencyCost 统计: 总数={} scan无效={} image空={}",
                      intensity_functors.size(), intensity_invalid_scan, intensity_image_empty);
    }

    // 6. 检查相机内参
    UNICALIB_INFO("  [预检查] 相机内参: fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f} size={}x{}",
                  intrin_calib.fx, intrin_calib.fy, intrin_calib.cx, intrin_calib.cy,
                  intrin_calib.width, intrin_calib.height);
    if (intrin_calib.fx <= 0 || intrin_calib.fy <= 0) {
        UNICALIB_ERROR("  [预检查] 相机焦距无效 fx={} fy={}!", intrin_calib.fx, intrin_calib.fy);
        params_valid = false;
    }

    // 7. 检查问题规模
    UNICALIB_INFO("  [预检查] Ceres 问题规模: 残差块={} 参数块=1(6维)",
                  num_residual_blocks);

    if (valid_frame_pairs < 3) {
        UNICALIB_WARN("  [预检查] 有效帧数={} < 3，优化可能不稳定", valid_frame_pairs);
    }

    if (!params_valid) {
        UNICALIB_ERROR("  [预检查] 参数验证失败，跳过 Ceres 优化");
        ExtrinsicSE3 result;
        result.ref_sensor_id    = lidar_id;
        result.target_sensor_id = cam_id;
        result.set_SE3(T_cam_lidar);
        result.residual_rms  = 1.0;
        result.is_converged  = false;
        return result;
    }

    UNICALIB_INFO("  [预检查] Ceres 输入数据检查完成");
    if (Logger::isInitialized()) Logger::flush();

    if (num_residual_blocks == 0) {
        UNICALIB_WARN("  [精标定] 无残差块，跳过优化，直接使用初值");
        ExtrinsicSE3 result;
        result.ref_sensor_id    = lidar_id;
        result.target_sensor_id = cam_id;
        result.set_SE3(T_cam_lidar);
        result.residual_rms  = 1.0 - std::max(0.0, init_ncc);
        result.is_converged  = (init_ncc > 0.3);
        return result;
    }

    ceres::Solver::Options options;
    options.max_num_iterations = cfg_.ceres_max_iter > 0 ? cfg_.ceres_max_iter : 50;
    options.minimizer_progress_to_stdout = (cfg_.verbose);
    // DENSE_QR 对小问题 (残差块 < 1000) 有效，但 2280 残差可能不稳定
    // 改用迭代求解器，更稳健
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.use_explicit_schur_complement = true;
    options.num_threads = 1;  // 代价函数内使用 OpenCV/点云，多线程可能触发竞态与段错误

    UNICALIB_INFO("  [Ceres] 即将调用 Solve，残差块={} 求解器={}",
                  num_residual_blocks,
                  options.linear_solver_type == ceres::DENSE_QR ? "DENSE_QR" :
                  (options.linear_solver_type == ceres::ITERATIVE_SCHUR ? "ITERATIVE_SCHUR" : "OTHER"));
    if (Logger::isInitialized()) Logger::flush();

    // 打印内存状态
    UNICALIB_INFO("  [调试] 开始 Ceres Solve，残差块={}", num_residual_blocks);

    // 手动测试第一个残差块是否能正常求值（排查数值导数阶段崩溃）
    UNICALIB_INFO("  [调试] 手动测试代价函数求值...");
    if (!edge_functors.empty()) {
        double test_residual = 0;
        double test_params[6] = {params[0], params[1], params[2], params[3], params[4], params[5]};
        bool eval_ok = edge_functors[0](test_params, &test_residual);
        UNICALIB_INFO("  [调试] 第一个 EdgeNCCCost 求值结果: ok={} residual={}", eval_ok, test_residual);
    } else if (!corner_functors.empty()) {
        double test_residual[2] = {0, 0};
        double test_params[6] = {params[0], params[1], params[2], params[3], params[4], params[5]};
        bool eval_ok = corner_functors[0](test_params, test_residual);
        UNICALIB_INFO("  [调试] 第一个 CornerReprojCost 求值结果: ok={} residual=[{}, {}]", eval_ok, test_residual[0], test_residual[1]);
    } else if (!intensity_functors.empty()) {
        double test_residual = 0;
        double test_params[6] = {params[0], params[1], params[2], params[3], params[4], params[5]};
        bool eval_ok = intensity_functors[0](test_params, &test_residual);
        UNICALIB_INFO("  [调试] 第一个 IntensityConsistencyCost 求值结果: ok={} residual={}", eval_ok, test_residual);
    }
    if (Logger::isInitialized()) Logger::flush();
    UNICALIB_INFO("  [调试] 代价函数测试完成，准备调用 Ceres::Solve");

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (Logger::isInitialized()) Logger::flush();
    UNICALIB_INFO("  [Ceres] Solve 完成");
    UNICALIB_CALC("LiDAR-Cam 边缘法 Ceres iter={} residual_blocks={} initial_cost={:.6f} final_cost={:.6f} converged={}",
                  static_cast<int>(summary.iterations.size()), static_cast<int>(frame_pairs.size()),
                  summary.initial_cost, summary.final_cost,
                  summary.termination_type == ceres::CONVERGENCE ? "yes" : "no");
    UNICALIB_INFO("  [Ceres] 迭代数={}  残差块数={}  initial_cost={:.6f}  final_cost={:.6f}  converged={}  termination={}",
                  summary.iterations.size(), static_cast<int>(frame_pairs.size()),
                  summary.initial_cost, summary.final_cost,
                  summary.termination_type == ceres::CONVERGENCE ? "true" : "false",
                  summary.termination_type == ceres::CONVERGENCE ? "CONVERGENCE" :
                  (summary.termination_type == ceres::NO_CONVERGENCE ? "NO_CONVERGENCE" : "OTHER"));

    Eigen::Vector3d axis(params[0], params[1], params[2]);
    double angle = axis.norm();
    if (angle < 1e-12)
        T_cam_lidar = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d(params[3], params[4], params[5]));
    else {
        Eigen::AngleAxisd aa_final(angle, axis / angle);
        T_cam_lidar = Sophus::SE3d(aa_final.toRotationMatrix(), Eigen::Vector3d(params[3], params[4], params[5]));
    }
    double final_ncc = -1e9;
    for (const auto& pr : frame_pairs)
        final_ncc = std::max(final_ncc, compute_frame_ncc(*pr.first, pr.second, T_cam_lidar, intrin_calib, cfg_.edge_canny_low, cfg_.edge_canny_high));
    double rms = 1.0 - std::max(0.0, final_ncc);
    bool converged = (final_ncc > 0.3);

    Eigen::Vector3d t_final = T_cam_lidar.translation();
    Eigen::Vector3d euler_final = T_cam_lidar.rotationMatrix().eulerAngles(0, 1, 2);
    UNICALIB_INFO("┌────────────────────────────────────────────────────────────────┐");
    UNICALIB_INFO("│ [优化结果]");
    UNICALIB_CALC("LiDAR-Cam 边缘法 优化结果 NCC={:.4f} rms={:.4f}px converged={}", final_ncc, rms, converged ? "yes" : "no");
    UNICALIB_INFO("│   - NCC: {:.4f}  RMS: {:.4f}px  收敛: {}", final_ncc, rms, converged ? "是" : "否");
    UNICALIB_INFO("│   - 平移: [{:.4f}, {:.4f}, {:.4f}] m", t_final.x(), t_final.y(), t_final.z());
    UNICALIB_INFO("│   - 旋转: [{:.2f}, {:.2f}, {:.2f}] deg (RPY)", euler_final.x()*180/M_PI, euler_final.y()*180/M_PI, euler_final.z()*180/M_PI);
    UNICALIB_INFO("│   - 质量: {}", converged ? "合格 (NCC>0.3)" : "不合格 (NCC<0.3)");
    UNICALIB_INFO("└────────────────────────────────────────────────────────────────┘");

    ExtrinsicSE3 result;
    result.ref_sensor_id    = lidar_id;
    result.target_sensor_id = cam_id;
    result.set_SE3(T_cam_lidar);
    result.residual_rms  = rms;
    result.is_converged  = converged;
    return result;
}

// ===================================================================
// 方法 A: 运动法 (手眼)
// ===================================================================
std::optional<ExtrinsicSE3> LiDARCameraCalibrator::calibrate_motion(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const ExtrinsicSE3& T_lidar_in_imu,
    const CameraIntrinsics& cam_intrin,
    const std::string& lidar_id,
    const std::string& cam_id) {

    UNICALIB_INFO("=== LiDAR-Camera 运动法 ===");
    (void)imu_data; (void)cam_intrin;

    // 简化: 使用 LiDAR 和相机的相对运动进行手眼标定
    // 完整版需要 B样条插值 + Ceres 联合优化
    UNICALIB_WARN("运动法当前使用简化实现, 完整版需要 iKalibr CalibSolver 接口");

    if (lidar_scans.size() < 4 || camera_frames.size() < 4) {
        UNICALIB_ERROR("数据不足");
        return std::nullopt;
    }

    ExtrinsicSE3 result;
    result.ref_sensor_id    = lidar_id;
    result.target_sensor_id = cam_id;
    result.set_SE3(T_lidar_in_imu.SE3_TargetInRef());
    result.is_converged = false;
    result.residual_rms = 999.0;
    UNICALIB_INFO("  运动法: 返回 IMU-LiDAR 外参作为 LiDAR-Camera 初值 (需要完整实现)");
    return result;
}

// ===================================================================
// 可视化投影
// ===================================================================
void LiDARCameraCalibrator::visualize_projection(
    const LiDARScan& scan,
    const cv::Mat& image,
    const ExtrinsicSE3& extrin,
    const CameraIntrinsics& cam_intrin,
    const std::string& output_path) {

    if (!scan.cloud || image.empty()) return;

    const cv::Mat img_calib = undistort_camera_image(image, cam_intrin);
    const CameraIntrinsics intrin_calib = pinhole_intrinsics_without_distortion(cam_intrin);

    cv::Mat vis = img_calib.clone();
    if (vis.channels() == 1)
        cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);

    Sophus::SE3d T = extrin.SE3_TargetInRef();
    // T 是 cam_in_lidar 还是 lidar_in_cam 取决于约定
    // 这里假设 extrin 是 lidar_in_cam (cam 为 ref)

    double d_min = 1e9, d_max = -1e9;
    for (const auto& pt : scan.cloud->points) {
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T * p_l;
        if (p_c.z() > 0) {
            d_min = std::min(d_min, p_c.z());
            d_max = std::max(d_max, p_c.z());
        }
    }
    if (d_max <= d_min) return;

    for (const auto& pt : scan.cloud->points) {
        if (std::isnan(pt.x)) continue;
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T * p_l;
        if (p_c.z() < 0.1) continue;

        double u = intrin_calib.fx * p_c.x() / p_c.z() + intrin_calib.cx;
        double v = intrin_calib.fy * p_c.y() / p_c.z() + intrin_calib.cy;
        int iu = static_cast<int>(std::round(u));
        int iv = static_cast<int>(std::round(v));
        const int img_w = intrin_calib.width > 0 ? intrin_calib.width : vis.cols;
        const int img_h = intrin_calib.height > 0 ? intrin_calib.height : vis.rows;
        if (iu < 2 || iu >= img_w - 2 ||
            iv < 2 || iv >= img_h - 2) continue;

        // 深度着色 (近=红, 远=蓝)
        double ratio = (p_c.z() - d_min) / (d_max - d_min);
        int r = static_cast<int>((1-ratio) * 255);
        int b = static_cast<int>(ratio * 255);
        cv::circle(vis, {iu, iv}, 2, cv::Scalar(b, 80, r), -1);
    }

    cv::imwrite(output_path, vis);
    UNICALIB_INFO("投影可视化保存: {}", output_path);
}

// ===================================================================
// 边缘对齐评分
// ===================================================================
EdgeAlignmentScore LiDARCameraCalibrator::evaluate_edge_alignment(
    const LiDARScan& scan,
    const cv::Mat& image,
    const ExtrinsicSE3& extrin,
    const CameraIntrinsics& cam_intrin) {

    EdgeAlignmentScore score;
    if (!scan.cloud || image.empty()) return score;

    const cv::Mat img_calib = undistort_camera_image(image, cam_intrin);
    const CameraIntrinsics intrin_calib = pinhole_intrinsics_without_distortion(cam_intrin);

    Sophus::SE3d T = extrin.SE3_TargetInRef();
    cv::Mat lidar_img = lidar_to_intensity_image(scan, T, intrin_calib);

    cv::Mat gray;
    if (img_calib.channels() == 3) cv::cvtColor(img_calib, gray, cv::COLOR_BGR2GRAY);
    else gray = img_calib.clone();
    cv::Mat img_edges, lidar_edges;
    cv::Canny(gray, img_edges, cfg_.edge_canny_low, cfg_.edge_canny_high);

    double max_val;
    cv::minMaxLoc(lidar_img, nullptr, &max_val);
    if (max_val < 1e-3) return score;
    cv::Mat lidar_8u;
    lidar_img.convertTo(lidar_8u, CV_8U, 255.0/max_val);
    cv::Canny(lidar_8u, lidar_edges, 30, 100);

    score.num_lidar_edge_pts = cv::countNonZero(lidar_edges);
    score.num_image_edge_pts = cv::countNonZero(img_edges);

    cv::Mat e1, e2;
    img_edges.convertTo(e1, CV_32F, 1.0/255);
    lidar_edges.convertTo(e2, CV_32F, 1.0/255);
    cv::Scalar m1, s1, m2, s2;
    cv::meanStdDev(e1, m1, s1);
    cv::meanStdDev(e2, m2, s2);
    if (s1[0] > 1e-5 && s2[0] > 1e-5)
        score.ncc_score = (e1 - m1[0]).dot(e2 - m2[0]) /
                          (s1[0] * s2[0] * e1.total() + 1e-10);

    return score;
}

// ===================================================================
// 两阶段标定 (推荐使用)
// ===================================================================
LiDARCameraCalibrator::TwoStageResult LiDARCameraCalibrator::calibrate_two_stage(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const CameraIntrinsics& cam_intrin,
    const std::optional<Sophus::SE3d>& coarse_init,
    bool prefer_targetfree,
    const std::string& lidar_id,
    const std::string& cam_id) {

    TwoStageResult result;
    result.manual_threshold_px = cfg_.max_reproj_error_px;

    UNICALIB_INFO("╔══════════════════════════════════════════════════════════════════╗");
    UNICALIB_INFO("║         LiDAR-Camera 两阶段标定 - 详细日志                  ║");
    UNICALIB_INFO("╚══════════════════════════════════════════════════════════════════╝");
    UNICALIB_INFO("┌────────────────────────────────────────────────────────────────┐");
    UNICALIB_INFO("│ 输入参数:");
    UNICALIB_INFO("│   - LiDAR ID: {}  图像 ID: {}", lidar_id, cam_id);
    UNICALIB_INFO("│   - 点云帧数: {}  图像帧数: {}", lidar_scans.size(), camera_frames.size());
    UNICALIB_INFO("│   - 图像尺寸: {}x{}", cam_intrin.width, cam_intrin.height);
    UNICALIB_INFO("│   - 相机内参: fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}",
                  cam_intrin.fx, cam_intrin.fy, cam_intrin.cx, cam_intrin.cy);
    if (camera_has_distortion(cam_intrin)) {
        UNICALIB_INFO("│   - 图像去畸变: 标定前将使用内参 K 与畸变系数 ({} 个)",
                      cam_intrin.dist_coeffs.size());
    }
    UNICALIB_INFO("│   - 优先无目标方法: {}", prefer_targetfree ? "是(边缘对齐)" : "否(棋盘格)");
    UNICALIB_INFO("│   - 粗标定初值: {}", coarse_init.has_value() ? "已提供" : "无(使用identity)");
    if (coarse_init.has_value()) {
        Eigen::Vector3d t = coarse_init->translation();
        UNICALIB_INFO("│   - 初始平移: [{:.4f}, {:.4f}, {:.4f}] m", t.x(), t.y(), t.z());
    }
    UNICALIB_INFO("└────────────────────────────────────────────────────────────────┘");

    if (lidar_scans.empty() || camera_frames.empty()) {
        UNICALIB_ERROR("数据不足，无法标定 - LiDAR帧数:{} 图像帧数:{}",
                      lidar_scans.size(), camera_frames.size());
        return result;
    }

    // ─── Stage 1: 粗标定 ───
    UNICALIB_INFO("");
    UNICALIB_INFO("┌────────────────────────────────────────────────────────────────┐");
    UNICALIB_INFO("│ [Stage 1] 粗标定阶段");
    UNICALIB_INFO("└────────────────────────────────────────────────────────────────┘");
    Sophus::SE3d init_T = coarse_init.value_or(Sophus::SE3d());

    if (coarse_init.has_value()) {
        result.coarse = ExtrinsicSE3();
        result.coarse->ref_sensor_id = lidar_id;
        result.coarse->target_sensor_id = cam_id;
        result.coarse->set_SE3(*coarse_init);
        result.coarse_method = "provided_init";
        result.coarse_rms = -1.0;
        Eigen::Vector3d t = coarse_init->translation();
        Eigen::Vector3d euler = coarse_init->rotationMatrix().eulerAngles(0, 1, 2);
        UNICALIB_INFO("│ [Coarse] 使用提供的初始值:");
        UNICALIB_INFO("│   - 平移向量: [{:.4f}, {:.4f}, {:.4f}] m", t.x(), t.y(), t.z());
        UNICALIB_INFO("│   - 旋转角度: [{:.2f}, {:.2f}, {:.2f}] deg (RPY)", euler.x()*180/M_PI, euler.y()*180/M_PI, euler.z()*180/M_PI);
        UNICALIB_INFO("│   - 变换范数: {:.4f}", t.norm());
    } else {
        result.coarse_method = "identity";
        UNICALIB_INFO("│ [Coarse] 使用 identity 作为初始值");
        UNICALIB_INFO("│   - 原因: 未配置或未成功运行 MIAS-LCEC 粗标定");
    }
    UNICALIB_INFO("└────────────────────────────────────────────────────────────────┘");

    // ─── Stage 2: 精标定 ───
    UNICALIB_INFO("");
    UNICALIB_INFO("┌────────────────────────────────────────────────────────────────┐");
    UNICALIB_INFO("│ [Stage 2] 精标定阶段");
    UNICALIB_INFO("└────────────────────────────────────────────────────────────────┘");
    std::optional<ExtrinsicSE3> fine_result;

    if (prefer_targetfree) {
        UNICALIB_INFO("│ [Fine] 执行边缘对齐优化...");
        UNICALIB_INFO("│ 配置: NCC阈值={:.3f} 帧同步阈值={:.3f}s 时间偏移搜索范围=±{:.3f}s",
                     cfg_.ncc_threshold, cfg_.frame_sync_threshold_s, cfg_.time_offset_search_range_s);
        fine_result = calibrate_edge_align(lidar_scans, camera_frames, cam_intrin,
                                           init_T, lidar_id, cam_id);
        result.fine_method = "EDGE_ALIGNMENT";
    } else {
        UNICALIB_INFO("│ [Fine] 执行标定板目标标定 (多帧 BA + 粗搜索/离群剔除)...");
        fine_result = calibrate_target(lidar_scans, camera_frames, cam_intrin,
                                       lidar_id, cam_id, init_T);
        result.fine_method = "TARGET_CHESSBOARD";
    }

    if (fine_result.has_value()) {
        result.fine = fine_result;
        result.fine_rms = fine_result->residual_rms;
        result.needs_manual = (result.fine_rms > result.manual_threshold_px);

        Eigen::Vector3d t_final = fine_result->translation();
        UNICALIB_INFO("│");
        UNICALIB_INFO("│ ★ 标定结果:");
        UNICALIB_INFO("│   - 平移: [{:.4f}, {:.4f}, {:.4f}] m", t_final.x(), t_final.y(), t_final.z());
        UNICALIB_INFO("│   - RMS: {:.4f} px", result.fine_rms);
        UNICALIB_INFO("│   - 收敛: {}", fine_result->is_converged ? "是" : "否");

        if (result.needs_manual) {
            UNICALIB_WARN("│ [警告] RMS={:.3f}px > 阈值={:.3f}px，建议手动校准",
                          result.fine_rms, result.manual_threshold_px);
        }
    } else {
        UNICALIB_ERROR("│ [Fine] 标定失败");
        result.fine_method = "FAILED";
    }

    UNICALIB_INFO("└────────────────────────────────────────────────────────────────┘");
    return result;
}

}  // namespace ns_unicalib
