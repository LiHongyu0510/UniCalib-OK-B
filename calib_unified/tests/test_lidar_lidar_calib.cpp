/**
 * UniCalib — LiDAR-LiDAR 标定单元测试
 * 验证粗标定与精标定流程可运行，并对合成数据恢复已知外参
 */
#include <gtest/gtest.h>
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/extrinsic/lidar_lidar_calib.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>

namespace ns_unicalib {

// 生成简单合成点云 (平面网格 + 少许高度)
static void make_synthetic_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, size_t n) {
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> u(-5.0, 5.0);
    for (size_t i = 0; i < n; ++i) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(u(gen));
        p.y = static_cast<float>(u(gen));
        p.z = static_cast<float>(u(gen) * 0.5);
        p.intensity = 100.f;
        cloud->push_back(p);
    }
}

TEST(LidarLidarCalib, TwoStageRunsWithoutCrash) {
    LiDARLiDARConfig cfg;
    cfg.voxel_size = 0.2;
    cfg.max_frames = 5;
    cfg.gicp_max_iterations = 10;
    cfg.ndt_max_iterations = 15;
    cfg.verbose = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw;
    make_synthetic_cloud(raw, 500);
    std::vector<LiDARScan> scans_ref(3), scans_target(3);
    for (int i = 0; i < 3; ++i) {
        scans_ref[i].cloud = raw;
        scans_ref[i].timestamp = i * 0.1;
        scans_target[i].cloud = raw;  // 同一点云，外参应为单位阵附近
        scans_target[i].timestamp = i * 0.1;
    }

    LiDARLiDARCalibrator calib(cfg);
    auto result = calib.calibrate_two_stage(scans_ref, scans_target, "ref", "tgt", std::nullopt);
    EXPECT_TRUE(result.best() != nullptr) << "两阶段标定应返回有效结果";
    if (result.best()) {
        const auto& T = result.best()->SE3_TargetInRef();
        double t_norm = T.translation().norm();
        EXPECT_LT(t_norm, 2.0) << "同源点云标定平移应接近 0";
    }
}

TEST(LidarLidarCalib, RecoverKnownTransform) {
    LiDARLiDARConfig cfg;
    cfg.voxel_size = 0.15;
    cfg.max_frames = 5;
    cfg.gicp_max_iterations = 25;
    cfg.ndt_max_iterations = 20;
    cfg.gicp_max_corr_dist = 0.5;
    cfg.verbose = false;

    Eigen::AngleAxisd rot(M_PI / 6, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d trans(0.5, 0.2, 0.1);
    Sophus::SE3d T_known(rot.toRotationMatrix(), trans);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ref_cloud;
    make_synthetic_cloud(ref_cloud, 600);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f Tf = T_known.inverse().matrix().cast<float>();
    pcl::transformPointCloud(*ref_cloud, *tgt_cloud, Tf);

    std::vector<LiDARScan> scans_ref(2), scans_target(2);
    for (int i = 0; i < 2; ++i) {
        scans_ref[i].cloud = ref_cloud;
        scans_ref[i].timestamp = i * 0.1;
        scans_target[i].cloud = tgt_cloud;
        scans_target[i].timestamp = i * 0.1;
    }

    LiDARLiDARCalibrator calib(cfg);
    auto result = calib.calibrate_two_stage(scans_ref, scans_target, "ref", "tgt", std::nullopt);
    ASSERT_TRUE(result.best() != nullptr);
    Sophus::SE3d T_recovered = result.best()->SE3_TargetInRef();
    Sophus::SE3d err = T_known * T_recovered.inverse();
    double trans_err = err.translation().norm();
    double rot_err_rad = err.so3().log().norm();
    EXPECT_LT(trans_err, 0.15) << "平移误差应 < 15cm";
    EXPECT_LT(rot_err_rad * 180.0 / M_PI, 5.0) << "旋转误差应 < 5deg";
}

}  // namespace ns_unicalib
