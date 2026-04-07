/**
 * UniCalib — LiDAR 点云投影到图像 (SensorsCalibration 风格)
 *
 * 移植自 PJLab-ADG/SensorsCalibration lidar2camera/manual_calib Projector，
 * 适配 UniCalib 的 CameraIntrinsics 与 T_lidar_to_cam 外参。
 * 支持：去畸变、按深度/强度着色、重叠过滤、点大小。
 */

#pragma once

#include "unicalib/common/calib_param.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace ns_unicalib {

#ifndef OVERLAP_FILTER_WINDOW
#define OVERLAP_FILTER_WINDOW 4
#endif
#ifndef OVERLAP_DEPTH_TH
#define OVERLAP_DEPTH_TH 0.4
#endif

struct ProjectorPt {
    cv::Point point;
    float dist;
    float z;
    float intensity;
};

class LidarProjector {
public:
    int point_size_ = 3;
    bool intensity_color_ = false;
    bool overlap_filter_ = false;

    void setPointSize(int size) { point_size_ = size; }
    void setDisplayMode(bool intensity_show) { intensity_color_ = intensity_show; }
    void setFilterMode(bool filter_mode) { overlap_filter_ = filter_mode; }

    bool loadPointCloud(const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud) {
        oriCloud_ = cv::Mat(cv::Size(static_cast<int>(pcl_cloud.points.size()), 3), CV_32FC1);
        intensitys_.clear();
        for (size_t i = 0; i < pcl_cloud.points.size(); ++i) {
            oriCloud_.at<float>(0, static_cast<int>(i)) = pcl_cloud.points[i].x;
            oriCloud_.at<float>(1, static_cast<int>(i)) = pcl_cloud.points[i].y;
            oriCloud_.at<float>(2, static_cast<int>(i)) = pcl_cloud.points[i].z;
            intensitys_.push_back(pcl_cloud.points[i].intensity);
        }
        return true;
    }

    static cv::Scalar fakeColor(float value) {
        const float posSlope = 255.f / 60.f;
        const float negSlope = -255.f / 60.f;
        value *= 255.f;
        cv::Vec3f color;
        if (value < 60) {
            color[0] = 255; color[1] = posSlope * value + 0; color[2] = 0;
        } else if (value < 120) {
            color[0] = negSlope * value + 2 * 255; color[1] = 255; color[2] = 0;
        } else if (value < 180) {
            color[0] = 0; color[1] = 255; color[2] = posSlope * value - 2 * 255;
        } else if (value < 240) {
            color[0] = 0; color[1] = negSlope * value + 4 * 255; color[2] = 255;
        } else if (value < 300) {
            color[0] = posSlope * value - 4 * 255; color[1] = 0; color[2] = 255;
        } else {
            color[0] = 255; color[1] = 0; color[2] = negSlope * value + 6 * 255;
        }
        return cv::Scalar(color[0], color[1], color[2]);
    }

    /**
     * 投影到图像。T_lidar_to_cam 为 4x4，即 p_cam = R * p_lidar + t。
     * 内参与畸变来自 CameraIntrinsics；图像会先做去畸变再叠加点云。
     */
    cv::Mat projectToImage(const cv::Mat& img,
                           const CameraIntrinsics& intrin,
                           const Eigen::Matrix4d& T_lidar_to_cam) {
        std::vector<double> D = intrin.dist_coeffs;
        if (D.size() < 4) D.resize(4, 0.0);
        if (D.size() < 8) D.resize(8, 0.0);

        Eigen::Matrix3d K = intrin.K();
        cv::Mat K1 = (cv::Mat_<float>(3, 3) <<
            static_cast<float>(K(0,0)), static_cast<float>(K(0,1)), static_cast<float>(K(0,2)),
            static_cast<float>(K(1,0)), static_cast<float>(K(1,1)), static_cast<float>(K(1,2)),
            static_cast<float>(K(2,0)), static_cast<float>(K(2,1)), static_cast<float>(K(2,2)));
        cv::Mat D1(static_cast<int>(D.size()), 1, CV_32FC1);
        for (size_t i = 0; i < D.size(); ++i) D1.at<float>(static_cast<int>(i), 0) = static_cast<float>(D[i]);

        cv::Mat R1 = (cv::Mat_<float>(3, 3) <<
            static_cast<float>(T_lidar_to_cam(0,0)), static_cast<float>(T_lidar_to_cam(0,1)), static_cast<float>(T_lidar_to_cam(0,2)),
            static_cast<float>(T_lidar_to_cam(1,0)), static_cast<float>(T_lidar_to_cam(1,1)), static_cast<float>(T_lidar_to_cam(1,2)),
            static_cast<float>(T_lidar_to_cam(2,0)), static_cast<float>(T_lidar_to_cam(2,1)), static_cast<float>(T_lidar_to_cam(2,2)));
        cv::Mat T1 = (cv::Mat_<float>(3, 1) <<
            static_cast<float>(T_lidar_to_cam(0,3)),
            static_cast<float>(T_lidar_to_cam(1,3)),
            static_cast<float>(T_lidar_to_cam(2,3)));

        return projectToRawMat(img, K1, D1, R1, T1);
    }

private:
    cv::Mat oriCloud_;
    std::vector<float> intensitys_;

    cv::Mat projectToRawMat(cv::Mat img, cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat T) {
        if (oriCloud_.empty() || oriCloud_.cols == 0) return img.clone();

        cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
        cv::Mat mapX, mapY;
        cv::Mat img_bgr;
        if (img.channels() == 1)
            cv::cvtColor(img, img_bgr, cv::COLOR_GRAY2BGR);
        else
            img_bgr = img.clone();

        cv::initUndistortRectifyMap(K, D, I, K, img.size(), CV_32FC1, mapX, mapY);
        cv::Mat outImg(img.size(), CV_8UC3);
        cv::remap(img_bgr, outImg, mapX, mapY, cv::INTER_LINEAR);

        cv::Mat dist = oriCloud_.rowRange(0, 1).mul(oriCloud_.rowRange(0, 1)) +
                       oriCloud_.rowRange(1, 2).mul(oriCloud_.rowRange(1, 2)) +
                       oriCloud_.rowRange(2, 3).mul(oriCloud_.rowRange(2, 3));

        cv::Mat projCloud2d = K * (R * oriCloud_ + repeat(T, 1, oriCloud_.cols));
        float maxDist = 0, maxIntensity = 0;
        std::vector<ProjectorPt> points;
        std::vector<std::vector<int>> filter_pts(img.rows, std::vector<int>(img.cols, -1));

        for (int i = 0; i < projCloud2d.cols; ++i) {
            float x = projCloud2d.at<float>(0, i);
            float y = projCloud2d.at<float>(1, i);
            float z = projCloud2d.at<float>(2, i);
            int x2d = cvRound(x / z);
            int y2d = cvRound(y / z);
            float d = std::sqrt(dist.at<float>(0, i));
            float intensity = intensitys_[i];

            if (x2d >= 0 && y2d >= 0 && x2d < img.cols && y2d < img.rows && z > 0) {
                maxDist = std::max(maxDist, d);
                maxIntensity = std::max(maxIntensity, intensity);
                points.push_back(ProjectorPt{cv::Point(x2d, y2d), d, z, intensity});
                if (filter_pts[y2d][x2d] != -1) {
                    int p_idx = filter_pts[y2d][x2d];
                    if (z < points[p_idx].z) filter_pts[y2d][x2d] = static_cast<int>(points.size()) - 1;
                } else {
                    filter_pts[y2d][x2d] = static_cast<int>(points.size()) - 1;
                }
            }
        }
        if (maxIntensity <= 0) maxIntensity = 1.f;
        if (maxDist <= 0) maxDist = 1.f;

        if (overlap_filter_) {
            std::vector<int> filtered_idxes;
            for (int m = 0; m < img.rows; m++) {
                for (int n = 0; n < img.cols; n++) {
                    int current_idx = filter_pts[m][n];
                    if (current_idx == -1) continue;
                    bool front = true;
                    for (int j = std::max(0, m - OVERLAP_FILTER_WINDOW);
                         j < std::min(img.rows, m + OVERLAP_FILTER_WINDOW + 1); j++) {
                        for (int k = std::max(0, n - OVERLAP_FILTER_WINDOW);
                             k < std::min(img.cols, n + OVERLAP_FILTER_WINDOW + 1); k++) {
                            if (filter_pts[j][k] == -1) continue;
                            int p_idx = filter_pts[j][k];
                            if (points[current_idx].z - points[p_idx].z > OVERLAP_DEPTH_TH) {
                                front = false;
                                break;
                            }
                        }
                    }
                    if (front) filtered_idxes.push_back(current_idx);
                }
            }
            for (int pt_idx : filtered_idxes) {
                cv::Scalar color = intensity_color_
                    ? fakeColor(points[pt_idx].intensity / maxIntensity)
                    : fakeColor(points[pt_idx].dist / maxDist);
                cv::circle(outImg, points[pt_idx].point, point_size_, color, -1);
            }
        } else {
            std::sort(points.begin(), points.end(),
                      [](const ProjectorPt& a, const ProjectorPt& b) { return a.dist > b.dist; });
            for (size_t i = 0; i < points.size(); ++i) {
                cv::Scalar color = intensity_color_
                    ? fakeColor(points[i].intensity / maxIntensity)
                    : fakeColor(points[i].dist / maxDist);
                cv::circle(outImg, points[i].point, point_size_, color, -1);
            }
        }

        return outImg;
    }
};

}  // namespace ns_unicalib
