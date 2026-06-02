#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <string>

namespace ns_unicalib {

struct Frame {
    double timestamp = 0.0;
    cv::Mat image;
    std::string camera_id;
};

struct ImuData {
    double timestamp = 0.0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

} // namespace ns_unicalib
