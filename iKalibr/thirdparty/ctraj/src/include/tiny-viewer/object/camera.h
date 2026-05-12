#ifndef TINY_VIEWER_CAMERA_H
#define TINY_VIEWER_CAMERA_H

#include <Eigen/Dense>

namespace tiny_viewer {

struct Camera {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    Camera() = default;
};

} // namespace tiny_viewer

#endif
