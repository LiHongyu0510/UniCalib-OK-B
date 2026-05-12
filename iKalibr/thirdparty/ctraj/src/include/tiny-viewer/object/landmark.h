#ifndef TINY_VIEWER_LANDMARK_H
#define TINY_VIEWER_LANDMARK_H

#include <Eigen/Dense>

namespace tiny_viewer {

struct Landmark {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    Landmark() = default;
};

} // namespace tiny_viewer

#endif
