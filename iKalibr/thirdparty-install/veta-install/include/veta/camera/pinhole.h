#ifndef VETA_STUB_PINHOLE_H
#define VETA_STUB_PINHOLE_H

#include "../type_def.hpp"
#include <memory>
#include <Eigen/Dense>

namespace ns_veta {

class PinholeIntrinsic {
 public:
  using Ptr = std::shared_ptr<PinholeIntrinsic>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d K;
  Eigen::VectorXd dist;  // distortion params

  PinholeIntrinsic() : K(Eigen::Matrix3d::Identity()), dist(Eigen::VectorXd(0)) {}

  inline Vec2d ImgToCam(const Vec2d& p) const {
    Vec2d pn;
    pn(0) = (p(0) - K(0, 2)) / K(0, 0);
    pn(1) = (p(1) - K(1, 2)) / K(1, 1);
    return pn;
  }

  inline Vec2d CamToImg(const Vec2d& p) const {
    return Vec2d(K(0, 0) * p(0) + K(0, 2), K(1, 1) * p(1) + K(1, 2));
  }

  static Ptr Create() { return std::make_shared<PinholeIntrinsic>(); }
};

using PinholeIntrinsicPtr = PinholeIntrinsic::Ptr;

}  // namespace ns_veta

#endif
