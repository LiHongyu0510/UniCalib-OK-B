#ifndef VETA_STUB_PINHOLE_FISHEYE_H
#define VETA_STUB_PINHOLE_FISHEYE_H

// veta/camera/pinhole_fisheye.h — Fisheye (Kannala-Brandt/Equidistant) camera model.
// Backed by aslam::EquidistantDistortion math.
// GetParams order: [fx, fy, cx, cy, k1, k2, k3, k4]

#include "pinhole.h"

namespace ns_veta {

class PinholeIntrinsicFisheye : public PinholeIntrinsic {
 public:
  using Ptr = std::shared_ptr<PinholeIntrinsicFisheye>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vec2d RemoveDisto(const Vec2d& p) const override {
    if (dist.size() < 4) return p;
    double x = p(0), y = p(1);
    detail::UndistortEquidistantD(x, y, dist(0), dist(1), dist(2), dist(3));
    return {x, y};
  }

  Vec2d AddDisto(const Vec2d& p) const override {
    if (dist.size() < 4) return p;
    double x = p(0), y = p(1);
    detail::ApplyEquidistantD(x, y, dist(0), dist(1), dist(2), dist(3));
    return {x, y};
  }

  // [fx, fy, cx, cy, k1, k2, k3, k4]
  std::vector<double> GetParams() const override {
    return {K(0,0), K(1,1), K(0,2), K(1,2),
            dist.size() > 0 ? dist(0) : 0,
            dist.size() > 1 ? dist(1) : 0,
            dist.size() > 2 ? dist(2) : 0,
            dist.size() > 3 ? dist(3) : 0};
  }

  double* K1Address() { return &dist(0); }
  double* K2Address() { return &dist(1); }
  double* K3Address() { return dist.size() > 2 ? &dist(2) : nullptr; }
  double* K4Address() { return dist.size() > 3 ? &dist(3) : nullptr; }

  static Ptr Create() {
    auto p = std::make_shared<PinholeIntrinsicFisheye>();
    p->dist = Eigen::VectorXd::Zero(4);  // k1,k2,k3,k4
    return p;
  }

  template <class Archive>
  void serialize(Archive& ar) { PinholeIntrinsic::serialize(ar); }
};

using PinholeFisheyeIntrinsic = PinholeIntrinsicFisheye;
using PinholeFisheyeIntrinsicPtr = PinholeIntrinsicFisheye::Ptr;

}  // namespace ns_veta

#endif  // VETA_STUB_PINHOLE_FISHEYE_H
