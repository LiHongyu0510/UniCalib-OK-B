#ifndef VETA_STUB_PINHOLE_H
#define VETA_STUB_PINHOLE_H

// veta/camera/pinhole.h — Camera intrinsic API backed by aslam_cv2 distortion math.
// Replaces the external veta library; exposes the ns_veta::PinholeIntrinsic interface
// that iKalibr uses throughout its source code.
//
// aslam_cv2 reference math:
//   RadTan   → aslam::RadTanDistortion  (k1, k2, p1, p2)
//   Fisheye  → aslam::EquidistantDistortion  (k1, k2, k3, k4)
// dist array for BrownT2: [k1, k2, k3, p1, p2]  (indices 0-4)
// dist array for Fisheye: [k1, k2, k3, k4]       (indices 0-3)
// GetParams() returns: [fx, fy, cx, cy, ...dist]

#include "../type_def.hpp"
#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

namespace ns_veta {

// ─── Distortion math (aslam_cv2-compatible, double only) ────────────────────

namespace detail {

// ── RadTan / Brown-Conrady forward distortion ────────────────────────────────
// Matches aslam::RadTanDistortion::distortUsingExternalCoefficients
// params: k1, k2, k3 (radial), p1, p2 (tangential)
inline void ApplyRadTanD(double& x, double& y,
                         double k1, double k2, double k3,
                         double p1, double p2) {
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    x = x * radial + dx;
    y = y * radial + dy;
}

// ── RadTan inverse (Newton iteration) ────────────────────────────────────────
// Matches aslam::RadTanDistortion::undistortUsingExternalCoefficients
inline void UndistortRadTanD(double& x, double& y,
                              double k1, double k2, double k3,
                              double p1, double p2) {
    const double x0 = x, y0 = y;
    for (int i = 0; i < 100; ++i) {
        double r2 = x * x + y * y;
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        // Approximate step using Jacobian diagonal
        double denom_x = radial + 2.0 * (k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4) * x * x + 1e-10;
        double denom_y = radial + 2.0 * (k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4) * y * y + 1e-10;
        x -= (x * radial + dx - x0) / denom_x;
        y -= (y * radial + dy - y0) / denom_y;
    }
}

// ── Equidistant / Kannala-Brandt forward distortion ──────────────────────────
// Matches aslam::EquidistantDistortion::distortUsingExternalCoefficients
// params: k1, k2, k3, k4
inline void ApplyEquidistantD(double& x, double& y,
                               double k1, double k2, double k3, double k4) {
    double r = std::sqrt(x * x + y * y);
    if (r < 1e-9) return;
    double theta = std::atan(r);
    double theta2 = theta * theta;
    double theta_d = theta * (1.0 + k1 * theta2
                                   + k2 * theta2 * theta2
                                   + k3 * theta2 * theta2 * theta2
                                   + k4 * theta2 * theta2 * theta2 * theta2);
    double scale = theta_d / r;
    x *= scale;
    y *= scale;
}

// ── Equidistant inverse (Newton iteration) ────────────────────────────────────
inline void UndistortEquidistantD(double& x, double& y,
                                   double k1, double k2, double k3, double k4) {
    double r_d = std::sqrt(x * x + y * y);
    if (r_d < 1e-9) return;
    // Solve: theta_d(theta) = r_d  via Newton
    double theta = r_d;
    for (int i = 0; i < 100; ++i) {
        double theta2 = theta * theta;
        double theta4 = theta2 * theta2;
        double fn = theta * (1.0 + k1 * theta2 + k2 * theta4
                             + k3 * theta2 * theta4
                             + k4 * theta4 * theta4) - r_d;
        double fn_d = 1.0 + 3.0 * k1 * theta2
                          + 5.0 * k2 * theta4
                          + 7.0 * k3 * theta2 * theta4
                          + 9.0 * k4 * theta4 * theta4;
        double d_theta = fn / (fn_d + 1e-10);
        theta -= d_theta;
        if (std::fabs(d_theta) < 1e-10) break;
    }
    double scale = std::tan(theta) / (r_d + 1e-10);
    x *= scale;
    y *= scale;
}

}  // namespace detail

// ─── PinholeIntrinsic — base (no distortion) ─────────────────────────────────

class PinholeIntrinsic {
 public:
  using Ptr = std::shared_ptr<PinholeIntrinsic>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix3d K;        // Intrinsic matrix
  Eigen::VectorXd dist;     // Distortion parameters (model-specific ordering)
  int imgWidth  = 0;
  int imgHeight = 0;

  PinholeIntrinsic() { K = Eigen::Matrix3d::Identity(); }
  virtual ~PinholeIntrinsic() = default;

  // ── Coordinate transforms ────────────────────────────────────────────────
  inline Vec2d ImgToCam(const Vec2d& p) const {
    return {(p(0) - K(0, 2)) / K(0, 0), (p(1) - K(1, 2)) / K(1, 1)};
  }
  inline Vec2d CamToImg(const Vec2d& p) const {
    return {K(0, 0) * p(0) + K(0, 2), K(1, 1) * p(1) + K(1, 2)};
  }

  // ── Distortion (override in derived classes) ─────────────────────────────
  virtual Vec2d RemoveDisto(const Vec2d& p) const { return p; }
  virtual Vec2d AddDisto(const Vec2d& p)    const { return p; }

  inline Vec2d GetUndistoPixel(const Vec2d& p) const {
    return CamToImg(RemoveDisto(ImgToCam(p)));
  }
  inline Vec2d GetDistoPixel(const Vec2d& p) const {
    return CamToImg(AddDisto(ImgToCam(p)));
  }

  // ── Accessors ────────────────────────────────────────────────────────────
  double FocalX() const { return K(0, 0); }
  double FocalY() const { return K(1, 1); }
  Eigen::Vector2d PrincipalPoint() const { return {K(0, 2), K(1, 2)}; }

  // Raw parameter pointers for Ceres optimization
  double*       FXAddress()       { return &K(0, 0); }
  double*       FYAddress()       { return &K(1, 1); }
  double*       CXAddress()       { return &K(0, 2); }
  double*       CYAddress()       { return &K(1, 2); }
  const double* FXAddress() const { return &K(0, 0); }
  const double* FYAddress() const { return &K(1, 1); }
  const double* CXAddress() const { return &K(0, 2); }
  const double* CYAddress() const { return &K(1, 2); }

  double ImagePlaneToCameraPlaneError(double e) const {
    if (K(0,0) <= 0 || K(1,1) <= 0) return e;
    return e * 0.5 * (1.0 / K(0,0) + 1.0 / K(1,1));
  }

  // GetParams() → [fx, fy, cx, cy, dist[0], dist[1], ...]
  virtual std::vector<double> GetParams() const {
    std::vector<double> p = {K(0,0), K(1,1), K(0,2), K(1,2)};
    for (int i = 0; i < dist.size(); ++i) p.push_back(dist(i));
    return p;
  }

  // ── Factories ────────────────────────────────────────────────────────────
  static Ptr Create() { return std::make_shared<PinholeIntrinsic>(); }

  static Ptr Create(double fx, double fy, double cx, double cy, int w, int h) {
    auto p = std::make_shared<PinholeIntrinsic>();
    p->K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    p->imgWidth = w;
    p->imgHeight = h;
    return p;
  }

  // ── Serialization ────────────────────────────────────────────────────────
  template <class Archive>
  void serialize(Archive& ar) {
    double fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
    ar(cereal::make_nvp("fx", fx), cereal::make_nvp("fy", fy),
       cereal::make_nvp("cx", cx), cereal::make_nvp("cy", cy),
       cereal::make_nvp("width",  imgWidth),
       cereal::make_nvp("height", imgHeight));
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    std::vector<double> dv(dist.data(), dist.data() + dist.size());
    ar(cereal::make_nvp("dists", dv));
    dist = Eigen::Map<Eigen::VectorXd>(dv.data(), static_cast<Eigen::Index>(dv.size()));
  }
};

using PinholeIntrinsicPtr = PinholeIntrinsic::Ptr;

// ─── PinholeIntrinsicBrownT2  (Brown-Conrady / RadTan distortion) ────────────
// dist layout: [k1, k2, k3, p1, p2]  (indices 0..4)
// GetParams → [fx, fy, cx, cy, k1, k2, k3, p1, p2]
//   ↑ iKalibr accesses par[4]=k1 par[5]=k2 par[6]=k3 par[7]=p1 par[8]=p2

class PinholeIntrinsicBrownT2 : public PinholeIntrinsic {
 public:
  using Ptr = std::shared_ptr<PinholeIntrinsicBrownT2>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vec2d RemoveDisto(const Vec2d& p) const override {
    if (dist.size() < 4) return p;
    double x = p(0), y = p(1);
    double k1 = dist(0), k2 = dist(1);
    double k3 = dist.size() > 2 ? dist(2) : 0.0;
    double p1 = dist.size() > 3 ? dist(3) : 0.0;
    double p2 = dist.size() > 4 ? dist(4) : 0.0;
    detail::UndistortRadTanD(x, y, k1, k2, k3, p1, p2);
    return {x, y};
  }

  Vec2d AddDisto(const Vec2d& p) const override {
    if (dist.size() < 4) return p;
    double x = p(0), y = p(1);
    double k1 = dist(0), k2 = dist(1);
    double k3 = dist.size() > 2 ? dist(2) : 0.0;
    double p1 = dist.size() > 3 ? dist(3) : 0.0;
    double p2 = dist.size() > 4 ? dist(4) : 0.0;
    detail::ApplyRadTanD(x, y, k1, k2, k3, p1, p2);
    return {x, y};
  }

  // [fx, fy, cx, cy, k1, k2, k3, p1, p2]
  std::vector<double> GetParams() const override {
    return {K(0,0), K(1,1), K(0,2), K(1,2),
            dist.size() > 0 ? dist(0) : 0.0,   // k1
            dist.size() > 1 ? dist(1) : 0.0,   // k2
            dist.size() > 2 ? dist(2) : 0.0,   // k3
            dist.size() > 3 ? dist(3) : 0.0,   // p1
            dist.size() > 4 ? dist(4) : 0.0};  // p2
  }

  // Raw distortion parameter pointers for Ceres
  double* K1Address() { return dist.size() > 0 ? &dist(0) : nullptr; }
  double* K2Address() { return dist.size() > 1 ? &dist(1) : nullptr; }
  double* K3Address() { return dist.size() > 2 ? &dist(2) : nullptr; }
  double* P1Address() { return dist.size() > 3 ? &dist(3) : nullptr; }
  double* P2Address() { return dist.size() > 4 ? &dist(4) : nullptr; }

  static Ptr Create() {
    auto p = std::make_shared<PinholeIntrinsicBrownT2>();
    p->dist = Eigen::VectorXd::Zero(5);  // k1, k2, k3, p1, p2
    return p;
  }

  template <class Archive>
  void serialize(Archive& ar) { PinholeIntrinsic::serialize(ar); }
};

using PinholeBrownIntrinsic    = PinholeIntrinsicBrownT2;
using PinholeBrownIntrinsicPtr = PinholeIntrinsicBrownT2::Ptr;

}  // namespace ns_veta

#endif  // VETA_STUB_PINHOLE_H
