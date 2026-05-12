// ctraj-compat: Posed and OdomPose using Sophus only (no ctraj library)
#ifndef CTRAJ_ODOMETER_POSE_H
#define CTRAJ_ODOMETER_POSE_H

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include "ctraj/utils/macros.hpp"

#ifndef CEREAL_NVP
#define CEREAL_NVP(x) cereal::make_nvp(#x, x)
#endif

namespace ns_ctraj {

template <class ScalarType>
inline Eigen::Matrix<ScalarType, 3, 3> AdjustRotationMatrix(
    const Eigen::Matrix<ScalarType, 3, 3>& rotMat) {
  Eigen::JacobiSVD<Eigen::Matrix<ScalarType, 3, 3>> svd(
      rotMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
  const auto& vMatrix = svd.matrixV();
  const auto& uMatrix = svd.matrixU();
  return uMatrix * vMatrix.transpose();
}

template <class ScalarType>
struct OdomPose {
  double timeStamp;
  Eigen::Matrix<ScalarType, 4, 4> pose;

  explicit OdomPose(
      double timeStamp = INVALID_TIME_STAMP,
      const Eigen::Matrix<ScalarType, 4, 4>& pose =
          Eigen::Matrix<ScalarType, 4, 4>::Identity())
      : timeStamp(timeStamp), pose(pose) {}
};

using OdomPosed = OdomPose<double>;
using OdomPosef = OdomPose<float>;

template <class ScalarType>
struct Pose {
  using Scale = ScalarType;
  using Rotation = Sophus::SO3<Scale>;
  using Translation = Sophus::Vector3<Scale>;

  Rotation so3;
  Translation t;
  double timeStamp;

  Pose(const Rotation& so3, const Translation& t,
       double timeStamp = INVALID_TIME_STAMP)
      : so3(so3), t(t), timeStamp(timeStamp) {}

  explicit Pose(double timeStamp = INVALID_TIME_STAMP)
      : so3(), t(Translation::Zero()), timeStamp(timeStamp) {}

  Eigen::Quaternion<ScalarType> q() const {
    return so3.unit_quaternion();
  }

  Eigen::Matrix<ScalarType, 3, 3> R() const {
    return q().toRotationMatrix();
  }

  Sophus::SE3<ScalarType> se3() const {
    return Sophus::SE3<ScalarType>(so3, t);
  }

  Eigen::Matrix<ScalarType, 4, 4> T() const {
    Eigen::Matrix<ScalarType, 4, 4> T =
        Eigen::Matrix<ScalarType, 4, 4>::Identity();
    T.template block<3, 3>(0, 0) = R();
    T.template block<3, 1>(0, 3) = t;
    return T;
  }

  static Pose FromT(const Eigen::Matrix<ScalarType, 4, 4>& T,
                    double timeStamp = INVALID_TIME_STAMP) {
    Eigen::Matrix<ScalarType, 3, 3> rotMat = T.template block<3, 3>(0, 0);
    rotMat = AdjustRotationMatrix(rotMat);
    Pose pose(timeStamp);
    pose.so3 = Rotation(rotMat);
    pose.t = T.template block<3, 1>(0, 3);
    return pose;
  }

  static Pose FromRt(const Eigen::Matrix<ScalarType, 3, 3>& R,
                     const Eigen::Matrix<ScalarType, 3, 1>& t,
                     double timeStamp = INVALID_TIME_STAMP) {
    Eigen::Matrix<ScalarType, 3, 3> rotMat = AdjustRotationMatrix(R);
    Pose pose(timeStamp);
    pose.so3 = Rotation(rotMat);
    pose.t = t;
    return pose;
  }

  static Pose FromSE3(const Sophus::SE3<ScalarType>& se3,
                      double timeStamp = INVALID_TIME_STAMP) {
    Pose pose(timeStamp);
    pose.so3 = se3.so3();
    pose.t = se3.translation();
    return pose;
  }

  template <class Archive>
  void save(Archive& ar) const {
    ar(CEREAL_NVP(timeStamp));
    Eigen::Quaternion<ScalarType> q = so3.unit_quaternion();
    ar(cereal::make_nvp("qx", q.x()), cereal::make_nvp("qy", q.y()),
       cereal::make_nvp("qz", q.z()), cereal::make_nvp("qw", q.w()));
    ar(cereal::make_nvp("tx", t(0)), cereal::make_nvp("ty", t(1)),
       cereal::make_nvp("tz", t(2)));
  }
  template <class Archive>
  void load(Archive& ar) {
    ar(CEREAL_NVP(timeStamp));
    ScalarType qx, qy, qz, qw, tx, ty, tz;
    ar(cereal::make_nvp("qx", qx), cereal::make_nvp("qy", qy),
       cereal::make_nvp("qz", qz), cereal::make_nvp("qw", qw));
    ar(cereal::make_nvp("tx", tx), cereal::make_nvp("ty", ty),
       cereal::make_nvp("tz", tz));
    so3 = Rotation(Eigen::Quaternion<ScalarType>(qw, qx, qy, qz));
    t << tx, ty, tz;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Posed = Pose<double>;
using Posef = Pose<float>;

}  // namespace ns_ctraj

#endif
