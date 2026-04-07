#ifndef VETA_STUB_TYPE_DEF_HPP
#define VETA_STUB_TYPE_DEF_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>  // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION
#include <cstdint>
#include <type_traits>

namespace ns_veta {

using IndexT = std::uint32_t;
constexpr IndexT UndefinedIndexT = static_cast<IndexT>(-1);

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;

struct Posed {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d so3;
  Eigen::Vector3d translation;
  Posed() : so3(Eigen::Matrix3d::Identity()), translation(Eigen::Vector3d::Zero()) {}
  Posed(const Eigen::Matrix3d& r, const Eigen::Vector3d& t) : so3(r), translation(t) {}
  // Template constructor: accepts Sophus::SO3, Sophus::SO3d, or any type with .matrix() → Matrix3d
  template <typename SO3Type,
            typename = std::enable_if_t<!std::is_same<std::decay_t<SO3Type>, Eigen::Matrix3d>::value>>
  Posed(const SO3Type& r, const Eigen::Vector3d& t) : so3(r.matrix()), translation(t) {}
  // Accept Eigen::Map or Ref to translation
  template <typename SO3Type, typename VecType,
            typename = std::enable_if_t<!std::is_same<std::decay_t<SO3Type>, Eigen::Matrix3d>::value>>
  Posed(const SO3Type& r, const VecType& t) : so3(r.matrix()), translation(t) {}
  Eigen::Matrix3d Rotation() const { return so3; }
  Eigen::Vector3d& Translation() { return translation; }
  const Eigen::Vector3d& Translation() const { return translation; }
  Vec3d operator()(const Vec3d& p) const { return so3 * p + translation; }
  Posed Inverse() const {
    Eigen::Matrix3d Rt = so3.transpose();
    return Posed(Rt, -(Rt * translation));
  }
  // Pose composition: this * other (apply other first, then this)
  Posed operator*(const Posed& other) const {
    return Posed(so3 * other.so3, so3 * other.translation + translation);
  }
};

}  // namespace ns_veta

// Eigen STL vector specializations (required before std::vector<Vector2d> etc.)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)

#endif
