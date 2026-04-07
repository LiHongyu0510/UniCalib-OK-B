#ifndef VETA_STUB_TYPE_DEF_HPP
#define VETA_STUB_TYPE_DEF_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>  // EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION
#include <cstdint>

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
};

}  // namespace ns_veta

// Eigen STL vector specializations (required before std::vector<Vector2d> etc.)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)

#endif
