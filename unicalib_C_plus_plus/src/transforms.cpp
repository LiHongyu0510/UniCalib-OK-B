/**
 * UniCalib C++ — 几何变换实现 (ZYX 欧拉角、四元数、SE3)
 */
#include "unicalib/transforms.hpp"
#include <cmath>
#include <random>
#include <algorithm>

namespace unicalib {

namespace {

const double kPi = 3.14159265358979323846;

// 旋转矩阵 → 四元数 [x,y,z,w]
Eigen::Vector4d rotation_to_quat(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

Eigen::Matrix3d quat_to_rotation(const Eigen::Vector4d& q) {
  return Eigen::Quaterniond(q(3), q(0), q(1), q(2)).toRotationMatrix();
}

}  // namespace

Eigen::Vector3d rotation_matrix_to_euler(const Eigen::Matrix3d& R, bool degrees) {
  double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  bool singular = sy < 1e-8;
  double x, y, z;
  if (!singular) {
    x = std::atan2(R(2, 1), R(2, 2));
    y = std::atan2(-R(2, 0), sy);
    z = std::atan2(R(1, 0), R(0, 0));
  } else {
    x = std::atan2(-R(1, 2), R(1, 1));
    y = std::atan2(-R(2, 0), sy);
    z = 0.0;
  }
  Eigen::Vector3d euler(x, y, z);
  if (degrees) euler *= (180.0 / kPi);
  return euler;
}

Eigen::Matrix3d euler_to_rotation_matrix(const Eigen::Vector3d& euler_zyx, bool degrees) {
  double x = euler_zyx(0), y = euler_zyx(1), z = euler_zyx(2);
  if (degrees) {
    x *= kPi / 180.0;
    y *= kPi / 180.0;
    z *= kPi / 180.0;
  }
  Eigen::AngleAxisd rx(x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(y, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(z, Eigen::Vector3d::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

double rotation_distance_deg(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
  Eigen::Matrix3d dR = R1.transpose() * R2;
  double trace = (dR.trace() - 1.0) / 2.0;
  trace = std::max(-1.0, std::min(1.0, trace));
  return std::acos(trace) * (180.0 / kPi);
}

Eigen::Vector4d quaternion_mean(const std::vector<Eigen::Vector4d>& quats) {
  if (quats.empty()) return Eigen::Vector4d(0, 0, 0, 1);
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  for (const auto& q : quats) M += q * q.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);
  int idx = 0;
  double max_val = es.eigenvalues()(0);
  for (int i = 1; i < 4; ++i) {
    if (es.eigenvalues()(i) > max_val) {
      max_val = es.eigenvalues()(i);
      idx = i;
    }
  }
  return es.eigenvectors().col(idx);
}

Eigen::Matrix4d T_from_Rt(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

void Rt_from_T(const Eigen::Matrix4d& T, Eigen::Matrix3d& R, Eigen::Vector3d& t) {
  R = T.block<3, 3>(0, 0);
  t = T.block<3, 1>(0, 3);
}

Eigen::Matrix3d robust_rotation_average(
  const std::vector<Eigen::Matrix3d>& rotations,
  int n_ransac,
  double threshold_deg) {
  if (rotations.empty()) return Eigen::Matrix3d::Identity();
  if (rotations.size() == 1) return rotations[0];

  std::vector<Eigen::Vector4d> quats;
  quats.reserve(rotations.size());
  for (const auto& R : rotations) quats.push_back(rotation_to_quat(R));

  std::vector<int> best_inliers;
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(0, static_cast<int>(quats.size()) - 1);

  for (int iter = 0; iter < n_ransac; ++iter) {
    int idx = dist(rng);
    const Eigen::Vector4d& ref = quats[idx];
    std::vector<int> inliers;
    for (size_t i = 0; i < quats.size(); ++i) {
      double dot = std::abs(ref.dot(quats[i]));
      dot = std::max(0.0, std::min(1.0, dot));
      double angle_deg = std::acos(dot) * 2.0 * (180.0 / kPi);
      if (angle_deg < threshold_deg) inliers.push_back(static_cast<int>(i));
    }
    if (inliers.size() > best_inliers.size()) best_inliers = inliers;
  }

  if (best_inliers.empty())
    for (size_t i = 0; i < quats.size(); ++i) best_inliers.push_back(static_cast<int>(i));

  std::vector<Eigen::Vector4d> inlier_quats;
  for (int i : best_inliers) inlier_quats.push_back(quats[i]);
  Eigen::Vector4d mean_q = quaternion_mean(inlier_quats);
  return quat_to_rotation(mean_q);
}

Eigen::Matrix4d interpolate_se3(
  const Eigen::Matrix4d& T1,
  const Eigen::Matrix4d& T2,
  double alpha) {
  Eigen::Matrix3d R1, R2;
  Eigen::Vector3d t1, t2;
  Rt_from_T(T1, R1, t1);
  Rt_from_T(T2, R2, t2);

  Eigen::Vector4d q1 = rotation_to_quat(R1);
  Eigen::Vector4d q2 = rotation_to_quat(R2);
  double dot = q1.dot(q2);
  if (dot < 0) { q2 = -q2; dot = -dot; }
  dot = std::max(-1.0, std::min(1.0, dot));
  double theta = std::acos(dot);

  Eigen::Vector4d q_interp;
  if (std::abs(theta) < 1e-8) {
    q_interp = q1;
  } else {
    q_interp = (std::sin((1 - alpha) * theta) * q1 + std::sin(alpha * theta) * q2) / std::sin(theta);
  }
  Eigen::Matrix3d R_interp = quat_to_rotation(q_interp);
  Eigen::Vector3d t_interp = (1 - alpha) * t1 + alpha * t2;
  return T_from_Rt(R_interp, t_interp);
}

}  // namespace unicalib
