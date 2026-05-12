/**
 * UniCalib C++ — 几何变换工具
 * 对应 Python: unicalib/utils/transforms.py
 */
#pragma once

#include <Eigen/Dense>
#include <vector>

namespace unicalib {

/** 3×3 旋转矩阵 → ZYX 欧拉角 [度] */
Eigen::Vector3d rotation_matrix_to_euler(const Eigen::Matrix3d& R, bool degrees = true);

/** ZYX 欧拉角 → 3×3 旋转矩阵 (degrees=true 表示输入为度) */
Eigen::Matrix3d euler_to_rotation_matrix(const Eigen::Vector3d& euler_zyx, bool degrees = true);

/** 3-向量 → 3×3 反对称矩阵 */
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v);

/** 两旋转矩阵之间的角度距离 [度] */
double rotation_distance_deg(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2);

/** 四元数集合的平均 (特征向量法), quats: N×4 [x,y,z,w] */
Eigen::Vector4d quaternion_mean(const std::vector<Eigen::Vector4d>& quats);

/** 4×4 变换矩阵 T = [R|t; 0|1] */
Eigen::Matrix4d T_from_Rt(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

/** 从 4×4 分解 R, t */
void Rt_from_T(const Eigen::Matrix4d& T, Eigen::Matrix3d& R, Eigen::Vector3d& t);

/** RANSAC + 四元数平均 鲁棒旋转集成 */
Eigen::Matrix3d robust_rotation_average(
  const std::vector<Eigen::Matrix3d>& rotations,
  int n_ransac = 100,
  double threshold_deg = 5.0);

/** 两 SE3 之间线性插值 (slerp 旋转 + 线性平移) */
Eigen::Matrix4d interpolate_se3(
  const Eigen::Matrix4d& T1,
  const Eigen::Matrix4d& T2,
  double alpha);

/** 旋转矩阵 → 旋转向量 (Rodrigues) */
Eigen::Vector3d rotationMatrixToRotvec(const Eigen::Matrix3d& R);

/** 旋转向量 → 旋转矩阵 (Rodrigues) */
Eigen::Matrix3d rotvecToRotationMatrix(const Eigen::Vector3d& rv);

}  // namespace unicalib
