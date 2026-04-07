"""
几何变换工具函数
"""
from __future__ import annotations
from typing import List

import numpy as np
from scipy.spatial.transform import Rotation


def rotation_matrix_to_euler(R: np.ndarray, degrees: bool = True) -> np.ndarray:
    """3×3 旋转矩阵 → ZYX 欧拉角。"""
    return Rotation.from_matrix(R).as_euler("zyx", degrees=degrees)


def euler_to_rotation_matrix(euler_zyx: np.ndarray, degrees: bool = True) -> np.ndarray:
    """ZYX 欧拉角 → 3×3 旋转矩阵。"""
    return Rotation.from_euler("zyx", euler_zyx, degrees=degrees).as_matrix()


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """3-向量 → 3×3 反对称矩阵。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def rotation_distance_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """计算两个旋转矩阵之间的角度距离（°）。"""
    dR = R1.T @ R2
    trace = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(trace)))


def quaternion_mean(quats: np.ndarray) -> np.ndarray:
    """
    计算四元数集合的平均值（特征向量法）。
    quats: (N, 4) 格式 [x, y, z, w]
    """
    Q = np.array(quats)
    M = Q.T @ Q
    eigvals, eigvecs = np.linalg.eigh(M)
    return eigvecs[:, np.argmax(eigvals)]


def T_from_Rt(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """拼合 4×4 变换矩阵 T = [R | t; 0 | 1]。"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


def Rt_from_T(T: np.ndarray):
    """从 4×4 变换矩阵分解 R, t。"""
    return T[:3, :3], T[:3, 3]


def robust_rotation_average(rotations: List[np.ndarray],
                             n_ransac: int = 100,
                             threshold_deg: float = 5.0) -> np.ndarray:
    """
    RANSAC + 四元数平均 鲁棒旋转集成。
    """
    if not rotations:
        return np.eye(3)
    if len(rotations) == 1:
        return rotations[0]

    quats = np.array([
        Rotation.from_matrix(R).as_quat() for R in rotations
    ])

    best_inliers: List[int] = []
    rng = np.random.default_rng(42)

    for _ in range(n_ransac):
        idx = rng.integers(0, len(quats))
        ref = quats[idx]
        angles = np.array([
            float(np.degrees(2.0 * np.arccos(
                np.clip(abs(float(np.dot(ref, q))), 0.0, 1.0))))
            for q in quats
        ])
        inliers = [i for i, a in enumerate(angles) if a < threshold_deg]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers

    if not best_inliers:
        best_inliers = list(range(len(quats)))

    mean_q = quaternion_mean(quats[best_inliers])
    return Rotation.from_quat(mean_q).as_matrix()


def interpolate_se3(T1: np.ndarray, T2: np.ndarray, alpha: float) -> np.ndarray:
    """在两个 SE3 变换之间线性插值（slerp 旋转 + 线性平移）。"""
    R1, t1 = Rt_from_T(T1)
    R2, t2 = Rt_from_T(T2)

    rot1 = Rotation.from_matrix(R1)
    rot2 = Rotation.from_matrix(R2)
    # SLERP
    q1, q2 = rot1.as_quat(), rot2.as_quat()
    dot = np.dot(q1, q2)
    if dot < 0:
        q2 = -q2
        dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    theta = np.arccos(dot)
    if abs(theta) < 1e-8:
        q_interp = q1
    else:
        q_interp = (np.sin((1 - alpha) * theta) * q1 +
                    np.sin(alpha * theta) * q2) / np.sin(theta)

    R_interp = Rotation.from_quat(q_interp).as_matrix()
    t_interp = (1 - alpha) * t1 + alpha * t2

    return T_from_Rt(R_interp, t_interp)
