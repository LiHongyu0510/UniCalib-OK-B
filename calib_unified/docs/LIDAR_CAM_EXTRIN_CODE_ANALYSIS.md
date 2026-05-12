# LiDAR-Camera 外参标定 — 逐行代码分析与精度保障

本文档对 `lidar_camera_calib.cpp` / `lidar_camera_calib.h` 做**计算正确性**与**逻辑正确性**的逐行分析，确保标定精度与可维护性。

---

## 0. Executive Summary

| 维度 | 结论 |
|------|------|
| **计算错误** | 已发现并修复 2 处：① `compute_reproj_rms` 分母错误（用总点数而非有效投影点数）；② 目标法代价中“点在相机后方”时残差置 0 导致优化无梯度。 |
| **逻辑错误** | 未发现明显逻辑反义或分支错误；外参语义（T：lidar→camera）在投影、BA、可视化中一致。 |
| **精度相关** | 目标法 BA 未使用畸变模型（仅针孔），与 PnP 阶段使用畸变不一致，可能在高畸变镜头下影响精度；圆点格网格划分与 OpenCV 行列约定一致。 |
| **建议** | 保持当前修复；可选：BA 中支持畸变、轴角 >2π 时等价到 [−π,π]、文档明确外参命名。 |

---

## 1. 外参约定与投影公式（必须一致）

### 1.1 本工程约定

- **变量名**：`T_cam_lidar` / `T_cam_in_lidar` 表示 **LiDAR 系 → 相机系** 的变换。
- **投影**：`p_cam = T_cam_lidar * p_lidar`，即 `p_cam = R * p_lidar + t`。
- **ExtrinsicSE3**：`ref_sensor_id = lidar`，`target_sensor_id = cam` 时，`SE3_TargetInRef()` 在本工程内被**用作** lidar→camera 的 4×4 矩阵（即 `p_cam = T * p_lidar`）。  
  （注意：`calib_param.h` 注释写的是 “旋转: target -> ref”，与“存储 lidar→cam”在字面上相反；实际代码中存储与使用的均为 lidar→camera，以代码为准。）

### 1.2 各模块一致性检查

| 位置 | 用法 | 是否正确 |
|------|------|----------|
| `lidar_to_intensity_image` L39-46 | `p_c = T_cam_in_lidar * p_l`，再 u,v 针孔 | ✓ |
| `TargetReprojCost` L414-419 | `p_cam = R*p_lidar + t`，u,v 针孔 | ✓ |
| `compute_reproj_rms` L424-428 | `p_c = T_cam_lidar * p_l`，u,v 针孔 | ✓ |
| `coarse_rotation_search` L447-450 | `p_c = T * p_l`，T=SE3(R,t) | ✓ |
| `calibrate_target` 结果 L925-926 | `SO3_TargetInRef=R_f`, `POS_TargetInRef=t_final`，T_final 即 lidar→cam | ✓ |
| `visualize_projection` L1765-1772 | `T = extrin.SE3_TargetInRef()`，`p_c = T * p_l` | ✓ |
| `evaluate_edge_alignment` L1815 | 同上，传入 `lidar_to_intensity_image` | ✓ |

结论：**投影与存储的外参语义一致，无计算方向错误。**

---

## 2. 逐段代码分析

### 2.1 强度图投影 `lidar_to_intensity_image` (L31-55)

```cpp
// T_cam_in_lidar: 将 LiDAR 点变换到 cam 坐标系
Eigen::Vector3d p_c = T_cam_in_lidar * p_l;
if (p_c.z() < 0.1) continue;
double u = cam_intrin.fx * p_c.x() / p_c.z() + cam_intrin.cx;
double v = cam_intrin.fy * p_c.y() / p_c.z() + cam_intrin.cy;
```

- **计算**：针孔模型正确；z < 0.1 过滤相机后方点，合理。
- **强度**：同一像素取 `max` 强度，避免多次覆盖导致偏暗，合理。
- **无畸变**：与后续边缘/NCC 一致；若需与带畸变图像严格对齐，需先对图像去畸变或对投影加畸变（当前为设计取舍）。

### 2.2 LiDAR 棋盘格角点 `detect_board_in_lidar` (L65-399)

- **平面方程** L156-163：`normal` 单位法向，`d_plane = -normal.dot(p1)`，点到面距离 `|n·p + d|`，正确。
- **局部坐标系** L204-209：`v_axis = normal × u_axis`，再 `u_axis = v_axis × normal`，形成右手系，正确。
- **网格生成** L265-276：`cell_w = board_width_estimate/(cols-1)`，`cell_h = board_height_estimate/(rows-1)`，角点按 (r,c) 均匀分布，正确。
- **梯度精化** L311-329：在 SEARCH_RADIUS 内取**强度梯度最大**的点作为角点。注意：`candidates.emplace_back(grad_score, points_3d_original[i])` 用的是邻域内点的 3D 位置，不是射线-角点交点，在边缘明显时可用，角点几何精度弱于理想“交点”方法，属实现取舍，非错误。
- **平面精化** L358-364：将角点投影回精化平面，正确。
- **网格正则化** L374-391：用理想网格与检测位置加权（0.7 理想 + 0.3 检测），减少噪声，逻辑正确。

### 2.3 LiDAR 圆点格 `detect_circles_in_lidar` (L405-527)

- **平面 RANSAC**：与棋盘格同构，正确。
- **2D 划分** L377-378：`cell_w = (u_max - u_min) / cols`，`cell_h = (v_max - v_min) / rows`，按 (r,c) 格子取质心或格子中心，与 OpenCV `findCirclesGrid` 的对称/非对称行列数一致，顺序为行优先 (r,c)，正确。
- **空格子** L394-396：`count==0` 时用格子中心 3D 点，合理。

### 2.4 目标法重投影代价 `TargetReprojCost` (L532-421)

- **参数**：x[0:2] 轴角，x[3:5] 平移；`R` 从轴角构造，`p_cam = R*p_lidar + t`，与约定一致。
- **z < 1e-4**：原逻辑 `residual[0]=residual[1]=0` 会导致该点对代价无贡献、梯度为 0，可能让解把点推到相机后方。**已修复**：改为固定大残差（如 100），使优化器获得梯度，避免错误收敛。
- **u,v 公式**：`u = fx*x/z + cx`，`v = fy*y/z + cy`，正确。

### 2.5 `compute_reproj_rms` (L424-435)

- **原 bug**：`sum2` 只累加“z ≥ 1e-4”的点的误差平方，但最后 `return sqrt(sum2 / pts3d.size())` 用总点数做分母，当有部分点在相机后方时 RMS 偏小且不符合“仅对可见点算 RMS”的语义。
- **修复**：用 `count` 记录实际参与累加的点数，`return count==0 ? 1e9 : sqrt(sum2/count)`。

### 2.6 粗旋转搜索 `coarse_rotation_search` (L438-478)

- **欧拉角**：R = Rz(yaw)*Ry(pitch)*Rx(roll)，与常见 ZYX 欧拉一致。
- **平移**：使用 `init_extrin` 的平移（若有），只搜旋转，正确。
- **内点计数**：重投影误差 ≤ inlier_threshold_px 即内点，逻辑正确。

### 2.7 仅平移精化 `refine_translation_only` (L461-497)

- 固定 R，只优化 t；`TargetReprojCostTwoBlocks` 中“点在后方”已同样改为大残差，与 `TargetReprojCost` 一致。

### 2.8 目标法主流程 `calibrate_target` (L503-931)

- **帧同步** L461-466：按图像时间戳找最近 LiDAR，`best_dt > frame_sync_threshold_s` 则丢弃，合理。
- **OpenCV PnP** L538-551：`solvePnPRansac(pts3d_all, pts2d_all, K, dist, ...)`，3D 为 LiDAR 系，2D 为图像系；OpenCV 返回的 rvec/tvec 满足 `p_cam = R*p_obj + t`，即物体（LiDAR）到相机，与 T_cam_lidar 一致；`T_ba_init = Sophus::SE3d(R, tv)` 正确。
- **畸变**：PnP 使用 `dist`，但后续 BA 的 `TargetReprojCost` 与 `compute_reproj_rms` 均为无畸变针孔。若畸变较大，会存在初值（PnP）与精化（BA）模型不一致，可能限制精度；非逻辑错误，属模型简化。
- **离群帧剔除** L553-572：按当前 T_ba_init 算每帧 RMS，剔除超过 `target_per_frame_rms_threshold_px` 的帧，再 BA，逻辑正确。
- **结果写入** L923-928：`SO3_TargetInRef = R_f`，`POS_TargetInRef = T_final.translation()`，与 lidar→camera 一致。

### 2.9 边缘对齐 NCC `compute_frame_ncc` (L597-658)

- **NCC 公式** L624-631：`denom = s1*s2*e1.total()`，`ncc = (e1-m1).dot(e2-m2)/denom`。Pearson 相关系数为 `sum((a-ma)(b-mb))/(N*sigma_a*sigma_b)`，这里 `s1,s2` 为 std，故 `s1*s2*N` 与分子对应，正确。
- **多尺度** L634-652：多尺度下采样后分别算 NCC 再加权平均，权重和归一化，正确；零方差与空图防护已有。

### 2.10 边缘/角点/强度 Ceres 代价 (L662-758)

- **EdgeNCCCost**：残差 = -NCC，最小化即最大化 NCC，正确；轴角 >2π 时返回 0 残差，梯度为 0，可能卡住，见下文建议。
- **CornerReprojCost**：在点云投影中找距 keypoint 最近的点，残差 = (best_u - keypoint_u, best_v - keypoint_v)，几何上为“最近点”近似，非严格射线-点云交点，设计如此。
- **IntensityConsistencyCost**：投影点处 LiDAR 强度与图像灰度差均值，归一化到 [0,1]，逻辑正确。

### 2.11 可视化与评估 (L1760-1845)

- **visualize_projection**：`T = extrin.SE3_TargetInRef()`，`p_c = T * p_l`，与约定一致；注释中“假设 extrin 是 lidar_in_cam”易误解，实际使用的是“target in ref”的矩阵作为 lidar→camera。
- **evaluate_edge_alignment**：同样用 `SE3_TargetInRef()` 生成强度图并算 NCC，一致。

---

## 3. 已修复问题汇总

1. **compute_reproj_rms**  
   - 问题：分母用 `pts3d.size()`，分子只统计 z≥1e-4 的点，导致 RMS 偏小且语义错误。  
   - 修复：用有效投影点数 `count` 做分母，count==0 时返回 1e9。

2. **TargetReprojCost / TargetReprojCostTwoBlocks（点在相机后方）**  
   - 问题：`p_cam.z() < 1e-4` 时 residual 置 0，优化器无梯度，可能收敛到错误位姿。  
   - 修复：该情况下置较大固定残差（如 100），使代价有梯度，驱动解远离“点在后端”的配置。

---

## 4. 潜在问题与建议（不影响正确性，可提升精度/鲁棒性）

| 项 | 说明 | 建议 |
|----|------|------|
| 畸变一致性 | PnP 用 dist，BA 用针孔 | 若镜头畸变大，可在 BA 中加畸变项或先对图像去畸变再标定。 |
| 轴角 >2π | EdgeNCCCost 等返回 0 残差，梯度为 0 | 将角等价到 [−π,π] 再构造 R，或对 Ceres 参数加边界。 |
| 角点“最近点” | 非严格射线-点云交点 | 文档注明；若需严格几何可考虑射线-表面相交。 |
| 外参命名 | TargetInRef 与“lidar→cam 矩阵”在注释上易混 | 在 calib_param.h 或本模块头文件中明确“LiDAR-Cam 存的是 lidar→camera 变换”。 |

---

## 5. 验证清单（保证标定精度）

1. **单帧投影**：用标定结果将单帧点云投影到图像，与 `visualize_projection` 输出对比，边缘/标定板应对齐。
2. **RMS 一致性**：`compute_reproj_rms` 修复后，与每帧角点重投影误差手工计算对比，应一致。
3. **多帧 BA**：有离群帧时，剔除前后 RMS 与保留帧数应在日志中合理。
4. **时间偏移**：固定一帧对，扫描小范围 time_offset，NCC(offset) 应有合理峰。
5. **初值敏感性**：同一数据用 identity 与粗标定初值各跑一次，边缘法应收敛到相近 NCC（粗标定初值更稳）。

---

## 6. 变更记录

- 修复 `compute_reproj_rms`：分母改为有效投影点数，count==0 返回 1e9。
- 修复 `TargetReprojCost` / `TargetReprojCostTwoBlocks`：点在相机后方时使用固定大残差而非 0，保证优化器有梯度。
- **轴角 >2π**：新增 `axis_angle_to_rotation(angle, axis)`，将角度等价到 [-π,π]；在 `EdgeNCCCost`、`CornerReprojCost`、`IntensityConsistencyCost` 中统一使用，避免大角度时返回 0 残差导致梯度为 0。
- **目标法畸变一致**：当 `dist_coeffs` 不少于 4 个时，对 2D 观测调用 `cv::undistortPoints` 得到去畸变像素坐标，粗搜索、平移精化、每帧 RMS、BA 均使用该去畸变点与针孔投影比较；PnP 仍用原始畸变模型求初值，保证 PnP 初值与 BA 模型一致。
- 新增本文档：逐行计算与逻辑分析、外参约定核对、修复说明与验证清单。
