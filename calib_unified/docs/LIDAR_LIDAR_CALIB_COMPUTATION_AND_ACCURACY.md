# LiDAR-LiDAR 外参标定：计算逻辑与精度分析

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **坐标系与语义** | T_target_in_ref 约定一致：NDT/GICP 的 source=target、target=ref，FPFH 对应 (src=target 点, tgt=ref 点)，SVD 求得的 T 满足 tgt≈T*src，语义正确。 |
| **已发现逻辑/精度问题** | ① FPFH 特征与点云索引在存在 NaN 时不对齐，导致错误对应；② RANSAC 未排除三点共线，易得退化解；③ 旋转矩阵未强制投影到 SO(3)，数值误差可能使 det(R)≠1；④ 加权四元数平均在旋转分散时不稳定。 |
| **精度保证** | 无 Ground Truth 时无法严格保证；可通过内点率、重叠率、多帧一致性做间接保证；建议增加退化检测与后验校验。 |

---

## 1. 坐标系与变换约定

### 1.1 外参语义

- **T_target_in_ref**：target 系在 ref 系下的位姿；点从 target 系变到 ref 系：**p_ref = T_target_in_ref * p_target**。
- 标定对 `[ref_id, target_id]` 表示求 **T_target_in_ref**（target 在 ref 下）。

### 1.2 各模块中的一致性

| 模块 | 输入/输出 | 约定检查 |
|------|-----------|----------|
| **NDT/GICP** | PCL: source=target 点云, target=ref 点云；align 得到 T: source→target | T 即 target→ref = T_target_in_ref ✓ |
| **FPFH+RANSAC** | src_pts=target 系点, tgt_pts=ref 系点；求 T 使 tgt_pts[i] ≈ T*src_pts[i] | T 即 T_target_in_ref ✓ |
| **pose_from_correspondences** | t = c_tgt - R*c_src，即 tgt = R*src + t | T 映射 src→tgt = target→ref ✓ |
| **evaluate_registration** | transform_cloud(scan_target, extrinsic)，再与 ref 最近邻比较 | 用 T 把 target 点变到 ref 系 ✓ |
| **calibrate_manual** | T_new = T_init * delta；若 delta 为「在 ref 系下的增量」则应为 T_new = delta * T_init | 当前为右乘 delta，即 delta 在 target 系；若 UI 按「在 ref 系调整」设计则需改为左乘。 |

**结论**：除手动校准的左右乘需与 UI 约定一致外，其余计算语义正确。

---

## 2. 已发现的问题与影响

### 2.1 FPFH 特征与点云索引不对齐（影响精度/正确性）

**位置**：`compute_fpfh_features` + `calibrate_fpfh_teaser`。

**原因**：`compute_fpfh_features` 内用 `cloud_with_normals` 只 push 了**非 NaN** 点，因此 `features.size() = cloud_with_normals.size()` ≤ `cloud.size()`，且 `features[i]` 对应的是「第 i 个非 NaN 点」，不是「原云中第 i 个点」。  
在 `calibrate_fpfh_teaser` 里用 `(*tgt)[i]` 与 `feat_tgt[i]` 配对，当 tgt 中存在 NaN 时，索引错位，**对应关系错误**，粗标定可能偏很大或失败。

**修复建议**：  
- 在调用 FPFH 前对 ref/tgt 做「去 NaN」过滤，得到与 `features` 严格 1:1 的点集再参与匹配与 RANSAC；或  
- 在 `compute_fpfh_features` 中返回「有效点的索引」或「有效点子云」，用该子集参与后续对应与位姿估计。

---

### 2.2 RANSAC 三点共线退化（影响鲁棒性）

**位置**：`solve_teaser` → `pose_from_correspondences(s3, t3, T)`。

**原因**：若采样的 3 对点近似共线，矩阵 H = Σ (tgt_i - c_tgt)(src_i - c_src)^T 秩 ≤ 2，SVD 得到的 R 不唯一或错误，易得到反射或错误旋转。

**修复建议**：  
- 采样后检查三点是否近似共线（例如两向量夹角接近 0 或 π，或面积/体积过小）；  
- 若共线则丢弃该样本，重新采样，不计入迭代次数上限。

---

### 2.3 旋转矩阵未投影到 SO(3)（影响数值精度）

**位置**：`pose_from_correspondences` 中 `R = V*U^T`（及 det<0 时翻正）。

**原因**：浮点误差可能导致 det(R) 略偏离 1 或 R 略非正交，长期使用或链式变换会放大误差。

**修复建议**：  
- 在得到 R 后做一次到 SO(3) 的投影：例如对 R 做 SVD，R = U*S*V^T，令 R_so3 = U * V^T（或再保证 det=1）；或使用 Eigen/Sophus 的「最近正交矩阵」工具。

---

### 2.4 加权四元数平均的稳定性（影响多帧融合精度）

**位置**：`calibrate_fine` 中 `fine_fusion_method == "weighted"` 的 q_sum 平均。

**原因**：对多个四元数做线性加权和再归一化，当旋转分散（如相差接近 180°）时，和向量接近 0，归一化结果不稳定；且未统一到半空间（q.w≥0）时符号歧义会削弱平均效果。

**现状**：代码已做 `if (q.w() < 0) q.coeffs() *= -1` 统一到半空间；但当 `q_sum.norm()` 很小时仍直接 `normalize()`，可能放大噪声。

**修复建议**：  
- 若 `q_sum.norm() < ε`，回退到取中值或取 fitness 最佳的一帧，避免使用不稳定的加权平均。

---

### 2.5 其他可能影响精度的点

- **体素下采样**：`voxel_size` 过大时几何细节丢失，配准精度上限降低；过小则计算量大且易过拟合噪声。  
- **多帧时间未对齐**：ref 与 target 若未按时间戳对齐，逐帧 GICP 的「同一时刻」假设不成立，多帧融合会引入额外误差（P3 B样条时间偏移正是为此预留）。  
- **可观测性**：`analyze_observability` 仍为固定返回值，未根据实际重叠率/几何判断是否可标定，无法从逻辑上保证「标定可行」。

---

## 3. 精度能否保证？

### 3.1 无法严格保证的情况

- 无 Ground Truth 时，无法给出「平移/旋转误差 < 某阈值」的数学保证。  
- 粗标定（FPFH+RANSAC）依赖特征匹配与内点比例，存在局部最优或错误收敛的可能。  
- NDT/GICP 为局部迭代，初值差时可能收敛到错误局部极小。

### 3.2 可做的间接保证与建议

| 手段 | 说明 |
|------|------|
| **内点率 / 重叠率** | 用 `evaluate_registration` 的 overlap_ratio、inlier 比例做后验；低于阈值则报警或建议重采。 |
| **多帧一致性** | 已有多帧 median/weighted 融合；可增加「逐帧外参离散度」检查（如平移/旋转方差过大则报警）。 |
| **退化检测** | RANSAC 中共线检查、内点过少时拒绝；加权平均时 q_sum 过小则回退中值。 |
| **SO(3) 投影** | 所有产生旋转的地方（含 RANSAC 精化、多帧融合）保证输出严格在 SO(3)。 |
| **validate_extrinsic** | 已有平移范数、有限性检查；可增加「旋转行列式/正交性」检查。 |

---

## 4. 建议修复优先级与已做修改

1. **P0（已实现）**：FPFH 前对 ref/tgt 调用 `remove_nan`，保证特征与点 1:1；RANSAC 中 `are_collinear` 检测，共线则跳过该样本。  
2. **P1（已实现）**：`pose_from_correspondences` 得到 R 后再做一次 SVD 投影到 SO(3)；加权融合时 `q_sum.norm() < 1e-6` 回退到 fitness 最佳单帧。  
3. **P2（待做）**：可观测性基于真实重叠率；手动校准左右乘与 UI 约定一致并文档化。

---

## 5. 异常与边界处理（完备性）

标定模块已做以下异常与边界防护，保证不因非法输入或中间结果导致崩溃或未定义行为：

| 类别 | 处理 |
|------|------|
| **配置** | voxel_size / ndt_resolution / gicp_max_corr_dist / fpfh_radius 等钳位到合法范围；max_frames/迭代次数取正且上限；无效时用安全默认值并继续。 |
| **输入** | 点云空、null、含 NaN/Inf 时提前返回或过滤；首帧/逐帧访问前检查 cloud 非空；init_guess/init_extrinsic 含 NaN/Inf 时用单位阵或拒绝并回退粗标定。 |
| **中间结果** | NDT/GICP 的 getFinalTransformation() 后检查 allFinite；pose_from_correspondences 的 R/t 检查有限；solve_teaser 返回前 is_se3_finite(best_T/refined)。 |
| **输出** | 所有产生 SE3 的路径后调用 validate_extrinsic（有限性、平移范数、旋转行列式）；未通过则丢弃或回退并打日志。 |
| **异常** | calibrate_two_stage 内 try/catch(std::exception&)/catch(...)，异常时写 result.failure_reason、needs_manual，若有 coarse 则回退到 result.fine=coarse，保证返回有效 result。 |
| **手动/B样条** | calibrate_manual 与 refine_with_bspline 校验输入 SE3，无效则使用单位阵或初始外参并打日志。 |

## 6. 参考文献与符号

- 外参：T_target_in_ref，p_ref = T * p_target。  
- PCL align：T 将 source 变换到 target 系。  
- 本文档与 `LIDAR_LIDAR_EXTRINSIC_ALGORITHM_AND_OPTIMIZATION.md` 配套使用。
