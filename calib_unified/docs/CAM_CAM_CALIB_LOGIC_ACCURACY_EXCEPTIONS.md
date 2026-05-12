# 相机-相机外参标定：计算逻辑、精度与异常处理分析

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **计算逻辑** | 坐标系约定与三角化/BA 公式正确；存在**退化初值**（零基线）未显式防护、**BA 结果未写 residual_rms**、**收敛判定未用 Ceres termination_type**。 |
| **标定精度** | 标定板路径有 RMS 输出且可保证；无目标路径依赖特征与初值，**BA 未输出 RMS 导致无法量化精度**，且零基线初值会导致三角化退化。 |
| **异常处理** | 部分分支有检查（E 空、内点不足、内参校验）；**空畸变系数**可能致 OpenCV 未定义行为、**BA 未运行/失败时 residual_rms 与收敛状态不准确**、缺少对退化初值与数值异常的防护。 |

下文逐项说明并给出已做/建议的修复。

---

## 1. 计算逻辑分析

### 1.1 坐标系与存储约定

- **ExtrinsicSE3**：`T_TargetInRef` = target 在 ref 系下，即 `p_ref = R * p_target + t`，对应 `SO3_TargetInRef`、`POS_TargetInRef`。
- **OpenCV stereoCalibrate**：返回的 R、T 满足将 cam1 下的点变换到 cam0：`p_cam0 = R * p_cam1 + T`，即 **T_cam0_cam1**。代码中 `result.SO3_TargetInRef = R`、`result.POS_TargetInRef = T`，与上述约定一致。✅
- **OpenCV recoverPose**：返回的 R、t 为第二相机相对第一相机位姿（第一相机为单位），即 cam1 在 cam0 下。与存储一致。✅

### 1.2 三角化与 BA 参数化

- **triangulatePoints**：`P0 = K0*[I|0]`（cam0 为参考），`P1 = K1*[R|t]`，其中 R,t = T_cam0_cam1。OpenCV 输出 3D 点位于**第一相机（cam0）坐标系**。✅
- **ReprojectCost**：3D 点 `(px,py,pz)` 在 cam0 系；pose 为 T_cam0_cam1（angle-axis + 平移）；`pt_c = R*pt + t` 为 cam1 系下点；观测为 cam1 图像 `(obs_u, obs_v)`。残差 = 投影到 cam1 的像素 - 观测。✅
- **多视图 MultiViewReprojectCost**：X 在 cam0 系，`T_cam0_cami` 为 cam_i 在 cam0 下，`p_cami = R^T*(X - t)` = T^{-1}*X，与注释一致。✅

### 1.3 已发现逻辑/鲁棒性问题

| 问题 | 位置 | 说明 | 修复建议 |
|------|------|------|----------|
| **零基线初值** | `bundle_adjustment_two_views` | 当 `init_T.translation().norm()` 接近 0 时，P1 = K1*[I|0]，与 P0 同，三角化退化（无基线），所有点无效或不可靠。 | 入口处若 `||t|| < ε`，打日志并直接返回 init_T，标记 `is_converged=false`，不执行三角化/BA。 |
| **BA 不写 residual_rms** | `bundle_adjustment_two_views` | 优化完成后未设置 `result.residual_rms`，一直为 0，无法反映重投影精度。 | 用 `summary.final_cost` 与有效观测数估算 RMS（见下），并写入 `result.residual_rms`。 |
| **收敛判定粗糙** | `bundle_adjustment_two_views` | 仅用 `n_valid >= 10` 判定 `is_converged`，未用 `summary.termination_type`。 | 同时要求 `termination_type == CONVERGENCE`（或至少非 FAILURE）。 |
| **角轴零向量** | `bundle_adjustment_two_views` 末尾 | `rv2.norm() < 1e-10` 时置单位旋转，逻辑正确；但若优化发散可能得到异常 pose，未做正交化/钳位。 | 可选：对 R 做 SVD 投影到 SO(3)；当前先保留，仅加强日志。 |

---

## 2. 标定精度能否保证

### 2.1 标定板路径 (stereoCalibrate)

- OpenCV `stereoCalibrate` 返回 **RMS**，代码已写入 `result.residual_rms`。
- 收敛判定：`rms < cfg_.max_rms_px`。
- **结论**：有明确数值指标，可通过 `max_rms_px` 控制精度要求。✅

### 2.2 无目标路径 (本质矩阵 + BA)

- **本质矩阵**：仅给出相对位姿（平移 up-to-scale），`calibrate_essential` 中 `result.residual_rms = 0`，无几何残差输出。
- **两视图 BA**：
  - 未设置 `result.residual_rms` → 无法从结果中读出重投影精度。
  - 若初值为单位阵（零基线），三角化无效，BA 实际未优化或仅用极少点，**精度无法保证**。
- **结论**：通过**补写 BA 的 residual_rms**、**避免零基线初值进入 BA**、以及（可选）尺度约束，可提升可观测性与精度可解释性。

### 2.3 尺度

- 本质矩阵 + 三角化/BA 的平移具有**尺度不确定性**；代码中已有 `enable_scale_constraint` + `known_baseline_scale` 可选约束。
- 标定板路径为度量标定，尺度由棋盘格尺寸确定。✅

---

## 3. 异常处理完整性

### 3.1 已有防护

- **recover_pose_from_E**：E 空、内点数 &lt; 8、内点比过低告警、R 行列式/正交性告警。
- **calibrate_stereo**：内参 fx/fy/width/height 校验、图像对数、有效帧数 ≥ 5。
- **calibrate_essential**：帧数 ≥ 3、总匹配数 ≥ 20、recover_pose 失败返回 nullopt。
- **bundle_adjustment_two_views**：匹配数 &lt; 10 直接返回；三角化后深度/投影范围/边界过滤；Ceres 求解 try/catch。
- **两阶段**：N &lt; 3 返回；无精标定结果时 `fine` 为空并打 WARN。

### 3.2 缺口与修复

| 缺口 | 风险 | 修复 |
|------|------|------|
| **空 dist_coeffs** | `cv::Mat(intrin.dist_coeffs).reshape(1,1)` 若 `dist_coeffs` 为空，得到空 Mat；部分 OpenCV 版本 `undistortPoints` 可能异常。 | 若 `dist_coeffs.empty()`，传入 1×5 零矩阵（或与内参模型一致的长度）作为 D。 |
| **BA 未运行** | n_valid &lt; 10 时不调用 Solve，但 `result.residual_rms` 仍为 0、`is_converged` 已置 false。 | 明确将“未运行”时的 `residual_rms` 设为 -1 或 NaN 表示无效；保持 `is_converged=false`。 |
| **零基线初值** | 三角化退化，可能大量无效点或数值不稳定。 | 入口检查 `init_T.translation().norm() < 1e-6`，直接返回并打 WARN。 |
| **recoverPose 返回的 t** | 若 t 为零（纯旋转或退化），尺度无定义。 | 可选：检查 `t.norm() < 1e-9` 时打 WARN 并仍返回（调用方可选择不加尺度约束）。 |
| **stereoCalibrate 失败** | OpenCV 可能不抛异常但 R/T 含 NaN/Inf。 | 可选：检查 R、T 的数值有效性（如 `cv::checkRange`）。 |
| **特征/读图异常** | `detectAndCompute`、`imread` 可能抛异常。 | 上层或调用方 try/catch；核心标定函数可文档说明“调用方应保证输入有效或捕获异常”。 |

---

## 4. 已实施/建议的代码修改摘要

1. **bundle_adjustment_two_views（已实施）**
   - 若 `init_T.translation().norm() < 1e-6`：打 WARN，设 `result.residual_rms = -1`、`is_converged = false`，返回 init_T。
   - 在 Ceres Solve 成功后：用 `summary.final_cost` 与 `n_valid` 估算重投影 RMS（`sqrt(2*cost/(2*n_valid))`），写入 `result.residual_rms`；`is_converged` 使用 `summary.termination_type == ceres::CONVERGENCE`。
   - 当 n_valid &lt; 10 未求解时：设 `result.residual_rms = -1`；Ceres 异常时同样设 -1 并返回 init_T。

2. **空畸变系数（已实施）**
   - 新增 `safe_dist_coeffs(dist_coeffs)`：空时返回 1×5 零 Mat。在 `recover_pose_from_E`、两处 `calibrate_stereo`、`bundle_adjustment_two_views`、`visualize_stereo_rectification` 中均改用该函数。

3. **文档与配置**
   - 在配置/README 中说明：无目标法建议提供非零初值或尺度约束；BA 的 residual_rms 为近似值（Huber 下非精确像素 RMS）。

---

## 5. 验证建议

- 单测：初值为单位阵时，两阶段应得到 coarse 或 BA 失败/未运行，且 `residual_rms <= 0` 或 `is_converged == false`。
- 回归：标定板模式 RMS 与修改前一致；无目标模式在给好初值下，BA 后 `residual_rms` 为合理正数且收敛。
- 边界：空图像、空内参、空 dist_coeffs、极少匹配点，均应有明确返回或日志，不崩溃。
