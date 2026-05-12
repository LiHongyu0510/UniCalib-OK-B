# LiDAR-Camera 标定「非常不准」原因分析：数据集 vs 计算逻辑

## 0. Executive Summary

| 结论 | 说明 |
|------|------|
| **与数据集关系很大** | SLAM 数据集多为「按索引/文件名」对齐、无标定板、无粗标定初值；时间戳常为回退值（非真实时间），帧对可能错位；场景与运动特性也不利于边缘对齐收敛。 |
| **计算逻辑本身无致命错误** | 外参约定（T_cam_lidar：lidar→camera）、投影公式、PnP 转 SE3、NCC/多特征/Ceres 流程与现有文档一致；在**正确初值 + 正确帧对**下可收敛。 |
| **根因集中在三处** | ① 无粗标定时常以 **identity 初值** 跑边缘对齐，NCC 盆地非凸，易陷局部最优或有效帧不足；② **时间戳来自文件名**，SLAM 数据多为索引回退（0, 0.1, 0.2…），实际未做真实时间同步；③ **帧对匹配策略** 在回退时间戳下等价于「按索引对齐」，若 LiDAR/相机帧率或起始不同则帧对错误。 |

**建议**：对 SLAM 数据优先提供粗标定（`--coarse` 或手给初值）；或保证数据为「真实时间戳 + 同步良好」；并在加载阶段确认时间戳来源（见下文「时间戳回退」日志）。

---

## 1. 数据流与标定流程（逐段说明）

### 1.1 入口与数据加载

- **入口**：`calib_unified_run.sh --run --task lidar-cam` → `CalibPipeline::run_fine_lidar_camera()`。
- **数据来源**：  
  - 文件模式：`cfg_.lidar_data_dir` 下 PCD、`camera_images_dir` 下图像。  
  - 加载逻辑（`calib_pipeline.cpp` 约 966–998、1072–1112）：
    - PCD：目录遍历 → 按路径排序 → 步长采样（最多 100 帧）→ 每帧 `scan.timestamp = std::stod(fname)`，**若 `stod` 抛异常则回退为 `i * 0.1`**（`fname` 为文件名 stem，如 `frame_001` 会回退）。
    - 图像：同理，`ts = std::stod(fname)`，异常则 `ts = i * 0.1`。

**与 SLAM 数据集的关系**：  
- 很多 SLAM 数据导出后文件名为 `frame_000001.pcd`、`image_000001.png` 等，**无法被 `stod` 解析**，因此时间戳实际为 `0, 0.1, 0.2, ...`，即**按索引生成的伪时间**。  
- 此时 LiDAR 与相机的时间序列完全由「排序后的文件顺序」决定，**没有真实时间对齐**；帧对匹配等价于「第 i 帧 LiDAR 配第 i 帧图像」，若录制时两传感器帧率或起始不同，会 systematic 错配。

### 1.2 粗标定初值

- **来源**：`coarse_lidar_cam_init_`，由 `run_coarse_stage(LIDAR_CAM_EXTRIN)` 调用 MIAS-LCEC 等得到；若未启用 `--coarse` 或粗标定失败，则为 `std::nullopt`。
- **使用**：`calibrate_two_stage(..., coarse_init, ...)` 中  
  `init_T = coarse_init.value_or(Sophus::SE3d())`  
  即**无粗标定时初值为 identity**。

**对精度的影响**：  
- 边缘对齐（NCC）是**非凸**的；从 identity 起步时，若真实外参偏离较大，NCC 对位姿的梯度很平或指向错误方向，容易：  
  - 有效帧数不足（NCC 均低于阈值，`frame_pairs.size() < 3`），直接返回初值（identity）；或  
  - 收敛到错误局部最优，标定「非常不准」。

### 1.3 精标定：边缘对齐（prefer_targetfree = true 时）

- **入口**：`calibrate_edge_align(lidar_scans, camera_frames, cam_intrin, init_T, ...)`。
- **时间偏移**（若 `optimize_time_offset`）：在 `time_offset_s ± search_range_s` 内离散搜索，对每个偏移用当前 `T_cam_lidar` 算 NCC，取 NCC 最大的偏移；后续帧对用 `ts_cam + time_offset_s` 找最近 LiDAR。
- **帧对采集**：  
  - 对每个候选图像时间 `ts_cam`，用 `ts_cam + time_offset_s` 找最近 LiDAR 帧；  
  - 若 `dt > frame_sync_threshold_s`（默认 0.1 s）丢弃；  
  - 用当前 `T_cam_lidar` 算该帧对 NCC，高于阈值才加入 `frame_pairs`。  
- 当时间戳均为回退值（0, 0.1, 0.2…）时，`time_offset_s` 搜索仍可能改善「相机与 LiDAR 索引之间的固定偏移」，但**无法纠正帧率差异或非固定延迟**。

### 1.4 单帧 NCC 与投影（计算逻辑核对）

- **NCC 用到的投影**：  
  `lidar_to_intensity_image_static(scan, T_cam_in_lidar, cam_intrin)` 中：  
  `p_c = T_cam_in_lidar * p_l`，  
  `u = fx * p_c.x()/p_c.z() + cx`，  
  `v = fy * p_c.y()/p_c.z() + cy`。  
- 与 pinhole 约定一致（相机系 x 右 y 下 z 前），**无符号错误**。  
- **T 的语义**：全工程用 `T_cam_lidar` 表示「lidar → camera」变换（`p_cam = T_cam_lidar * p_lidar`），与 `ExtrinsicSE3` 的 `SE3_TargetInRef()` 在本工程中的使用一致（见 CODE_ANALYSIS_CALCULATION_AND_ISSUES.md）。

### 1.5 Ceres 优化与结果

- 参数：6 维（轴角 3 + 平移 3），初值由 `init_T`（粗标定或 identity）转成轴角+平移。  
- 代价：`EdgeNCCCost`（最小化 -NCC）、可选 `CornerReprojCost`、`IntensityConsistencyCost`；数值微分 `NumericDiffCostFunction<..., CENTRAL, 1, 6>`。  
- 结果：从 `params` 恢复 `T_cam_lidar`，写回 `result.set_SE3(T_cam_lidar)`，并用于 RMS/converged 判断与可视化。  
- **逻辑正确**；在初值合理、帧对正确的前提下，可收敛到合理外参。

### 1.6 棋盘格法（prefer_targetfree = false）

- 使用 OpenCV `solvePnPRansac` + `solvePnPRefineLM`：3D 点为 LiDAR 系下角点，2D 为图像角点；得到 R,t 满足 `p_cam = R * p_lidar + t`，即 T_cam_lidar；代码中 `SO3_TargetInRef = R_cam_lidar`、`POS_TargetInRef = tv`，与投影处使用一致，**无 PnP 转换错误**。  
- SLAM 数据通常**没有棋盘格**，故此路多不适用。

---

## 2. 逐项：是数据集问题还是计算问题？

| 项目 | 更偏向 | 说明 |
|------|--------|------|
| 初值为 identity 导致不准 | 数据/流程 | 未提供粗标定或初值，算法只能从 identity 起步；边缘对齐对初值敏感，易失败或局部最优。 |
| 时间戳为索引回退 (0,0.1,…) | 数据集 | 文件名非数值时必然回退；SLAM 导出常用 frame_xxx，导致无真实时间、仅按索引对齐。 |
| 帧对错位（不同步） | 数据集 + 配置 | 回退时间戳下等价按索引配对；若两传感器帧率/起始不一致，则 systematic 错配。 |
| 有效帧 < 3 或 NCC 普遍低 | 二者 | 初值差或帧对错都会导致 NCC 低；数据集缺少清晰边缘或运动模糊也会。 |
| 投影公式 / T 方向 / PnP | 计算 | 已核对，与文档一致，未发现错误。 |
| 多尺度 NCC、鲁棒核、Ceres 配置 | 计算 | 实现正确；在初值/数据合适时能发挥作用。 |

---

## 3. 建议措施（可落地）

1. **对 SLAM 数据尽量提供粗标定**  
   - 使用 `--coarse` 跑 MIAS-LCEC，或手测/别工具得到初值，通过配置或 API 传入，避免从 identity 起步。

2. **确认时间戳含义**  
   - 若数据为「真实时间戳」（如 bag 转存时保留 topic 时间），应保证文件名 stem 为数值（如 `1634567890.123.pcd`），以便 `stod` 成功；否则当前实现会回退到索引时间，需在加载后打日志（见下节）以便排查。

3. **时间戳回退时打日志**  
   - 在 PCD/图像加载处，当 `stod` 异常使用回退值时打一条 INFO/WARN，便于确认 SLAM 数据是否处于「按索引对齐」模式。

4. **帧同步与阈值**  
   - 若使用回退时间戳，可适当增大 `frame_sync_threshold_s` 以容忍索引对齐的「虚时间」偏差；但根本仍是尽量用真实时间戳或保证 LiDAR/相机按相同索引严格对应。

5. **收敛与质量**  
   - 关注日志中的「初值 NCC」「有效帧数」「converged」；若初值 NCC 很低或有效帧 < 3，优先从初值与帧对（时间戳/同步）排查，再考虑场景是否适合边缘对齐。

---

## 4. 变更记录

- 新增本文档：区分「数据集特性」与「计算逻辑」，并给出逐段说明与建议。  
- 建议在 pipeline 加载 PCD/图像时，对时间戳回退打日志（见下一节实现）。
