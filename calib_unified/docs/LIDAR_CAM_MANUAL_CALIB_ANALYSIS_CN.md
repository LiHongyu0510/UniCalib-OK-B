# LiDAR-相机外参手动调整：工程实现与「直观性」设计分析

## 1. 核心结论：如何让「调 6 个参数」变直观？

**SensorsCalibration / UniCalib 的做法并不是换一套参数或换一种数学表示，而是：**

- **主视图 = 当前外参的「结果」**：把 3D 点云按当前 6-DOF 外参投影到 2D 图像上，形成**点云叠加图**。
- 用户**不看 6 个数字**，而是**看叠加图里点云与图像边缘/特征是否对齐**；对齐得越好，标定越准。
- 6 个参数（Roll/Pitch/Yaw + Tx/Ty/Tz）只是**控制手段**，通过键盘或面板的 ± 按钮微调；**直观性来自「所见即所得」的视觉反馈**。

也就是说：**仍然是「三维点云投影到二维图像 + 调 6 个参数」**，但通过「主视图 = 投影叠加图 + 实时重绘」把「调参数」变成了「对着图像对齐点云」，从而更直观。

---

## 2. 手动调整在工程中的实现路径

### 2.1 入口与数据流

- **入口**：  
  - 脚本：`./calib_unified_run.sh --run --task lidar-cam --manual`  
  - 或独立可执行：`unicalib_lidar_camera --config config.yaml --manual`  
- **数据**：精标定完成后，Pipeline 把**首帧**（LiDAR 一帧点云 + 相机一帧图像 + 相机内参）写入 `manual_cache_`，手动阶段只使用这一帧，不加载多帧。
- **调用链**：  
  `Pipeline::run_manual_stage("lidar_cam")` → `ManualCalibSession::run_lidar_cam(scan, image, cam_intrin, auto_result, ...)` → 根据配置二选一：
  - **方案 A（默认）**：`run_6dof_interactive_loop_lidar_cam(...)`，OpenCV 窗口 + 点云叠加图 + 键盘 6-DOF。
  - **方案 B**：`run_lidar_cam_pangolin_panel(...)`（需 `UNICALIB_WITH_PANGOLIN` 且 `use_pangolin_manual_panel=true`），Pangolin 窗口：左侧 6-DOF 面板 + 右侧点云叠加图，风格与 SensorsCalibration 一致。

### 2.2 方案 A：OpenCV 叠加循环（`manual_calib.cpp`）

- **主循环**（约 201–277 行）：
  1. 取当前外参 `cur = adjuster.current()`。
  2. 将 Pipeline 存储的 **T_lidar_to_cam** 转为 `project_to_image` 所需的 **T_cam_in_lidar**（即 `cur.SE3_TargetInRef().inverse()`）。
  3. 调用 `LiDARProjectionViz::project_to_image(cloud, image, T_cam_in_lidar, fx, fy, cx, cy, W, H, ...)` 得到**点云叠加图** `overlay`。
  4. 在 `overlay` 上绘制 T (m)、RPY (deg) 和按键说明（Q/A Roll、W/S Pitch、…、Enter=Accept、Esc=Cancel）。
  5. `cv::imshow` 显示，`cv::waitKey(80)` 处理按键，调用 `adjuster.apply(...)` 更新外参；Enter 接受，Esc 取消。

- **直观性体现**：用户每次按键后，下一帧立即用新外参重绘叠加图，**直接看点云与图像边缘是否更对齐**，无需心算 6 个数。

### 2.3 方案 B：Pangolin 面板（`manual_lidar_cam_window.cpp`，源自 SensorsCalibration）

- 布局与 SensorsCalibration 的 `run_lidar2camera.cpp` 一致：
  - **左侧面板**：6-DOF 的 ± 按钮（+x deg, -x deg, … , +z trans, -z trans）、**Intensity Color**、**Overlap Filter**、**deg step**、**t step(cm)**、**point size**、**Reset**、**Save Image**、**Accept**。
  - **右侧主视图**：当前外参下的点云投影叠加图（`projector.projectToImage(image, cam_intrin, T_lidar_to_cam)`），每帧重绘。
- 键盘：q/a, w/s, e/d, r/f, t/g, y/h 与 6-DOF 对应，U 撤销，Enter 接受，Esc 取消。
- **Intensity Color**：按强度着色，便于看车道线等与图像对齐（SensorsCalibration README 明确提到用于检查地面车道线对齐）。
- **Overlap Filter**：按深度过滤重叠点（如 0.4 m 内），减少重叠带来的视觉干扰。
- **deg step / t step**：可调步长，便于先粗调再细调。

因此，**「不够直观」的问题**是通过「主视图 = 投影叠加图 + 实时更新 + 可选强度图/过滤/步长」来缓解的，而不是换参数化或换标定模型。

---

## 3. 6-DOF 调整器（ManualExtrinsicAdjuster）

- **旋转**：`apply_delta_rotation(roll_deg, pitch_deg, yaw_deg)` 使用 RPY 顺序构造增量旋转，**右乘**到当前外参：`T_new = T_old * delta`（在 target 系即相机系下施加旋转），与面板「+x deg」等含义一致。
- **平移**：`apply_delta_translation(dx_m, dy_m, dz_m)` 直接在外参的平移向量上加减。
- **步长**：由 `ManualAdjustStep` 配置（如 `rot_step_deg=0.1`、`trans_step_m=0.005`）；支持 Shift 倍率（快速模式）。
- **撤销/重做**：`undo()` / `redo()`，栈深度可配置。

用户无需理解欧拉角或变换矩阵，只需记住「Q/A=Roll、W/S=Pitch、…」或点面板按钮，观察叠加图即可。

---

## 4. 投影与坐标系约定（避免搞反）

- Pipeline 存储：**T_lidar_to_cam**（`p_cam = T * p_lidar`），即 `ExtrinsicSE3::SE3_TargetInRef()`。
- `LiDARProjectionViz::project_to_image` 的形参是 **T_cam_in_lidar**（相机在 LiDAR 系下位姿），内部用 `T_lidar_to_cam = T_cam_in_lidar.inverse()` 再投影。因此 OpenCV 方案里必须传 `cur.SE3_TargetInRef().inverse()`。
- Pangolin 使用的 `LidarProjector::projectToImage` 形参为 **T_lidar_to_cam**，因此这里要传 `cur.SE3_TargetInRef().matrix()`，**不要**再 `.inverse()`。  
详见 `MANUAL_LIDAR_CAM_ANALYSIS.md`。

---

## 5. 可选：点击精化（ManualClickRefiner）

除纯 6-DOF 交互外，工程还提供「点击精化」：

- 用户在一张图上点若干** 3D–2D 对应**（LiDAR 点 + 像素点），`ManualClickRefiner::refine_lidar_cam` 用 Ceres 做**重投影误差最小化**得到优化外参。
- 适合：已有大致对齐，希望用少量精确对应点再收一收精度；与「看叠加图调 6 个参数」互补。

---

## 6. SensorsCalibration 原始设计（本仓库 thrid_party/SensorsCalibration）

- **README**（`lidar2camera/README.md`）说明：  
  - 标定窗口 = **左侧控制面板 + 右侧点云投影图**；  
  - 用户**通过观察点云与图像是否对齐**来调整外参，对齐后点 Save 保存。
- **run_lidar2camera.cpp** 主循环（约 329–454 行）：
  - 每次面板或键盘修改外参后，调用 `projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_)` 重绘**右侧主视图**；
  - 12 个按键对应 6-DOF 的 ±，与 UniCalib 的 q/a,w/s,e/d,r/f,t/g,y/h 一致。
- **projector_lidar.hpp**：`ProjectToRawMat` 内 `projCloud2d = K*(R*oriCloud + T)`，即 `p_cam = R*p_lidar + t`，与 T_lidar_to_cam 约定一致。

UniCalib 的 LiDAR-Cam 手动流程与上述逻辑一致：**主视图即标定质量视图（点云叠加图），6-DOF 仅作输入手段，直观性来自「对齐即正确」的视觉判断**。

---

## 7. 小结表

| 问题 | 工程做法 |
|------|----------|
| 调 6 个参数不直观？ | **主视图 = 点云投影叠加图**，用户看「边缘/特征是否对齐」，不直接看 6 个数。 |
| 如何操作 6-DOF？ | 键盘 q/a,w/s,e/d,r/f,t/g,y/h 或 Pangolin 左侧 ± 按钮；步长可调，支持 Undo。 |
| 如何更易判断对齐？ | 可选 **Intensity Color**（强度图，看车道线等）、**Overlap Filter**、调节 **point size**。 |
| 数据从哪来？ | 精标定后缓存的**首帧**（一帧点云 + 一帧图像 + 内参），不加载多帧。 |
| 与 SensorsCalibration 关系？ | UniCalib 参考并融合其 lidar2camera/manual_calib：主视图=投影图、12 键、Pangolin 面板、Intensity/Overlap/步长等。 |

**一句话**：SensorsCalibration / UniCalib 并没有用另一种参数化替代 6-DOF，而是把「调 6 个参数」嵌进「实时看投影叠加图、以视觉对齐为准」的交互里，用**所见即所得**来提升直观性。
