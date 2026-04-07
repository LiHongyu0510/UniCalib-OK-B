# 相机-相机外参标定：融合 SensorsCalibration 手动可视化微调方案

## 1. 目标

将 **SensorsCalibration** 的手动可视化微调功能（交互模式、主视图即标定质量视图、可选面板）融合到 **UniCalib 相机-相机外参标定**的手动调整中，使 Cam-Cam 手动阶段与 LiDAR-Cam 一样具备**实时可视化**（对极线/对应点），并与 SensorsCalibration 的按键与界面风格一致。

---

## 2. SensorsCalibration 源码与 UniCalib 对应关系

### 2.1 已融合部分（LiDAR-Camera）

| SensorsCalibration | UniCalib 对应 | 说明 |
|--------------------|---------------|------|
| `lidar2camera/manual_calib/projector_lidar.hpp` | `calib_unified/include/unicalib/viz/projector_lidar.h` | 点云投影到图像，风格一致 |
| `lidar2camera/manual_calib/run_lidar2camera.cpp` 主循环：`current_frame = projector.ProjectToRawImage(...)` + 键盘 12 键 | `manual_calib.cpp` 中 `run_6dof_interactive_loop_lidar_cam`：`overlay = LiDARProjectionViz::project_to_image(...)` + q/a,w/s,e/d,r/f,t/g,y/h | 主视图 = 点云叠加图，6-DOF 键一致 |
| run_lidar2camera 的 Pangolin 面板 (cp: +/- degree/trans, Reset, Save) | `manual_lidar_cam_window.h` 的 `run_lidar_cam_pangolin_panel` | 可选 Pangolin 方案 B |

### 2.2 待融合到 Cam-Cam 的部分

SensorsCalibration **没有**相机-相机对极线可视化的现成代码；可融合的是**交互与界面模式**，对极线绘制在 UniCalib 内用 OpenCV 实现。

| SensorsCalibration 模式 | 融合到 Cam-Cam 的体现 |
|-------------------------|------------------------|
| **run_lidar2camera**：主视图 = 标定质量图（点云叠加），每帧重绘 | 主视图 = **左右图拼接 + 对极线**，每帧根据当前外参重绘对极线 |
| **run_lidar2camera / run_avm**：12 键 q/a,w/s,e/d,r/f,t/g,y/h，右乘增量 | 保持现有 `ManualExtrinsicAdjuster` 的 q/a,w/s,e/d,r/f,t/g,y/h（与 SC 一致），U Undo、Enter/Esc |
| **run_lidar2camera**：步长 `cali_scale_degree_` / `cali_scale_trans_` | 使用现有 `ManualAdjustStep`（rot_step_deg, trans_step_m） |
| **run_lidar2camera**：Reset、Save Image 按钮 | 首版 OpenCV 循环可只做 Enter/Esc；Pangolin 版可加 Reset、Save Image |
| **run_avm**：`CalibrationInit` 构建 12 个 4x4 增量矩阵 | UniCalib 已用 `apply_delta_rotation` / `apply_delta_translation` 等价实现 |

---

## 3. 实现步骤（融合后的具体改动）

### 3.1 对极线渲染（对标 SensorsCalibration 的 ProjectToRawImage）

- **位置**：[`calib_unified/src/pipeline/manual_calib.cpp`](calib_unified/src/pipeline/manual_calib.cpp) 中实现 `ManualClickRefiner::render_stereo_epipolar`（头文件已声明于 [`manual_calib.h`](calib_unified/include/unicalib/pipeline/manual_calib.h)）。
- **输入**：img0, img1, intrin0, intrin1, 当前 ExtrinsicSE3；可选 clicks（对应点）。
- **逻辑**：
  - 由 R, t 与 K0, K1 构造 E = [t]_x * R，F = K1^{-T} * E * K0^{-1}（注意 OpenCV 的 F 与点的左右顺序）。
  - 左图网格采样点（如步长约 1/6 图宽高），`cv::computeCorrespondEpilines` 得右图极线并绘制；可选右图采样画左图极线。
  - 若 clicks 非空：画对应点及在另一图的极线。
  - 返回左右图水平拼接的 `cv::Mat`（与 `visualize_stereo_rectification` 拼接方式一致）。

### 3.2 Cam-Cam 专用 6-DOF 交互循环（对标 run_6dof_interactive_loop_lidar_cam）

- **新增函数**：`run_6dof_interactive_loop_cam_cam(adjuster, window_title, img_cam0, img_cam1, intrin0, intrin1)`，与 `run_6dof_interactive_loop_lidar_cam` 并列放在同一匿名 namespace。
- **行为**（与 SensorsCalibration 主循环 + UniCalib LiDAR-Cam 一致）：
  - 若 img0 或 img1 为空，回退到现有 `run_6dof_interactive_loop(adjuster, window_title)`（仅文字）。
  - 循环内：`cur = adjuster.current()` → `vis = render_stereo_epipolar(img0, img1, {}, intrin0, intrin1, cur)` → 在 vis 上叠加 T (m)、RPY (deg) 及按键说明（Q/A W/S E/D R/F T/G Y/H U Undo, Enter=Accept, Esc=Cancel）→ `cv::imshow` + `cv::waitKey(80)`，处理按键与 Enter/Esc。
  - 按键映射与现有 `run_6dof_interactive_loop` 完全一致（q/a,w/s,e/d,r/f,t/g,y/h,u, Enter, Esc）。

### 3.3 在 run_cam_cam 中启用可视化循环

- **修改**：`ManualCalibSession::run_cam_cam` 中，将  
  `run_6dof_interactive_loop(adjuster_, "Cam-Cam Manual Adjust")`  
  改为  
  `run_6dof_interactive_loop_cam_cam(adjuster_, "Cam-Cam Manual Adjust", img_cam0, img_cam1, intrin0, intrin1)`。
- 保持 `enable_interactive_gui` 判断及 accept/cancel 写回逻辑不变。

### 3.4 文档

- 在 [MANUAL_EXTRINSIC_ADJUSTMENT.md](calib_unified/docs/MANUAL_EXTRINSIC_ADJUSTMENT.md) 中补充：Cam-Cam 手动调整时主视图为**左右目拼接 + 对极线**，对极线对齐越好标定越准；按键与 LiDAR-Cam 一致。

### 3.5 可选（后续）：Cam-Cam 的 Pangolin 面板

- 参考 `run_lidar2camera.cpp` 的 Pangolin 布局：主视图 + 左侧 `cp` 面板（deg step、t step、+/- 6-DOF、Reset、Save Image、Accept）。
- 主视图显示对极线拼接图（每帧用当前外参调用 `render_stereo_epipolar`，再 Upload 到 GlTexture 显示）。
- 可与现有 `run_lidar_cam_pangolin_panel` 并列增加 `run_cam_cam_pangolin_panel`，由配置或 `use_pangolin_manual_panel` 在 Cam-Cam 时启用。

---

## 4. 小结

- **融合内容**：SensorsCalibration 的**手动微调交互模式**（6-DOF 键、主视图=标定质量图、步长/Reset/Save 思路）融合到 UniCalib Cam-Cam；**可视化内容**为对极线（SensorsCalibration 无现成实现，在 UniCalib 内用 OpenCV 实现）。
- **代码对齐**：Cam-Cam 的 OpenCV 循环与现有 `run_6dof_interactive_loop_lidar_cam` 结构一致，仅把「点云投影图」换成「左右图 + 对极线」，与 SensorsCalibration 的「主视图即 ProjectToRawImage 结果」一致。
- **涉及文件**：`calib_unified/src/pipeline/manual_calib.cpp`（实现 `render_stereo_epipolar`、新增 `run_6dof_interactive_loop_cam_cam`、修改 `run_cam_cam` 调用）、可选更新 `manual_calib.h`（若 `render_stereo_epipolar` 需暴露或签名微调）、`calib_unified/docs/MANUAL_EXTRINSIC_ADJUSTMENT.md`。
