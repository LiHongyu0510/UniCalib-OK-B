# LiDAR-Camera 手动调整代码深度分析

## 1. 坐标系与外参约定（结论先置顶）

- **精标定与 Pipeline 存储**（`lidar_camera_calib.cpp`）：`ExtrinsicSE3.SE3_TargetInRef()` 存的是 **T_lidar_to_cam**，即 `p_cam = T * p_lidar`（将 LiDAR 点变到相机系）。变量名虽写 `T_cam_in_lidar`，语义是 “将 LiDAR 点变换到 cam 坐标系”，即 lidar→cam。
- **cloud_viewer::project_to_image**：形参名为 `T_cam_in_lidar`，**语义是「相机在 LiDAR 系下的位姿」**，即 `p_lidar = T_cam_in_lidar * p_cam`；内部用 `T_lidar_to_cam = T_cam_in_lidar.inverse()`，再 `p_cam = T_lidar_to_cam * p_lidar`。
- **LidarProjector::projectToImage**：形参为 **T_lidar_to_cam**（4×4），即 `p_cam = R*p_lidar + t`，与 pipeline 存储一致。

因此：
- 调用 **project_to_image** 时，应传入「相机在 LiDAR 下的位姿」，即 **SE3_TargetInRef().inverse()**（因为存的是 T_lidar_to_cam，其 inverse 才是 T_cam_in_lidar）。
- 调用 **projector.projectToImage** 时，应传入 **SE3_TargetInRef().matrix()**，即 T_lidar_to_cam，**不要**再 .inverse()。

---

## 2. 已发现的严重问题

### 2.1 【严重】OpenCV 叠加循环中传入 project_to_image 的变换反了

**位置**：`manual_calib.cpp` 第 224–226 行。

**现状**：
```cpp
Sophus::SE3d T_cam_in_lidar = cur.SE3_TargetInRef();
cv::Mat overlay = LiDARProjectionViz::project_to_image(
    *scan.cloud, image, T_cam_in_lidar, ...);
```

**问题**：Pipeline 里 `SE3_TargetInRef()` 存的是 **T_lidar_to_cam**（p_cam = T * p_lidar）。而 `project_to_image` 的形参 `T_cam_in_lidar` 表示「相机在 LiDAR 系下的位姿」，内部会做 `T_lidar_to_cam = T_cam_in_lidar.inverse()` 再投影。这里把 T_lidar_to_cam 当成 T_cam_in_lidar 传入，相当于投影时用了 **inverse(T_lidar_to_cam)**，即用错了变换，投影结果会错（旋转、平移都会反）。

**修复**：传入「相机在 LiDAR 下的位姿」，即当前外参的 inverse：
```cpp
Sophus::SE3d T_cam_in_lidar = cur.SE3_TargetInRef().inverse();
cv::Mat overlay = LiDARProjectionViz::project_to_image(
    *scan.cloud, image, T_cam_in_lidar, ...);
```

---

### 2.2 【严重】Pangolin 窗口中传给 Projector 的 T_lidar_to_cam 反了

**位置**：`manual_lidar_cam_window.cpp` 第 175–178 行。

**现状**：
```cpp
Eigen::Matrix4d T_lidar_to_cam = cur.SE3_TargetInRef().inverse().matrix();
cv::Mat frame = projector.projectToImage(image, cam_intrin, T_lidar_to_cam);
```

**问题**：Pipeline 存的 `SE3_TargetInRef()` 已是 **T_lidar_to_cam**。Projector 的 `projectToImage` 需要的也是 T_lidar_to_cam（p_cam = R*p_lidar + t）。这里却传了 `.inverse().matrix()`，即把 T_cam_in_lidar 传给了期望 T_lidar_to_cam 的接口，投影会错。

**修复**：直接传当前外参矩阵，不要 inverse：
```cpp
Eigen::Matrix4d T_lidar_to_cam = cur.SE3_TargetInRef().matrix();
cv::Mat frame = projector.projectToImage(image, cam_intrin, T_lidar_to_cam);
```

---

## 3. 逐段逻辑与潜在问题

### 3.1 ManualExtrinsicAdjuster 的旋转/平移语义

- **apply_delta_rotation**：`T_new = T_old * delta`（右乘，delta 仅旋转）。即在 **target（相机）系** 下施加旋转，与 “+x deg” 等面板含义一致，无逻辑错误。
- **apply_delta_translation**：`POS_TargetInRef += (dx,dy,dz)`。对存为 T_lidar_to_cam 的 SE3，其平移部分是在相机系下的位移，与 “+ x trans” 等一致，无逻辑错误。

### 3.2 run_6dof_interactive_loop_lidar_cam（OpenCV 叠加）

| 行/逻辑 | 说明 | 问题/备注 |
|--------|------|------------|
| W/H 用 cam_intrin 或 image 尺寸 | 合理 | 若内参 width/height 与 image 不一致，可能造成越界或裁切；一般应一致。 |
| project_to_image 传入 T | 见上 | **严重**：当前传的是 T_lidar_to_cam，应传 T_cam_in_lidar = inverse。 |
| depth 用 p_lidar.norm()（cloud_viewer 内） | 着色用 | 用 LiDAR 系下范数近似深度，与 “近红远蓝” 一致；若要做相机深度可改用 p_cam.z()，非必须。 |
| 未使用 `bool fast` | 仅未用 Shift 倍率 | 非错误；若需可后续用 key 检测 Shift 并传 true。 |

### 3.3 manual_lidar_cam_window（Pangolin）

| 行/逻辑 | 说明 | 问题/备注 |
|--------|------|------------|
| T_lidar_to_cam 传 Projector | 见上 | **严重**：当前用了 .inverse()，应直接用 SE3_TargetInRef().matrix()。 |
| Reset 调 set_initial_extrinsic | 正确 | 恢复为自动标定初值。 |
| Save 用当前 cur 再 projectToImage | 已修 | 原先 Save 也用了 .inverse()，已改为 .matrix()，与主循环一致。 |
| kbhit/getchar 与 Pangolin 事件 | 平台相关 | 非 GUI 环境下 kbhit 可能不可用；仅影响键盘 6-DOF，面板仍可用。 |

### 3.4 LidarProjector（projector_lidar.h）

| 逻辑 | 说明 | 问题/备注 |
|------|------|------------|
| projectToRawMat 中 projCloud2d = K*(R*oriCloud_ + T) | 即 p_cam = R*p_lidar + t | 与 T_lidar_to_cam 定义一致，无误。 |
| 去畸变用 initUndistortRectifyMap | 标准流程 | dist 不足 8 时已 resize 补 0，无问题。 |
| D 维数与 OpenCV 要求 | 已补到 8 | 若实际 dist 大于 8 会截断，一般 pinhole 为 4–5，风险低。 |

### 3.5 其他潜在问题（非致命）

- **eulerAngles(0,1,2)**：Eigen 的 eulerAngles(0,1,2) 是 (roll,pitch,yaw) 的提取顺序，但与旋转合成顺序（例如 R(z)*R(y)*R(x)）需一致；当前仅用于显示，不参与投影，影响小。
- **图像/内参尺寸不一致**：若 `cam_intrin.width/height` 与 `image.cols/rows` 不同，project_to_image 用 W/H 做边界，可能裁切或留黑边；建议调用方保证一致或文档说明。
- **点云为空或 NaN**：project_to_image 与 Projector 内都有 continue 跳过，不会崩；空云时叠加图仅图像，可接受。
- **Pangolin 未链接 pango_vars**：若未链接，CreatePanel/Var 会链接错误；当前 CMake 已通过 pango_display 拉取 pango_vars，一般无问题。

---

## 4. 修复汇总

1. **manual_calib.cpp**（OpenCV 叠加）：  
   将传入 `project_to_image` 的位姿改为 **T_cam_in_lidar = cur.SE3_TargetInRef().inverse()**。
2. **manual_lidar_cam_window.cpp**（Pangolin）：  
   传给 `projector.projectToImage` 的矩阵改为 **cur.SE3_TargetInRef().matrix()**，去掉 `.inverse()`。

修好后，手动调整时的叠加与精标定/存储的外参约定一致，投影结果正确。
