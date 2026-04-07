# 外参标定可视化深入分析：LiDAR-Cam / LiDAR-LiDAR / Cam-Cam

本文档针对三种外参标定（LiDAR-Camera、LiDAR-LiDAR、Camera-Camera）的**可视化链路**做逐项分析，指出**是否存在问题**及**根因**。

---

## 0. Executive Summary

| 标定类型 | 窗口是否创建 | 是否向 3D 窗口推送数据 | 是否 spin/等待用户 | 是否 close_display | 主要问题 |
|----------|--------------|------------------------|--------------------|--------------------|----------|
| **LiDAR-Camera** | ✓ Pipeline 创建 | ✓ 精标定结束后调用 update_realtime_data + render_and_handle_input | ✓ spin() 后 close_display | ✓ | 已修复：3D 窗口可显示首帧点云与优化后外参；投影图仍写 PNG |
| **LiDAR-LiDAR** | ✓ main 创建 | ✗ 未调用 visualizer；generate_visualization 为占位 | ✗ 无 spin，无 close_display | ✗ | 窗口打开后无内容、流程结束直接退出，窗口依赖进程退出才关 |
| **Camera-Camera** | ✓ Pipeline 创建 | ✗ 标定器未使用 visualizer_；立体矫正只写文件 | ✗ 无 spin，无 close_display | ✗ | 同上；且 visualize_stereo_rectification 从未被调用 |

**结论**：三种外参标定的可视化均存在**结构性缺失**——要么不向 Pangolin 推送任何标定结果（点云/图像/外参），要么不等待用户查看、不显式关窗，导致“界面无内容”或“标定完直接退出”的体验。

---

## 1. LiDAR-Camera 外参标定可视化

### 1.1 当前流程（Pipeline）

1. **创建与启动**（`calib_pipeline.cpp`）
   - `lidar_cam_viz = CalibVisualizer::Create()`
   - `lidar_cam_viz->start_display("UniCalib LiDAR-Camera", 1280, 720)` → Pangolin 窗口创建，首帧泵送 12 帧（仅网格+坐标轴）
   - `calibrator.set_visualizer(lidar_cam_viz)`，`calibrator.enable_realtime_viz(true)`

2. **标定过程中**
   - `LiDARCameraCalibrator::calibrate_edge_align` 在 Ceres 完成后调用：
     - `visualizer_->report_progress(..., 0.8, "Ceres 优化完成")`
     - **已实现**：取首帧点云与优化后外参，调用 `visualizer_->update_realtime_data(ref_cloud, nullptr, extrinsic)` 向 Pangolin 推送点云与外参，并泵送若干次 `render_and_handle_input()`，使 3D 窗口显示点云与坐标系。

3. **标定完成后**
   - `calibrator.visualize_projection(...)` → 仅将投影图 **cv::imwrite** 到 PNG，不显示在 3D 窗口。
   - `lidar_cam_viz->spin()` → 阻塞直至用户关窗（已修复为手动关窗）。
   - `lidar_cam_viz->close_display()` → 正常关闭。

### 1.2 已修复与仍存在

| 项目 | 说明 |
|------|------|
| **3D 窗口点云/外参** | 已修复：Ceres 完成后调用 `update_realtime_data(first_pair.first->cloud, nullptr, extrin)` 并泵送 8 次 `render_and_handle_input()`，3D 窗口可显示首帧点云与优化后外参。 |
| **结果落盘** | 投影图仍仅写 PNG，用户可打开结果目录中的投影图查看。 |

### 1.3 环境与依赖（MIAS-LCEC 粗标定）

- **pyautogui**：LcMatch_CApi (.so) 依赖 `pyautogui`，若未安装则自动回退到纯 PyTorch 路径，功能可用但无 C API 加速。
- **OverlapTransformer**：若未加载预训练权重，日志会提示“OverlapTransformer 未加载，使用零特征”，粗标定仅用 SAM，可能影响初值稳定性；可配置模型路径以启用。
- **PyTorch 与 GPU**：若本机 GPU 架构（如 RTX 50 系 sm_120）未被当前 PyTorch 支持，会回退 CPU，粗标定耗时会明显增加；可安装支持该架构的 PyTorch 或使用其他机器。

---

## 2. LiDAR-LiDAR 外参标定可视化

### 2.1 当前流程（apps/lidar_lidar_extrin/main.cpp）

1. **创建与启动**
   - `viz = CalibVisualizer::Create()`
   - `viz->start_display("UniCalib LiDAR-LiDAR", 1280, 720)`
   - `calibrator.set_visualizer(viz)`，`calibrator.enable_realtime_viz(true)`

2. **标定过程中**
   - `LiDARLiDARCalibrator` 内部**从未**使用 `visualizer_`：
     - 无 `update_realtime_data`
     - 无 `show_aligned_point_clouds` 或任何 `show_*`
     - 无 `report_progress` 到 visualizer
   - `generate_visualization(scan_ref, scan_target, extrinsic, output_path)` 仅为**占位**：只打日志 `"[LiDAR-LiDAR] 可视化报告: {} (占位)"`，不写文件、不调用 visualizer。
   - 且 **main 从未调用** `generate_visualization`，因此即使日后实现该函数，当前流程也不会触发。

3. **标定完成后**
   - main 直接 `UNICALIB_INFO("LiDAR-LiDAR 标定完成...")` 后结束。
   - **无** `viz->spin()`、**无** `viz->close_display()`。
   - 窗口依赖进程退出才被系统关闭；用户无法“手动关窗后程序再继续”。

### 2.2 存在的问题

| 问题 | 说明 |
|------|------|
| **窗口无内容** | 标定器未向 visualizer 推送任何点云或外参，Pangolin 始终为空。 |
| **generate_visualization 未实现且未被调用** | 占位实现 + main 未调用，无法生成任何可视化输出。 |
| **无 spin/close_display** | 无法“标定完成→保持窗口→用户关窗→再退出”；窗口随进程退出而关。 |

### 2.3 建议修复（LiDAR-LiDAR）

- 在 `LiDARLiDARCalibrator::calibrate_two_stage` 成功得到外参后：
  - 若 `enable_realtime_viz_ && visualizer_`：取一对点云（如 ref 首帧、target 首帧）及外参，调用 `visualizer_->update_realtime_data(ref_cloud, target_cloud, extrinsic)`（需将 ExtrinsicSE3 转为 SE3），并泵送若干帧渲染；或调用 `show_aligned_point_clouds` 等已有接口。
- 在 main 中，每个标定对完成后（或全部完成后）：
  - 调用 `generate_visualization(ref_scan, target_scan, result, output_path)`，并在其中实现：写对齐结果图/点云到文件，且若 visualizer 可用则向 3D 窗口推送点云并泵送渲染。
- 在 main 末尾（所有对处理完）：
  - 若创建了 viz：先 `viz->spin()` 等待用户关窗，再 `viz->close_display()`。

---

## 3. Camera-Camera 外参标定可视化

### 3.1 当前流程（Pipeline run_fine_cam_cam）

1. **创建与启动**
   - `cam_cam_viz = CalibVisualizer::Create()`
   - `cam_cam_viz->start_display("UniCalib Cam-Cam", 1280, 720)`
   - 对每个相机对：`calib.set_visualizer(cam_cam_viz)`，`calib.enable_realtime_viz(true)`

2. **标定过程中**
   - `CamCamCalibrator::calibrate_two_stage` 及内部 BA/立体标定**从未**使用 `visualizer_`：
     - 无 `show_camera_cam_result`
     - 无 `show_matched_points_3d` / `show_epipolar_lines` / `show_disparity_map`
     - 无 `report_progress` 到 visualizer
   - `visualize_stereo_rectification` 仅将立体矫正结果 **cv::imwrite** 到文件，且**从未被 pipeline 或 apps 调用**，因此当前流程下立体矫正图也不会生成。

3. **标定完成后**
   - Pipeline 直接 `return r`，**无** `cam_cam_viz->spin()`、**无** `cam_cam_viz->close_display()`。
   - 窗口同样依赖进程退出才关闭；用户无法“查看后手动关窗”。

### 3.2 存在的问题

| 问题 | 说明 |
|------|------|
| **3D 窗口无内容** | 标定器未调用任何 visualizer 的 show_*，Pangolin 始终为空。 |
| **立体矫正可视化未接入流程** | `visualize_stereo_rectification` 仅实现写文件，且无调用点，立体矫正图不会输出。 |
| **无 spin/close_display** | 与 LiDAR-LiDAR 相同，无“等待用户关窗”与显式关窗。 |

### 3.3 建议修复（Cam-Cam）

- 在 `run_fine_cam_cam` 中，每个相机对标定成功后：
  - 若存在 `cam_cam_viz`：构造 `CameraCameraVizData`（外参、匹配点、立体对、视差等），调用 `cam_cam_viz->show_camera_cam_result(data)`（或按需调用 show_epipolar_lines / show_disparity_map）；并调用 `visualize_stereo_rectification` 将立体矫正图写入结果目录。
- 在 `run_fine_cam_cam` 末尾（所有对处理完）：
  - 若 `cam_cam_viz` 非空：先 `cam_cam_viz->spin()`，再 `cam_cam_viz->close_display()`。

---

## 4. 共通问题与设计建议

### 4.1 共通问题

1. **Pangolin 仅作“空壳”**  
   三种标定都创建了 Pangolin 窗口并 set_visualizer，但标定逻辑几乎不向 visualizer 推送点云/图像/外参，导致 3D 窗口没有标定结果可显示。

2. **结果仅写文件、不显式展示**  
   LiDAR-Cam 的投影图、Cam-Cam 的立体矫正图都只写 PNG，不通过 3D/2D 窗口展示；LiDAR-LiDAR 连文件都未生成（generate_visualization 占位且未调用）。

3. **窗口生命周期不统一**  
   仅 LiDAR-Cam 在 pipeline 中有 spin + close_display；LiDAR-LiDAR 与 Cam-Cam 既无 spin 也无 close_display，窗口依赖进程退出关闭，体验不一致。

### 4.2 设计建议

- **统一约定**：凡启用 viz，则（1）标定成功后向 visualizer 推送至少一帧结果（点云/图像/外参），（2）调用 spin() 等待用户关窗，（3）再 close_display()。
- **接口使用**：在标定模块内显式调用 `update_realtime_data` / `show_*`，而不是只 report_progress；report_progress 可作为进度与日志的补充，不作为唯一可视化路径。
- **立体/投影图**：在 pipeline 或 app 中显式调用 `visualize_stereo_rectification`（Cam-Cam）、`visualize_projection`（LiDAR-Cam），并考虑在 3D 窗口或单独 2D 窗口中展示同一张图（若 API 支持）。

---

## 5. 小结表（便于排查）

| 标定类型 | 谁创建 viz | 谁向 viz 推送数据 | 谁调 spin/close | 立体/投影图谁写 | 当前现象 |
|----------|-------------|--------------------|------------------|------------------|----------|
| LiDAR-Cam | Pipeline | 精标定后 update_realtime_data + render_and_handle_input | Pipeline spin+close | calibrator.visualize_projection 写 PNG | 3D 窗口显示点云与外参，关窗后退出正常 |
| LiDAR-LiDAR | main | 无；generate_visualization 占位且未调用 | 无 | 无 | 窗口空，进程结束才关窗 |
| Cam-Cam | Pipeline | 无 | 无 | visualize_stereo_rectification 未调用 | 窗口空，立体矫正图未生成，进程结束才关窗 |

上述问题修复后，三种外参标定的可视化将能：在 3D 窗口中看到标定结果（点云/外参或相机对结果）、在流程结束前通过手动关窗结束展示、并统一关闭 Pangolin 窗口。
