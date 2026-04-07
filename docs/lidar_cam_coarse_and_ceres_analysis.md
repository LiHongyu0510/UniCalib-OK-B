# LiDAR-Cam 粗标定与 Ceres 边缘对齐 — 实现说明

## 0. Executive Summary

| 项目 | 状态 | 说明 |
|------|------|------|
| **MVP** | ✅ 已实现 | PCD 读入兼容（Open3D/NumPy 失败时回退 ASCII）；Pipeline 内粗标定真实调用 MIASLCECAdapter；粗标定结果传入精标定初值 |
| **V1** | ✅ 已实现 | 边缘对齐接入 Ceres：NumericDiffCostFunction 最小化 -NCC，多帧参与优化 |
| **V2** | 部分就绪 | 多相机由 `lidar_camera_camera_list` / `lidar_camera.pairs` 支持；质量由 `lidar_cam_rms_threshold` 与 `needs_manual_refine` 体现；批量/回放可多次执行脚本 |

---

## 1. 粗标定（MVP）

### 1.1 环境与 PCD 读入

- **问题**：日志中 `读取 PCD 失败: numpy.dtype size changed...` 为 Open3D 与当前 NumPy ABI 不兼容。
- **处理**：`calib_unified/scripts/run_lidar_cam_coarse.py` 中，在 `load_pcd_points()` 内对 Open3D 的 `Exception` 做捕获，若错误信息含 `numpy`/`dtype`/`binary`/`incompatibility` 则回退到 ASCII PCD 读取，保证脚本在 Docker 内可跑通。

### 1.2 repo_dir 解析

- **问题**：配置里 `mias_lcec.repo_dir: "calib_unified"` 为相对路径，在容器内需相对 `ai_root` 解析。
- **处理**：
  - `ai_coarse_calib.cpp` 的 `AICoarseCalibManager::setup_adapters()` 中，若 `mias_lcec.repo_dir` 非空且非绝对路径，则设为 `ai_root + "/" + repo_dir`。
  - `calib_pipeline.cpp` 的 `run_coarse_stage(LIDAR_CAM_EXTRIN)` 中同样对 `mias_lcec_repo_dir` 做相对 `ai_models_root` 的解析。

### 1.3 Pipeline 内粗标定真实调用

- **原状**：`run_coarse_stage(LIDAR_CAM_EXTRIN)` 仅打占位日志并返回成功。
- **现状**：
  - 构建 `AICoarseCalibManager`，从 `PipelineConfig` 读取 `mias_lcec_*`。
  - 数据来源：ROS2 bag 时用 `UnifiedDataLoader` 取首帧并写出临时 PCD/图像；文件模式时从 `lidar_data_dir` 与首相机图像目录取首帧。
  - 调用 `ai_mgr.coarse_lidar_cam(pcd_file, image_file, cam_intrin, ...)`，成功则将 `coarse_result->SE3_TargetInRef()` 写入 `coarse_lidar_cam_init_`。
- **精标定使用粗结果**：`run_fine_lidar_camera()` 中调用 `calibrator.calibrate_two_stage(..., coarse_lidar_cam_init_, ...)`，不再传 `std::nullopt`。

### 1.4 配置

- `unicalib_example.yaml` 中 `third_party.mias_lcec` 可配 `repo_dir`、`calib_script`、`model_path`（可选，供后续真实 MIAS-LCEC 模型使用）。
- `joint_calib` 在构建 `pipe_cfg` 时从 `third_party.mias_lcec` 写入 `pipe_cfg.mias_lcec_repo_dir`、`mias_lcec_calib_script`、`mias_lcec_timeout_sec`、`mias_lcec_work_dir`。

---

## 2. 边缘对齐 Ceres 优化（V1）

### 2.1 实现方式

- **目标**：最大化多帧上的边缘 NCC，等价于最小化 `-NCC`。
- **参数**：6 维（angle-axis 3 + translation 3），与当前外参 SE3 对应。
- **残差**：每帧一个残差块，残差 = `-NCC(T)`；使用 `ceres::NumericDiffCostFunction<EdgeNCCCost, ceres::CENTRAL, 1, 6>`，无需手写雅可比。
- **流程**：收集最多 30 帧有效 (scan, image) 对 → 初值评估 → 若有效帧 ≥3 则构建 Ceres Problem → Solve → 用优化后的 SE3 再算一次 NCC 作为最终指标与收敛判断。

### 2.2 代码位置

- `calib_unified/src/extrinsic/lidar_camera_calib.cpp`：
  - `lidar_to_intensity_image_static()`：静态强度图生成（供 Ceres 代价用）。
  - `compute_frame_ncc()`：单帧 NCC。
  - `EdgeNCCCost`：Ceres 代价，内部调用 `compute_frame_ncc`。
  - `calibrate_edge_align()`：组帧、初值、Ceres 优化、结果写入 `ExtrinsicSE3`。

### 2.3 配置

- `lidar_camera.edge_canny_low/high`、`ceres_max_iter` 等已存在于配置；精标定阶段会使用 `cfg_.ceres_max_iter`（默认 50）。

---

## 3. V2：多相机 / 质量门 / 批量

- **多相机**：已支持。通过 `lidar_camera.pairs` 或 `lidar_camera_camera_list` 配置多相机，精标定对每个相机分别跑边缘对齐并写各自外参。
- **质量门**：Pipeline 中 `lidar_cam_rms_threshold`、`StageResult::needs_manual_refine()` 与 report 中的 `needs_manual_refine` 已存在；RMS 超过阈值会在汇总中标记并建议手动校准。
- **批量/回放**：可通过多次执行 `./calib_unified_run.sh --run --task lidar-cam --config ... --dataset <名>` 或不同 `--data-dir` 实现多数据集跑批；观测性依赖现有日志与 `pipeline_report_<ts>.yaml`。

---

## 4. 编译与运行（Docker）

- 编译与运行均在 Docker 内完成，例如：
  - `./calib_unified_run.sh`：编译 + 自动化验证
  - `./calib_unified_run.sh --run --task lidar-cam`：默认会启用粗标定（lidar-cam 任务默认 `DO_COARSE=true`）
- 数据：ROS2 bag 时配置 `ros2.ros2_bag_file`（相对 `CALIB_DATA_DIR` 或 `--data-dir`）；文件模式时配置 `data.lidar.<id>` 与 `data.camera.<id>.images_dir`。
- 模型路径：内置脚本为 `calib_unified/scripts/run_lidar_cam_coarse.py`（几何恒等/PnP）；真实 Overlap Transformer 时需配置 MIAS-LCEC 仓库与 `model/pretrained_overlap_transformer.pth.tar`。

---

## 5. 验证建议

1. **粗标定**：`./calib_unified_run.sh --run --task lidar-cam`，日志中应出现「粗标定成功」或「使用粗标定初值」及 t=[...]m；若 Python 曾报 numpy.dtype，应已回退 ASCII 且无崩溃。
2. **精标定 Ceres**：同一运行中应看到「Ceres: N iterations」「边缘对齐 优化后 NCC=...」「converged=...」。
3. **联合任务**：`./calib_unified_run.sh --run --task joint --coarse` 时，LiDAR-Cam 粗标定由 Pipeline 内 `run_coarse_stage(LIDAR_CAM_EXTRIN)` 执行，精标定使用 `coarse_lidar_cam_init_`。
