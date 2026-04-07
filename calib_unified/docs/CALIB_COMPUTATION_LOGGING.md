# 标定计算环节日志说明

## 目的

为每个**计算环节**增加统一前缀 `[CALC]` 的日志，便于标定异常时快速定位阶段、数值与收敛情况，支持高效分析与修复。

## 如何使用

- **过滤计算日志**：`grep '[CALC]' <log_file>` 或 `grep "\[CALC\]" run_*.log`
- **按模块过滤**：`grep '\[CALC\].*LiDAR-Cam' run_*.log`、`grep '\[CALC\].*Phase3' run_*.log`
- 日志级别与现有 `UNICALIB_INFO` 一致，由 `log_level` / `--log-level` 控制；`[CALC]` 使用 `info` 级别。

## 日志宏

- `UNICALIB_CALC(msg, ...)` — 计算环节信息（info）
- `UNICALIB_CALC_DEBUG(msg, ...)` — 计算环节调试（debug）
- `UNICALIB_CALC_WARN(msg, ...)` — 计算环节警告（warn）

定义见 `include/unicalib/common/logger.h`。

## 各模块 [CALC] 记录点

| 模块 | 记录内容 |
|------|----------|
| **Pipeline** | 粗/精标定阶段开始(task)、阶段结束(success, residual_rms, elapsed_ms, threshold) |
| **JointCalibSolver** | 联合标定入口/出口；Phase1 内参(RMS/收敛)；Phase2 粗外参起止；Phase3 B样条 Ceres(initial_cost, final_cost, iter, converged)；Phase4 验证开始 |
| **Cam-Cam** | 本质矩阵内点数/内点比；立体标定 stereoCalibrate rms；两视图 BA Ceres(n_valid, initial/final_cost, iter, converged)、BA 完成 residual_rms；**多视图 BA** Ceres(initial/final_cost, iter, 观测数, converged) |
| **LiDAR-Camera** | **目标法仅平移精化** Ceres(initial/final_cost, iter, converged)；目标法 BA Ceres、BA 完成(帧数, 点对数, reproj_rms)；边缘法 Ceres、优化结果(NCC, rms, converged) |
| **LiDAR-LiDAR** | NDT/GICP 完成(fitness_score, converged)；FPFH+RANSAC(内点数, 总对应, 内点比)；配准评估(inliers, overlap_ratio, inlier_rmse, converged)；B样条精化(占位说明) |
| **IMU-LiDAR** | 手眼旋转(平均残差 deg, 旋转对数)；平移估计(t, rms_m_s, 约束数)；B样条 Ceres(initial/final_cost, iter, time_offset_s, converged) |
| **手动标定** | LiDAR-Cam 点击优化 Ceres(initial/final_cost, iter, converged)、完成(点数, residual_rms)；Cam-Cam 对极优化 Ceres(initial/final_cost, iter, 点数, converged)、完成(residual_rms) |
| **相机内参** | 针孔 calibrateCamera(帧数, rms)；针孔剔除离群后(保留帧数, rms)；鱼眼 fisheye::calibrate(帧数, rms)；立体内参 stereoCalibrate(帧数, rms, converged) |
| **IMU 内参** | Allan 方差标定(帧数, allan_fit_rms, gyro_noise, accel_noise)；六面法(静态片段数, rms_residual, gravity)；(legacy 实现同义 CALC) |

## 排查建议

1. **收敛/精度问题**：先看 `[CALC]` 中 `converged`、`final_cost`、`residual_rms`/`rms`，判断是哪个阶段变差。
2. **粗标定失败**：看对应模块的粗阶段 `[CALC]`（如本质矩阵内点比、NDT/GICP fitness、手眼残差）。
3. **精标定不达标**：看 BA/边缘法/B样条的 `initial_cost` vs `final_cost` 和迭代数，判断是否收敛或初值过差。
4. **联合标定**：按 Phase1→Phase2→Phase3→Phase4 顺序看各 `[CALC]`，结合 `residual_rms`/`quality_threshold` 判断是否需手动校准或重新采集。

## 编译与运行

无额外依赖；与现有日志系统(spdlog)一致。构建后直接运行标定，将 `log_level` 设为 `info`（默认）即可在控制台/日志文件中看到 `[CALC]` 行。
