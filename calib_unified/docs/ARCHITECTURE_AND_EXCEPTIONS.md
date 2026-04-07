# UniCalib 架构分析与异常处理设计

## 0. Executive Summary

- **架构**：应用层 (apps) → 统一标定库 (libunicalib) → iKalibr/第三方；统一配置与 Status/ErrorCode 异常体系已就绪。
- **收益**：在关键路径（IO、求解器、可视化、保存）增加异常处理与上下文信息，便于精确定位配置/数据/数值/系统类问题。
- **风险**：部分逻辑原依赖 map 错误迭代（Phase3），已修正为按「传感器 ID → 帧序列」正确遍历。

---

## 1. 整体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│ 应用层 (apps/)                                                           │
│   imu_intrinsic | camera_intrinsic | imu_lidar | lidar_camera |         │
│   cam_cam_extrin | joint_calib (CalibPipeline)                          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ 统一标定库 (libunicalib)                                                  │
│   • JointCalibSolver (Orchestrator)  • IMU/Camera 内参标定               │
│   • IMU-LiDAR / LiDAR-Cam / Cam-Cam 外参  • Phase3 B样条精化 (可选)      │
│   • YamlIO / CalibParamManager       • CalibVisualizer / ReportGenerator │
│   • CalibPipeline / AICoarseCalib / ManualCalib                          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
          ┌─────────────────────────┼─────────────────────────┐
          ▼                         ▼                         ▼
┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│ iKalibr 核心      │    │ 第三方            │    │ 工程化基础设施     │
│ (B样条/因子/Ceres)│    │ OpenCV/PCL/Ceres  │    │ exception/status/  │
│ (UNICALIB_WITH_   │    │ Sophus/basalt     │    │ logger/error_code  │
│  IKALIBR=ON)      │    │ yaml-cpp/spdlog   │    │                    │
└──────────────────┘    └──────────────────┘    └──────────────────┘
```

**数据流**：  
配置文件 (unicalib_example.yaml) → SystemConfig → CalibDataBundle (imu_data / lidar_scans / camera_frames) → Phase1 内参 → Phase2 粗外参 → Phase3 B样条精化 → Phase4 验证 → save_results (YAML/JSON/报告)。

---

## 2. 模块职责与依赖

| 模块 | 职责 | 关键接口 | 异常/状态 |
|------|------|----------|-----------|
| **common/** | 错误码、异常类、Status、Logger、CalibParamManager、SensorTypes | ErrorCode, UniCalibException, Status, UNICALIB_THROW_* | 已统一 |
| **io/yaml_io** | 系统配置/IMU CSV/相机内参 读写 | load_system_config, load_imu_csv, load/save_camera_intrinsics | 已用 UNICALIB_THROW_DATA，可加强上下文 |
| **solver/joint_calib_solver** | 联合标定编排、Phase1–4、B样条精化 | calibrate(), phase1_intrinsics, phase2_coarse_extrinsic, phase3_joint_refine, save_results | 需入口校验、Phase 内上下文、Phase3 迭代修正 |
| **intrinsic/** | IMU Allan/六面法、相机针孔/鱼眼 | IMUIntrinsicCalibrator, CameraIntrinsicCalibrator | 可返回 optional，由 solver 转异常 |
| **extrinsic/** | IMU-LiDAR / LiDAR-Cam / Cam-Cam | IMULiDARCalibrator, LiDARCameraCalibrator, CamCamCalibrator | 同上 |
| **viz/calib_visualizer** | 3D 轨迹、收敛曲线、投影、报告 | show_imu_lidar_result, show_lidar_camera_projection, save_plots, generate_report | 需空指针/空容器检查与 VISUALIZATION_ERROR |
| **pipeline/** | 流水线编排、AI 粗标定、手动校准 | CalibPipeline::run(), AICoarseCalibManager, ManualCalibSession | 依赖 solver/io 异常，main 需捕获并打印 |
| **common/calib_param** | 外参/内参存储、YAML 序列化 | save_yaml, load_yaml | 已有 FILE_WRITE_ERROR，可加 try/catch 与路径上下文 |

---

## 3. 异常处理设计原则

1. **谁抛谁带上下文**：抛出时带上模块名、操作、关键参数（路径、sensor_id、帧数、阶段）。
2. **错误码分类**：1xxx 配置、2xxx 数据/IO、3xxx 标定、4xxx 数值、5xxx 系统、6xxx 可视化。
3. **可定位**：UniCalibException 含 file/line/func、timestamp、withContext/withTag。
4. **降级策略**：Phase3 失败时保留 Phase2 结果；保存结果失败时记录日志并写 summary.error_message。

---

## 4. 变更清单（异常与校验增强）

| 文件 | 变更要点 |
|------|----------|
| `src/solver/joint_calib_solver.cpp` | calibrate() 入口校验 CalibDataBundle；Phase1/2 空数据与缺失内参时带 sensor_id 抛出；Phase3 修正 lidar_scans/imu_data 迭代为「map→vector」并加 try/catch 与阶段信息；save_results 含路径 try/catch。 |
| `src/io/yaml_io.cpp` | 解析后校验（如传感器数量）、关键 key 缺失时带 key 名抛出。 |
| `src/viz/calib_visualizer.cpp` | 空 viewer/空容器/无效路径用 UNICALIB_THROW 或 VisualizationException 带上下文。 |
| `src/common/calib_param.cpp` | save_yaml/load_yaml 外层 try/catch，消息带 path 与 section。 |
| `apps/joint_calib/main.cpp` | pipeline.run() 外捕获 UniCalibException 与 std::exception，打印 e.toString()/e.what()，返回 1。 |

---

## 5. 验证与回滚

- **验证**：运行 `./calib_unified_run.sh --test-only` 与单任务 `--run --task imu-intrin`（及 lidar-cam 等），确认无回归；故意删配置/错路径，确认错误信息含文件与上下文。
- **回滚**：本次仅增加异常分支与少量逻辑修正（Phase3 迭代）；若问题可逐文件 revert。

---

## 6. 术语表 (Glossary)

- **CalibDataBundle**：按 sensor_id 组织的 IMU 帧、LiDAR 扫描、相机帧/图像路径的输入数据包。
- **Phase1–4**：内参 → 粗外参 → B样条精化 → 验证。
- **UNICALIB_THROW_DATA / UNICALIB_THROW_CALIB**：带 ErrorCode 与源码位置的 DataException/CalibException 宏。
- **Status / summary.toStatus()**：非异常路径的轻量返回，携带 ErrorCode 与 message。
