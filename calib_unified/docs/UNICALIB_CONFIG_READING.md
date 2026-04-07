# unicalib_example.yaml 配置读取核对说明

## 0. Executive Summary

- **结论**：已从系统层、模块层、代码层核对并修正 `unicalib_example.yaml` 的读取逻辑，统一格式（`sensors` + `reference_imu` + `data`）由 `YamlIO::load_system_config()` 支持，各子段落参数由 joint_calib main 中的 `fill_solver_config_from_yaml()` 正确映射到各 Calibrator Config。
- **收益**：使用 `config/unicalib_example.yaml` 时，传感器列表、任务开关、内外参标定参数、数据路径均可被正确读取；旧格式（`imu_sensors`/`lidar_sensors`/`camera_sensors`）保持兼容。
- **风险与回滚**：修改集中在 `yaml_io.cpp`、`sensor_types`、`apps/joint_calib/main.cpp`；若需回滚，可还原上述文件并保留 `SystemConfig` 新增的 data 路径 map（或一并还原）。

---

## 1. 系统层：配置入口与数据流

```
unicalib_example.yaml
        │
        ├── 统一入口: YamlIO::load_system_config(yaml_path)
        │   └── 产出: SystemConfig (sensors, reference_imu, output_dir, bag_file, data paths)
        │
        └── joint_calib main: YAML::LoadFile() + fill_solver_config_from_yaml(cfg, solver_cfg)
            └── 产出: 任务掩码(tasks)、PipelineConfig、JointCalibSolver::Config
```

- **joint_calib**：先 `YAML::LoadFile(config_file)`，再调用 `YamlIO::load_system_config(config_file)` 得到 `SystemConfig`（含传感器与 data 路径），并用 `fill_solver_config_from_yaml(cfg, solver_cfg)` 填充求解器配置；`output_dir`、`do_*`、`verbose` 以 YAML 为准（存在时覆盖命令行）。

---

## 2. 模块层：谁读哪些段落

| 配置段落 | 读取位置 | 产出/用途 |
|----------|----------|-----------|
| 顶层 `verbose`, `output_dir` | main + load_system_config | 日志级别、输出目录 |
| `do_*` 任务开关 | main | CalibTaskType 掩码 |
| `sensors` | YamlIO::load_system_config | SystemConfig.sensors (SensorDesc) |
| `reference_imu` | YamlIO::load_system_config | SystemConfig.reference_imu |
| `data` | YamlIO::load_system_config | bag_file, imu_data_paths, lidar_data_paths, camera_images_dirs, camera_intrinsic_yamls |
| `imu_intrinsic` | fill_solver_config_from_yaml | JointCalibSolver::Config.imu_intrin_cfg |
| `camera_intrinsic` | fill_solver_config_from_yaml | JointCalibSolver::Config.cam_intrin_cfg |
| `imu_lidar` | fill_solver_config_from_yaml | JointCalibSolver::Config.imu_lidar_cfg |
| `lidar_camera` | fill_solver_config_from_yaml | JointCalibSolver::Config.lidar_cam_cfg |
| `cam_cam` | fill_solver_config_from_yaml | JointCalibSolver::Config.cam_cam_cfg |
| `joint_bspline` | fill_solver_config_from_yaml | 写入 imu_lidar_cfg (spline/ceres) 与 cam_cam_cfg.ba_optimize_intrinsics |

---

## 3. 代码层：每个 YAML 参数与读取位置

### 3.1 顶层

| YAML 键 | 类型 | 读取位置 | 说明 |
|---------|------|-----------|------|
| `do_imu_intrinsic` | bool | main | 覆盖 do_imu_intrin，参与 tasks |
| `do_camera_intrinsic` | bool | main | 同上 |
| `do_imu_lidar_extrinsic` | bool | main | 同上 |
| `do_lidar_camera_extrinsic` | bool | main | 同上 |
| `do_cam_cam_extrinsic` | bool | main | 同上 |
| `do_joint_bspline_refine` | bool | fill_solver_config_from_yaml → solver_cfg.do_joint_bspline_refine |
| `verbose` | bool | main | 为 true 时 log_level 置为 debug |
| `output_dir` | string | load_system_config + main | SystemConfig.output_dir，main 再覆盖 output_dir 变量 |

### 3.2 sensors[]（统一格式）

| 键 | 位置 | 说明 |
|----|------|------|
| `id` | load_system_config | SensorDesc.sensor_id |
| `type` | load_system_config | imu/lidar/camera → sensor_type_from_str |
| `topic` | load_system_config | SensorDesc.topic |
| `imu.rate_hz` | load_system_config | IMUParams.rate_hz |
| `imu.model` | load_system_config | bias_only/scale_only/scale_misalignment → imu_model_from_str |
| `lidar.type` | load_system_config | spinning/solid_state → lidar_type_from_str |
| `lidar.scan_lines` | load_system_config | LiDARParams.scan_lines |
| `lidar.rate_hz` | load_system_config | LiDARParams.rate_hz |
| `camera.model` | load_system_config | pinhole/fisheye → camera_model_from_str |
| `camera.width` / `height` | load_system_config | CameraParams |
| `camera.fps` | load_system_config | CameraParams.fps |
| `camera.rolling_shutter` | load_system_config | CameraParams.is_rolling_shutter |

### 3.3 reference_imu、data

| YAML 键 | 读取位置 | 说明 |
|---------|----------|------|
| `reference_imu` | load_system_config | SystemConfig.reference_imu |
| `data.bag_file` | load_system_config | SystemConfig.bag_file |
| `data.imu.<id>` | load_system_config | SystemConfig.imu_data_paths[id] |
| `data.lidar.<id>` | load_system_config | SystemConfig.lidar_data_paths[id] |
| `data.camera.<id>.images_dir` | load_system_config | SystemConfig.camera_images_dirs[id] |
| `data.camera.<id>.intrinsic_yaml` | load_system_config | SystemConfig.camera_intrinsic_yamls[id] |

### 3.4 imu_intrinsic

| YAML 键 | Config 成员 | 默认 |
|---------|-------------|------|
| `static_gyro_thresh` | imu_intrin_cfg.static_gyro_threshold | 0.05 |
| `static_detect_window` | imu_intrin_cfg.static_detect_window | 0.5 |
| `min_static_frames` | imu_intrin_cfg.min_static_frames | 50 |
| `allan_num_tau_points` | imu_intrin_cfg.allan_num_tau_points | 50 |

### 3.5 camera_intrinsic

| YAML 键 | Config 成员 | 默认 |
|---------|-------------|------|
| `min_images` | cam_intrin_cfg.min_images | 15 |
| `max_images` | cam_intrin_cfg.max_images | 100 |
| `max_rms_px` | cam_intrin_cfg.max_rms_px | 1.5 |
| `target.type` | cam_intrin_cfg.target.type | chessboard/circles/asym_circles |
| `target.cols` / `rows` | target.cols, target.rows | 9, 6 |
| `target.square_size` | target.square_size_m | 0.025 |
| `model` | cam_intrin_cfg.model | pinhole/fisheye |

### 3.6 imu_lidar

| YAML 键 | Config 成员 | 默认 |
|---------|-------------|------|
| `ndt_resolution` | imu_lidar_cfg.ndt_resolution | 1.0 |
| `ndt_max_iter` | imu_lidar_cfg.ndt_max_iter | 30 |
| `spline_dt_s` | imu_lidar_cfg.spline_dt_s | 0.1 |
| `spline_order` | imu_lidar_cfg.spline_order | 4 |
| `optimize_time_offset` | imu_lidar_cfg.optimize_time_offset | true |
| `time_offset_init` | imu_lidar_cfg.time_offset_init_s | 0.0 |
| `time_offset_max` | imu_lidar_cfg.time_offset_max_s | 0.2 |
| `min_motion_rot_deg` | imu_lidar_cfg.min_motion_rot_deg | 3.0 |

### 3.7 lidar_camera

| YAML 键 | Config 成员 | 默认 |
|---------|-------------|------|
| `method` | lidar_cam_cfg.method | target/edge/motion → TARGET_CHESSBOARD/EDGE_ALIGNMENT/MOTION_BSPLINE |
| `board_cols` / `board_rows` | board_cols, board_rows | 9, 6 |
| `square_size` | square_size_m | 0.025 |
| `optimize_time_offset` | optimize_time_offset | true |

### 3.8 cam_cam

| YAML 键 | Config 成员 | 默认 |
|---------|-------------|------|
| `method` | cam_cam_cfg.method | chessboard/essential/ba |
| `fix_intrinsics` | cam_cam_cfg.ba_optimize_intrinsics = !fix_intrinsics | true |
| `max_rms_px` | cam_cam_cfg.max_rms_px | 2.0 |
| `board_cols`/`board_rows`/`square_size` | cam_cam_cfg.target | 9, 6, 0.025 |

### 3.9 joint_bspline

| YAML 键 | Config 成员 | 默认 |
|---------|-------------|------|
| `spline_order` | imu_lidar_cfg.spline_order | 4 |
| `spline_dt_s` | imu_lidar_cfg.spline_dt_s | 0.05 |
| `max_iterations` | imu_lidar_cfg.ceres_max_iter | 50 |
| `optimize_gravity` | imu_lidar_cfg.optimize_gravity | true |
| `optimize_intrinsics` | cam_cam_cfg.ba_optimize_intrinsics | false |

---

## 4. 变更清单（文件级）

| 文件 | 变更说明 |
|------|----------|
| `include/unicalib/common/sensor_types.h` | 声明 `lidar_type_from_str`, `camera_model_from_str`, `imu_model_to_str`, `imu_model_from_str`；SystemConfig 增加 `imu_data_paths`, `lidar_data_paths`, `camera_images_dirs`, `camera_intrinsic_yamls` |
| `src/common/sensor_types.cpp` | 实现上述 from_str/to_str |
| `src/io/yaml_io.cpp` | 支持统一格式 `sensors` 及嵌套 imu/lidar/camera；读取 reference_imu, output_dir, data.*（含 bag_file 与各 data 路径 map） |
| `apps/joint_calib/main.cpp` | 调用 load_system_config；YAML 存在 do_* 时用其覆盖任务掩码；verbose 控制 log_level；新增 fill_solver_config_from_yaml 并调用，填充 JointCalibSolver::Config |

---

## 5. 编译与验证

- 编译：在工程根目录使用现有 CMake/Make 流程编译 `calib_unified` 及 `unicalib_joint`。
- 验证建议：
  - 使用 `config/unicalib_example.yaml` 运行：  
    `unicalib_joint --config calib_unified/config/unicalib_example.yaml --all`  
    检查日志中传感器数量、output_dir、reference_imu 与各标定参数是否与 YAML 一致。
  - 将 `sensors` 改为旧格式 `imu_sensors`/`lidar_sensors`/`camera_sensors` 再跑一次，确认仍能正确加载传感器列表。

---

## 6. 后续可选

- 将 `solver_cfg` 传入 CalibPipeline / JointCalibSolver，使精标定阶段真正使用 YAML 中的 imu_intrinsic、camera_intrinsic、imu_lidar 等参数。
- 使用 SystemConfig 中的 `imu_data_paths`、`lidar_data_paths`、`camera_images_dirs` 在流水线内自动解析数据路径，减少各 app 单独配置。
