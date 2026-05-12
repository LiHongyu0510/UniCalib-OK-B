## 0. 先建立全局图

### 要回答的 4 个问题

1. 用户命令如何映射到具体可执行程序？
2. 一次任务运行会经历哪些阶段（coarse/fine/manual）？
3. 配置从 YAML 如何进入运行时？
4. 结果和日志最终写到哪里？

### 必读文件（顺序）

1. `calib_unified_run.sh`
2. `calib_unified/apps/joint_calib/main.cpp`
3. `calib_unified/src/pipeline/calib_pipeline.cpp`
4. `calib_unified/config/unicalib_example.yaml`
5. `calib_unified/docs/UNICALIB_CONFIG_READING.md`

### 读完后的“通过标准”

- 你能画出命令到二进制的映射关系（task -> unicalib_*）。
- 你能解释 pipeline 的三阶段触发条件与降级逻辑。
- 你知道如何从日志定位失败阶段。

---

## 1. 配置系统专项

### 学习目标

- 看懂必须项、默认值、回退路径。
- 区分 `unicalib` 配置链与 `ikalibr::Configor` 配置链。

### 必读文件（顺序）

1. `calib_unified/config/unicalib_example.yaml`
2. `calib_unified/src/io/yaml_io.cpp`
3. `calib_unified/include/ikalibr/config/configor.h`
4. `calib_unified/src/ikalibr/config/configor.cpp`

### 阅读时重点检查

- `pairs` 与 `sensors.id`、ROS2 topic 的一致性。
- `data.*` 相对路径与 `--data-dir`/`CALIB_DATA_DIR` 的拼接关系。
- 默认值宏（例如 `YAML_GET_OR`）是否导致“静默兜底”。

### 常见坑位

- 把 iKalibr 的键写进 unified YAML 但实际不生效。
- 多传感器场景依赖回退逻辑，未显式配置 `pairs`。

---

## 2. 按任务阅读

推荐顺序：`lidar-cam` -> `imu-lidar` -> `cam-cam` -> `lidar-lidar` -> `joint`。

### A. LiDAR-Camera（最建议先读）

#### 必读文件

1. `calib_unified/apps/lidar_camera_extrin/main.cpp`
2. `calib_unified/src/pipeline/calib_pipeline.cpp`
3. `calib_unified/src/pipeline/manual_calib.cpp`
4. `calib_unified/src/io/ros2_data_source.cpp`
5. `calib_unified/docs/LIDAR_CAM_EXTRIN_CODE_ANALYSIS.md`

#### 你要搞清楚

- coarse 何时启用，何时跳过。
- manual 模式触发条件与保存产物（manual_extrinsic_*）。
- ROS2 与文件模式的输入差异对精标定的影响。

---

### B. IMU-LiDAR

#### 必读文件

1. `calib_unified/apps/imu_lidar_extrin/main.cpp`
2. `calib_unified/src/pipeline/calib_pipeline.cpp`
3. `calib_unified/src/io/ros2_data_source.cpp`
4. `calib_unified/docs/ROS2_INTEGRATION.md`
5. `calib_unified/config/unicalib_example.yaml`（imu_lidar 段）

#### 你要搞清楚

- 时间重叠窗口如何裁剪（IMU-LiDAR 对齐前提）。
- `optimize_time_offset` 之类开关何时有收益。
- planar/运动先验对收敛稳定性的帮助与风险。

---

### C. Cam-Cam

#### 必读文件

1. `calib_unified/apps/cam_cam_extrin/main.cpp`
2. `calib_unified/src/pipeline/calib_pipeline.cpp`
3. `calib_unified/docs/CAM_CAM_EXTRINSIC_ALGORITHM_AND_OPTIMIZATION.md`
4. `calib_unified/docs/CAM_CAM_CALIB_LOGIC_ACCURACY_EXCEPTIONS.md`
5. `calib_unified/docs/CAM_CAM_MANUAL_6DOF_GUIDE_CN.md`

#### 你要搞清楚

- `fix_intrinsics` 与外参估计的耦合关系。
- 手动微调和自动优化的边界。
- RMS 阈值与回退策略如何设置更稳。

---

### D. LiDAR-LiDAR

#### 必读文件

1. `calib_unified/apps/lidar_lidar_extrin/main.cpp`
2. `calib_unified/docs/LIDAR_LIDAR_EXTRINSIC_ALGORITHM_AND_OPTIMIZATION.md`
3. `calib_unified/docs/LIDAR_LIDAR_CALIB_COMPUTATION_AND_ACCURACY.md`
4. `calib_unified/config/unicalib_example.yaml`（lidar_lidar 段）

#### 你要搞清楚

- 两阶段（粗+精）参数如何影响鲁棒性与速度。
- 初值不佳时，失败最常发生在哪一步。

---

### E. Joint（最后读）

#### 必读文件

1. `calib_unified/apps/joint_calib/main.cpp`
2. `calib_unified/src/pipeline/calib_pipeline.cpp`
3. `calib_unified/src/solver/joint_calib_solver.cpp`
4. `calib_unified/config/unicalib_example.yaml`（joint_bspline 段）

#### 你要搞清楚

- 哪些任务参数被完整传递到 solver，哪些仍是部分接线。
- 多任务并行/串行执行对结果一致性的影响。

---

## 3. 算法内核阅读

### 学习目标

- 明确状态变量、因子、求解器配置。
- 能从 residual 角度解释“为什么这个数据会失败”。

### 必读文件（顺序）

1. `calib_unified/src/ikalibr/calib/estimator.cpp`
2. `calib_unified/include/ikalibr/factor/`（按 imu/lidar/visual 先后读）
3. `calib_unified/src/ikalibr/solver/calib_solver_proc_impl.cpp`
4. `calib_unified/src/ikalibr/solver/calib_solver_bo_impl.cpp`
5. `calib_unified/src/ikalibr/calib/spat_temp_priori.cpp`

### 核心检查点

- 当前阶段“哪些参数在优化、哪些被固定”。
- 鲁棒核与权重如何作用在不同观测。
- 时延与外参是否同时可观测，何时退化。

---

## 4. 实战验证清单（建议每读完一个任务就执行）

以下命令用来验证你对链路的理解，不要求一次全跑完。

```bash
# 1) 看入口能力与任务列表
./calib_unified_run.sh --task-help

# 2) 单任务最小闭环（示例：lidar-cam）
CALIB_DATA_DIR=/path/to/data ./calib_unified_run.sh --run --task lidar-cam --manual

# 3) 看日志与结果是否落在预期目录
ls logs
ls results
```

验证时请记录：

- 失败发生在 coarse/fine/manual 哪一阶段；
- 使用的是 ROS2 还是文件模式；
- 对应 YAML 关键项（pairs/topic/path）是否全显式填写。

---

## 5. 一页速记：问题定位顺序

1. 先看 `calib_unified_run.sh` 的参数展开是否正确。
2. 再看 `pipeline` 判定当前执行阶段与分支。
3. 再看数据源加载是否拿到正确传感器与时间范围。
4. 最后下钻 `estimator/factor` 看可观测性与残差行为。

---

## 6. 后续可迭代

如果后续要做团队知识沉淀，建议在本文件基础上新增：

- “每任务最小可用 YAML 模板（必须项/可选项）”
- “常见报错 -> 原因 -> 修复动作”速查表
- “按数据质量分级的参数推荐值（保守/标准/激进）”

