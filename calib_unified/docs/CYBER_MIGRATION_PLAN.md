# UniCalib CyberRT 化工程方案

> 基于现有离线标定工具，升级为模块化、实时化、可部署的 CyberRT 标定平台。

---

## 目录

1. [当前系统现状](#一当前系统现状)
2. [Cyber 化目标](#二cyber-化目标)
3. [系统总体架构](#三系统总体架构)
4. [模块拆分方案](#四模块拆分方案)
5. [同步层设计](#五同步层设计)
6. [标定核心层](#六标定核心层)
7. [可视化系统](#七可视化系统)
8. [配置管理系统](#八配置管理系统)
9. [Recorder 系统](#九recorder-系统)
10. [在线标定系统](#十在线标定系统重点)
11. [Docker 部署方案](#十一docker-部署方案)
12. [域控部署方案](#十二域控部署方案)
13. [通信设计](#十三通信设计)
14. [调度与线程模型](#十四调度与线程模型)
15. [目录结构建议](#十五目录结构建议)
16. [与现有系统的衔接](#十六与现有系统的衔接)
17. [开发阶段规划](#十七开发阶段规划)
18. [未来扩展方向](#十八未来扩展方向)

---

## 一、当前系统现状

### 现有 7 个独立应用

| 应用 | 入口 | 功能 |
|------|------|------|
| unicalib_imu_intrinsic | `apps/imu_intrinsic/main.cpp` | IMU 内参（Allan 方差） |
| unicalib_camera_intrinsic | `apps/camera_intrinsic/main.cpp` | 相机内参（棋盘格 + DM-Calib） |
| unicalib_imu_lidar | `apps/imu_lidar_extrin/main.cpp` | IMU-LiDAR 外参（B-样条） |
| unicalib_lidar_camera | `apps/lidar_camera_extrin/main.cpp` | LiDAR-Camera 外参（边缘对齐 + 互信息） |
| unicalib_lidar_lidar | `apps/lidar_lidar_extrin/main.cpp` | LiDAR-LiDAR 外参（NDT + FPFH） |
| unicalib_cam_cam | `apps/cam_cam_extrin/main.cpp` | Camera-Camera 外参（BA + 特征匹配） |
| unicalib_joint | `apps/joint_calib/main.cpp` | 联合标定（全传感器） |

### 体系结构

```
每个 app/main.cpp:
 ┌──────────────────────────────┐
 │  YAML 配置解析                │
 │  PipelineConfig 填充          │
 ├──────────────────────────────┤
 │  数据源加载 (三选一)           │
 │  ├── 文件模式 (PCD + 图像)    │
 │  ├── ROS2 Bag / 话题          │
 │  └── NEW_FORMAT (CSV 索引)    │
 ├──────────────────────────────┤
 │  三阶段流水线 (CalibPipeline) │
 │  ├── Stage 1: AI 粗标定       │
 │  ├── Stage 2: 自动精标定      │
 │  └── Stage 3: 手动精校准      │
 ├──────────────────────────────┤
 │  可视化 (Pangolin / PCL / 无) │
 │  结果输出 YAML                │
 └──────────────────────────────┘
```

### 现有问题

- 7 个独立 main.cpp，代码分散
- 数据加载与标定逻辑耦合，每个 app 重复数据加载代码
- 无实时数据流接口（`feed_*` 方法被 `#if 0` 包裹，未启用）
- 依赖 ROS2 可选，但 CyberRT 完全未接入
- 无法在线检测外参漂移
- 模块不可独立运行，难以部署到域控

### 已有 Cyber 集成资产（待启用）

`calib_unified/cyber/` 目录已包含：

| 文件 | 内容 | 状态 |
|------|------|------|
| `auto_calib_component.h/.cpp` | Cyber Component 主入口 | 完整，被 `#if 0` 包裹 |
| `cyber_data_adapter.h` | Cyber 消息 → 内部 Frame 适配器 | 接口就绪，实现为空 |
| `calib_result.proto` | 标定结果 Protobuf 定义 | 完整 |
| `shm_calibration_channel.h` | 双缓冲共享内存通道（域控零拷贝） | 完整 |
| `pipeline.h` 中 `feed_lidar_frame` / `feed_camera_frame` / `feed_imu_data` | 流式数据接口 | 被 `#if 0` 包裹 |

---

## 二、Cyber 化目标

### 架构升级目标

```
升级前 (离线工具)        升级后 (CyberRT 平台)
┌──────────────┐       ┌──────────────────────────┐
│ main.cpp     │       │  Component 架构            │
│  耦合全部逻辑  │  →   │  Topic 通信                │
│  无实时能力   │       │  实时流式标定               │
│  难部署      │       │  在线监控                  │
└──────────────┘       │  Docker 部署              │
                       │  域控运行                  │
                       └──────────────────────────┘
```

### 具体目标

| 维度 | 当前 | 目标 |
|------|------|------|
| 模块化 | 7 个独立 main.cpp | 7+ 个 Cyber Component |
| 通信 | CLI 参数 + YAML | Cyber Topic + SHM |
| 实时性 | 不支持 | 实时数据流接入 |
| 在线化 | 离线 | 在线漂移检测 + 触发优化 |
| 部署 | 手动运行 | Docker + cyber_launch |
| 扩展 | 加传感器改代码 | 加传感器改 DAG 配置 |

---

## 三、系统总体架构

```
                         ┌──────────────────────────────────────┐
                         │         Config Manager               │
                         │   (sensor.yaml / extrinsic.yaml)     │
                         └──────────┬───────────────────────────┘
                                    │ 动态加载
                                    ▼
┌──────────┐    ┌──────────┐    ┌───────────────────┐    ┌──────────────┐
│ Camera   │───▶│          │    │                   │    │              │
│Component │    │          │    │                   │    │  Result      │
├──────────┤    │  Sync    │───▶│  CalibPipeline    │───▶│  Publisher   │
│ LiDAR    │───▶│  Compo-  │    │  Component        │    │  (Proto+SHM) │
│Component │    │  nent    │    │                   │    │              │
├──────────┤    │          │    │  (三阶段:          │    └──────┬───────┘
│ IMU      │───▶│          │    │   Coarse/Fine/    │           │
│Component │    │          │    │   Manual)         │           ▼
├──────────┤    └──────────┘    └───────────────────┘    ┌──────────────┐
│ OEM7     │                                              │  Monitor    │
│Decoder   │                                              │  Component  │
└──────────┘                                             └──────────────┘

                           ┌──────────────────┐
                           │  Viz Component   │
                           │ (Pangolin/RViz2) │
                           └──────────────────┘
```

### 与现有系统的映射

| Cyber Component | 对应的现有代码 |
|----------------|---------------|
| CameraComponent | `apps/camera_intrinsic/main.cpp` + `io/new_format_data_source.cpp` (Camera 部分) |
| LiDARComponent | `io/lidar_packet_reader.cpp` + `io/new_format_data_source.cpp` (LiDAR 部分) |
| ImuComponent | `io/oem7_imu_reader.cpp` + `io/new_format_data_source.cpp` (IMU 部分) |
| SyncComponent | 当前分散在 `pipeline/calib_pipeline.cpp` 中的时间同步逻辑 |
| CalibPipelineComponent | `pipeline/calib_pipeline.cpp` 完整三阶段 + `solver/` |
| VizComponent | `viz/calib_visualizer.cpp` + `viz/pangolin_render.cpp` |
| ConfigManager | `io/yaml_io.cpp` + `config/unicalib_example.yaml` |
| MonitorComponent | 新增 |

---

## 四、模块拆分方案

### 4.1 Camera Component

```
modules/camera/
├── camera_component.h        # Cyber Component 声明
├── camera_component.cpp      # Init/Proc 实现
├── camera_config.h           # 采集配置
└── proto/
    └── camera_image.proto    # 自定义图像消息（或复用 Cyber 的 Image）
```

**功能**：
- 图像采集（相机驱动封装）
- h265 解码（复用 `sensor_decode/h265_decode/`）
- 时间戳生成
- 图像压缩（可选）

**发布 Topic**：
```
/camera/{id}/image     # Image proto（或 cv::Mat 转 bytes）
/camera/{id}/info      # CameraInfo（内参 + 畸变模型）
```

**现有可复用代码**：
- `sensor_decode/h265_decode/h265_decode.py`（Python h265 解码）
- `src/io/new_format_data_source.cpp`（图像文件加载逻辑）
- `include/unicalib/intrinsic/camera_calib.h`（相机内参模型）

### 4.2 LiDAR Component

```
modules/lidar/
├── lidar_component.h
├── lidar_component.cpp
├── lidar_config.h
└── proto/
    └── point_cloud.proto
```

**功能**：
- LiDAR 原始数据包接收
- 多品牌解码（Livox / Hesai / RoboSense / Velodyne）
- 运动补偿
- 点云发布

**发布 Topic**：
```
/lidar/{id}/points    # PointCloud2（或自定义 proto）
```

**现有可复用代码**：
- `src/io/lidar_packet_reader.cpp`（899 行，纯 C++ 零 ROS 依赖的 LiDAR 解码器）
- `thirdparty/`（各 LiDAR SDK）

### 4.3 IMU Component

```
modules/imu/
├── imu_component.h
├── imu_component.cpp
├── imu_config.h
└── proto/
    └── imu.proto
```

**功能**：
- IMU 数据接收（原始 / 预处理）
- OEM7 二进制解码
- 时间戳对齐

**发布 Topic**：
```
/imu/{id}/data        # Imu proto（角速度 + 加速度 + 时间戳）
```

**现有可复用代码**：
- `src/io/oem7_imu_reader.cpp`（OEM7 CORRIMUDATAS / IMURATECORRIMUS / CORRIMUS 解码）
- `src/intrinsic/allan_variance.cpp`（Allan 方差分析）

### 4.4 OEM7 Decoder Component（新增，从 IMU Component 剥离）

```
modules/oem7/
├── oem7_component.h
├── oem7_component.cpp
└── proto/
    └── oem7.proto
```

**功能**：
- NovAtel OEM7 二进制帧同步 + CRC32 校验
- CORRIMUDATAS 等消息解析
- 可选 GPS 时间 → Unix 时间转换

**发布 Topic**：
```
/oem7/{id}/imu        # IMU 数据（解码后）
/oem7/{id}/raw        # 原始帧（调试用）
```

---

## 五、同步层设计

### Sync Component

```
modules/sync/
├── sync_component.h
├── sync_component.cpp
├── time_buffer.h               # 时间窗口缓存
└── proto/
    └── sync_frame.proto
```

**功能**：
- 接收多个传感器的异步数据
- 按时间窗口进行数据对齐
- 插值 / 时间补偿
- 发布同步后的标定帧

**输入 Topic**：
```
/camera/{id}/image
/lidar/{id}/points
/imu/{id}/data
```

**输出 Topic**：
```
/sync/frame           # SyncFrame（时间对齐后的多传感器帧）
```

**现有可复用代码**：
- `calib_unified/src/pipeline/calib_pipeline.cpp` 中现有的数据加载与时间戳处理逻辑
- `calib_unified/src/io/unified_data_loader.cpp` 的三种数据源统一接口设计模式

**时间同步策略**：

| 同步方式 | 适用场景 | 精度 |
|---------|---------|------|
| 硬触发（HW Sync） | 车载域控 | us 级 |
| 软时间戳对齐 | ROS2 / Cyber | ms 级 |
| B-样条插值 | IMU-LiDAR 联合标定 | us 级 |

---

## 六、标定核心层

### 6.1 CalibPipelineComponent — 核心标定组件

```
modules/calib/
├── pipeline/
│   ├── calib_component.h       # Cyber Component 声明
│   ├── calib_component.cpp     # Init/Proc + 结果发布
│   ├── coarse_component.h      # AI 粗标定（可独立运行）
│   ├── coarse_component.cpp
│   ├── refine_component.h      # 精标定
│   ├── refine_component.cpp
│   ├── manual_component.h      # 手动微调 (GUI)
│   └── manual_component.cpp
├── proto/
│   ├── calib_command.proto     # 标定控制指令
│   └── calib_result.proto      # 标定结果（已定义）
└── config/
    └── calib_params.yaml       # 标定参数
```

**三阶段映射**：

| 阶段 | 对应现有代码 | Cyber Component |
|------|------------|-----------------|
| Coarse (AI 粗标定) | `pipeline/ai_coarse_calib.cpp` | `CoarseComponent` |
| Fine (自动精标定) | `pipeline/calib_pipeline.cpp` (run_fine_*) | `RefineComponent` |
| Manual (手动微调) | `pipeline/manual_calib.cpp` | `ManualComponent` |

**订阅 Topic**：
```
/sync/frame           # 同步后的多传感器帧
/calib/command        # 标定触发指令（start / stop / status）
```

**发布 Topic**：
```
/calib/result         # CalibrationResult proto（完整标定结果）
/calib/status         # 标定进度 / 状态
/calib/error          # 实时重投影误差
```

**现有可复用代码**：
- `pipeline/calib_pipeline.cpp`（核心流水线）
- `pipeline/ai_coarse_calib.cpp`（AI 粗标定桥接）
- `pipeline/manual_calib.cpp`（手动校准）
- `extrinsic/*.cpp`（所有外参标定算法）
- `solver/joint_calib_solver.cpp`（联合优化求解器）

### 6.2 流式接口启用

`calib_pipeline.h:403-408` 中已有的流式接口需启用：

```cpp
// 取消 #if 0，启用流式接口
void feed_lidar_frame(const std::string& lidar_id, const LiDARScan& scan);
void feed_camera_frame(const std::string& cam_id, const Frame& frame);
void feed_imu_data(const ImuData& imu);
std::optional<StageResult> try_auto_calibrate(CalibTaskType tasks);
```

### 6.3 Cyber 数据适配器实现

`cyber/cyber_data_adapter.h` 中三个转换函数需补齐实现：

| 函数 | 输入 → 输出 |
|------|-----------|
| `from_cyber_pointcloud2` | Cyber PointCloud2 → project::PointCloud |
| `from_cyber_image` | Cyber Image → project::Frame (cv::Mat) |
| `from_cyber_imu` | Cyber Imu → project::ImuData |

---

## 七、可视化系统

### Viz Component

```
modules/viz/
├── viz_component.h
├── viz_component.cpp
├── projection_overlay.h        # 点云投影叠加
└── proto/
    └── viz_marker.proto
```

**功能**：
- 实时点云投影到图像
- TF Tree 显示
- 误差曲线（重投影误差、fitness score）
- 标定质量仪表盘

**可视化后端选型**：

| 后端 | 适用场景 | 现有代码 |
|------|---------|---------|
| Pangolin | 离线调试、手动微调 | `viz/pangolin_render.cpp`、`viz/calib_visualizer.cpp` |
| RViz2 | 在线监控（需要 ROS2） | `viz/ros_rviz_visualizer.cpp` |
| Foxglove | Web 远程监控 | 新增（推荐） |
| Cyber RT Visualizer | Apollo 原生 | 新增 |

**订阅 Topic**：
```
/calib/result
/calib/error
/sync/frame
/tf
```

**发布 Topic**：
```
/viz/projection      # 投影图像
/viz/error_curve     # 误差曲线数据
/viz/status          # 标定仪表盘 JSON
```

---

## 八、配置管理系统

### Config Manager

```
modules/config/
├── config_manager.h
├── config_manager.cpp
├── sensor_config.h
├── extrinsic_config.h
└── proto/
    └── config.proto
```

**功能**：
- 统一管理多 YAML 配置（复用现有 `yaml_io.cpp`）
- 动态重加载（信号触发 / 文件监控）
- 参数校验
- 配置热更新通过 Cyber Topic 发布

**配置分层**：

| 层级 | 文件 | 说明 |
|------|------|------|
| 系统级 | `/opt/calib/config/sensors.yaml` | 传感器拓扑、型号、安装位置 |
| 标定级 | `/opt/calib/config/calib_params.yaml` | 标定参数、阈值、方法选择 |
| 运行时 | Cyber Parameter Server | 动态调参 |

**现有可复用代码**：
- `io/yaml_io.cpp`（YAML 配置加载 + 校验）
- `config/unicalib_example.yaml`（完整配置模板）
- `include/unicalib/pipeline/calib_pipeline.h`（PipelineConfig 结构）

---

## 九、Recorder 系统

### Record Component

```
modules/record/
├── record_component.h
├── record_component.cpp
├── bag_writer.h
└── proto/
    └── record_config.proto
```

**功能**：
- 订阅所有原始传感器 Topic
- 分段录包（按时间 / 文件大小）
- 数据回放
- 自动标注（标定触发前后数据标记）

**支持格式**：

| 格式 | 用途 | 实现方式 |
|------|------|---------|
| Cyber Record | 实时回放 | Cyber RT 原生 |
| ROS2 Bag | 兼容现有工具链 | `ros2_data_source.cpp`（已有） |
| NEW_FORMAT CSV | 标定输入 | `new_format_data_source.cpp`（已有） |
| PCD + PNG | 离线标定 | `lidar_packet_reader.cpp` + OpenCV（已有） |

---

## 十、在线标定系统（重点）

### 10.1 在线化目标

在车辆运行过程中实时检测外参漂移，触发局部优化，更新 TF。

### 10.2 工作流

```
实时数据流
    │
    ▼
特征提取 (边缘 / 平面 / 关键点)
    │
    ▼
误差评估 (重投影误差 / fitness score / IMU 积分偏差)
    │
    ▼
漂移检测 (滑动窗口统计)
    │  ├── yaw drift > 0.3°
    │  ├── pitch drift > 0.2°
    │  └── reprojection error > 2.0 px
    │
    ▼
局部优化 (固定其他参数，仅优化漂移外参)
    │
    ▼
更新 TF + 发布新外参
    │
    ▼
共享内存通知下游 (感知 / 规划)
```

### 10.3 关键功能

| 功能 | 描述 | 涉及现有代码 |
|------|------|-------------|
| 外参漂移检测 | 滑动窗口监测重投影误差均值 | `extrinsic/lidar_camera_calib.cpp`（重投影误差计算） |
| 实时误差评估 | 每帧计算 fitness score / reprojection error | `extrinsic/cam_cam_calib.cpp`（特征匹配误差） |
| 局部优化 | 固定其他参数，只优化漂移的外参 | `solver/joint_calib_solver.cpp`（部分参数固定） |
| 自动回滚 | 优化失败时恢复历史外参 | `calib_param.cpp`（外参管理 + 快照） |
| 结果发布 | 通过 SHM + Cyber Topic 通知下游 | `cyber/shm_calibration_channel.h`（已实现） |

### 10.4 现有前置条件

- `calib_pipeline.h` 已有 `feed_lidar_frame` / `feed_camera_frame` / `feed_imu_data` 接口（被 `#if 0` 包裹）
- `calib_pipeline.cpp` 已有完整的 `run_fine_*` 实现，只需将数据源从"文件加载"改为"实时 feed"
- Cyber Component 的 `auto_calib_component.h` 已有 `try_auto_calibrate` 骨架

---

## 十一、Docker 部署方案

### 推荐镜像结构

```
docker/
├── Dockerfile.runtime          # 运行时镜像
├── Dockerfile.dev              # 开发镜像
├── docker-compose.yml          # 多容器编排
├── dag/                        # Cyber DAG 文件
│   ├── calib.dag               # 标定系统 DAG
│   └── online_monitor.dag      # 在线监控 DAG
└── scripts/
    ├── build.sh
    ├── run_calib.sh
    ├── run_online_monitor.sh
    └── entrypoint.sh
```

### Runtime Image

```dockerfile
FROM ubuntu:22.04

# Cyber RT 10.0.0 运行时
COPY cyber /opt/apollo/cyber

# UniCalib 依赖
RUN apt update && apt install -y \
    libeigen3-dev libceres-dev \
    libopencv-dev libpcl-dev \
    libyaml-cpp-dev libspdlog-dev \
    libfmt-dev

# 标定模型（AI 粗标定）
COPY models /opt/calib/models

# UniCalib 运行时
COPY build/bin /opt/calib/bin
COPY config /opt/calib/config
COPY cyber/conf/*.dag /opt/calib/dag/
```

### Dev Image

额外包含：
```
gdb, clangd, valgrind
Pangolin (可视化)
RViz2 (ROS2 可视化桥接)
h265_decode 依赖 (ffmpeg)
```

---

## 十二、域控部署方案

### 推荐平台

| 平台 | 适用场景 | 优势 |
|------|---------|------|
| NVIDIA Orin | 在线标定 + AI 粗标定 | CUDA 加速，AI 模型推理 |
| NVIDIA Thor | 量产 | 更高算力 |
| x86 工控机 | 工厂标定 | 多 LiDAR 处理，Pangolin 可视化 |
| ARM 域控 | 车载部署 | 低功耗 |

### SHM 跨进程通信

使用已有的 `cyber/shm_calibration_channel.h`：

```
┌─────────────────┐          /dev/shm/unicalib_calib        ┌──────────────────┐
│ UniCalib        │  ─────────────────────────────────────  │ 下游进程          │
│ Cyber Component │    SharedExtrinsic[16] + seqlock        │ (感知 / 规划)     │
│ (写者)           │                                         │ (读者)            │
└─────────────────┘                                         └──────────────────┘
```

### 文件系统建议

```
/opt/calib/
├── bin/              # 可执行文件
├── lib/              # 动态库
├── config/           # 配置文件
│   ├── sensors.yaml
│   ├── extrinsic.yaml
│   └── calib_params.yaml
├── models/           # AI 标定模型
│   ├── dm-calib/
│   ├── mias-lcec/
│   └── l2calib/
└── dag/              # Cyber DAG 文件

/data/
├── record/           # 录制数据
├── log/              # 标定日志
└── results/          # 标定结果
```

---

## 十三、通信设计

### 13.1 Topic 定义

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/camera/{id}/image` | `apollo::cyber::proto::Image` | Sensor → Sync | RAW 图像 |
| `/camera/{id}/info` | `CameraInfo` | Sensor → Sync | 内参 + 畸变 |
| `/lidar/{id}/points` | `apollo::cyber::proto::PointCloud2` | Sensor → Sync | 点云 |
| `/imu/{id}/data` | `apollo::cyber::proto::Imu` | Sensor → Sync | 6 轴 IMU |
| `/oem7/{id}/imu` | `Imu` | OEM7 → IMU | 解码后的 IMU |
| `/sync/frame` | `SyncFrame` | Sync → Calib | 多传感器同步帧 |
| `/calib/command` | `CalibCommand` | 外部 → Calib | 标定触发指令 |
| `/calib/result` | `CalibrationResult` | Calib → 外部 | 标定结果 |
| `/calib/status` | `CalibStatus` | Calib → 外部 | 进度 / 状态 |
| `/calib/error` | `CalibError` | Calib → Viz | 实时误差 |
| `/viz/projection` | `Image` | Viz → 外部 | 投影图像 |
| `/config/update` | `ConfigUpdate` | Config → 全局 | 配置热更新 |

### 13.2 Protobuf 消息

**已有**（`cyber/calib_result.proto`）：
- `ExtrinsicSE3` — 单个外参
- `CameraIntrinsic` — 相机内参
- `ImuIntrinsic` — IMU 内参
- `CalibrationResult` — 完整标定结果

**需新增**：
- `SyncFrame` — 同步后的多传感器帧
- `CalibCommand` — 标定控制指令（START / STOP / STATUS / TRIGGER_COARSE）
- `CalibStatus` — 标定状态（IDLE / COLLECTING / COARSE / FINE / MANUAL / DONE / FAILED）
- `CalibError` — 实时误差（reprojection_error / fitness_score / timestamp_offset）

---

## 十四、调度与线程模型

### 推荐线程架构

```
┌─────────────────────────────────────────────────────┐
│                     Main Thread                      │
│  Component::Init() + Component::Proc() 管理          │
│  DAG 配置调度                                       │
└─────────────────────────────────────────────────────┘

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  IO Thread       │  │  Processing       │  │  Optimization    │
│                  │  │  Thread           │  │  Thread          │
│ 数据接收         │  │ ─────────        │  │ ─────────       │
│ 解码             │  │ 点云处理          │  │ Ceres 求解       │
│ 时间戳提取       │  │ 图像特征提取      │  │ BA / ICP / 边缘   │
│ ────────         │  │ IMU 预积分        │  │ 多帧优化          │
│ 低延迟优先       │  │ ────────         │  │ ────────         │
│                  │  │ CPU 密集          │  │ 计算最密集        │
└──────────────────┘  └──────────────────┘  └──────────────────┘

┌──────────────────┐  ┌──────────────────┐
│  Viz Thread      │  │  Record Thread   │
│                  │  │                  │
│ GUI 渲染         │  │ Bag 写入         │
│ 点云投影显示     │  │ 文件 I/O         │
│ 误差曲线更新     │  │ 磁盘操作         │
│ ────────         │  │ ────────         │
│ 30 FPS 刷新      │  │ 低优先级         │
└──────────────────┘  └──────────────────┘
```

### Cyber RT 调度策略

| 组件 | 调度策略 | 优先级 | 说明 |
|------|---------|--------|------|
| Camera Component | CYBER_SCHED_IO | 高 | 帧率高，低延迟要求 |
| LiDAR Component | CYBER_SCHED_IO | 高 | 点云数据量大 |
| IMU Component | CYBER_SCHED_IO | 最高 | IMU 频率最高（200Hz） |
| Sync Component | CYBER_SCHED_BACKGROUND | 中 | 缓存 + 等待对齐 |
| CalibPipeline Component | CYBER_SCHED_BACKGROUND | 低 | 计算密集，可被抢占 |
| Viz Component | CYBER_SCHED_IO | 低 | 30 FPS 即可 |

---

## 十五、目录结构建议

### 整体项目目录

```
UniCalib-OK/
├── calib_unified/                # 现有系统（保持不动）
│   ├── apps/                     # 现有 7 个 main.cpp（保留离线模式）
│   ├── src/                      # 核心算法库
│   ├── include/                  # 头文件
│   ├── config/                   # YAML 配置
│   ├── cyber/                    # ◀ 现有 Cyber 资产（待启用）
│   │   ├── auto_calib_component.h/.cpp
│   │   ├── cyber_data_adapter.h
│   │   ├── calib_result.proto
│   │   └── shm_calibration_channel.h
│   ├── CMakeLists.txt            # ◀ 需增加 ENABLE_CYBER 选项
│   └── ...
│
├── modules/                      # ★ 新增：Cyber Component 模块
│   ├── camera/
│   │   ├── camera_component.h
│   │   ├── camera_component.cpp
│   │   └── BUILD                 # Cyber 构建文件
│   ├── lidar/
│   │   ├── lidar_component.h
│   │   ├── lidar_component.cpp
│   │   └── BUILD
│   ├── imu/
│   │   ├── imu_component.h
│   │   ├── imu_component.cpp
│   │   └── BUILD
│   ├── sync/
│   │   ├── sync_component.h
│   │   ├── sync_component.cpp
│   │   └── BUILD
│   ├── calib/
│   │   ├── calib_component.h
│   │   ├── calib_component.cpp
│   │   ├── coarse_component.cpp
│   │   ├── refine_component.cpp
│   │   ├── manual_component.cpp
│   │   ├── online_monitor.cpp
│   │   └── BUILD
│   ├── viz/
│   │   ├── viz_component.h
│   │   ├── viz_component.cpp
│   │   └── BUILD
│   ├── config/
│   │   ├── config_manager.h
│   │   ├── config_manager.cpp
│   │   └── BUILD
│   └── record/
│       ├── record_component.h
│       ├── record_component.cpp
│       └── BUILD
│
├── proto/                        # ★ 新增：Cyber Protobuf 定义
│   ├── sync_frame.proto
│   ├── calib_command.proto
│   ├── calib_status.proto
│   └── calib_error.proto
│
├── dag/                          # ★ 新增：Cyber DAG 启动文件
│   ├── calib_full.dag            # 完整标定系统
│   ├── calib_online.dag          # 在线监控
│   ├── calib_record.dag          # 录播系统
│   └── calib_debug.dag           # 调试模式 (含 Viz)
│
├── launch/                       # ★ 新增：cyber_launch 入口
│   ├── start_calib.sh
│   └── start_online_monitor.sh
│
└── docker/                       # ★ 新增
    ├── Dockerfile.runtime
    ├── Dockerfile.dev
    └── docker-compose.yml
```

---

## 十六、与现有系统的衔接

### 16.1 共存策略（过渡期）

```
┌─────────────────────────────────────────────────────────────┐
│                    CMake 构建系统                            │
│                                                             │
│  option(ENABLE_CYBER "Build Cyber RT components" OFF)       │
│                                                             │
│  ENABLE_CYBER=OFF 时：                                      │
│    → 编译现有 7 个 app (离线模式，完全不变)                    │
│                                                             │
│  ENABLE_CYBER=ON 时：                                       │
│    → 现有 app 仍然编译（离线标定继续可用）                    │
│    → 额外编译 modules/ 下的 Cyber Component                  │
│    → 编译 cyber/ 下的已有资产                                │
│    → 生成 .dag 文件供 cyber_launch 启动                      │
└─────────────────────────────────────────────────────────────┘
```

### 16.2 逐步迁移路径

| 阶段 | 离线模式 (ENABLE_CYBER=OFF) | 在线模式 (ENABLE_CYBER=ON) |
|------|--------------------------|--------------------------|
| 第一阶段 | 7 app 正常运行 | 启动 Sensor Component（仅采集，不标定） |
| 第二阶段 | 不受影响 | Sync Component 完成，可录制/回放 |
| 第三阶段 | 不受影响 | Calib Component 完成，在线标定可用 |
| 第四阶段 | 新增功能同步到 Component | Monitor + Viz Component 完成 |

### 16.3 现有代码复用率

| 现有模块 | 复用方式 | 复用率 |
|---------|---------|--------|
| `src/pipeline/calib_pipeline.cpp` | 直接复用为核心算法的入口 | 90% |
| `src/extrinsic/*.cpp` | 直接复用 | 100% |
| `src/solver/joint_calib_solver.cpp` | 直接复用 | 100% |
| `src/intrinsic/*.cpp` | 直接复用 | 100% |
| `src/io/lidar_packet_reader.cpp` | 封装为 LiDAR Component | 95% |
| `src/io/oem7_imu_reader.cpp` | 封装为 OEM7 Component | 95% |
| `src/io/new_format_data_source.cpp` | 部分逻辑迁移到 Sensor Component | 60% |
| `src/io/yaml_io.cpp` | 封装为 Config Manager | 90% |
| `src/viz/calib_visualizer.cpp` | 封装为 Viz Component | 80% |
| `src/viz/pangolin_render.cpp` | 复用 | 80% |
| `cyber/auto_calib_component.h/.cpp` | 取消 `#if 0` 后直接使用 | 100% |
| `cyber/cyber_data_adapter.h` | 补齐转换函数实现 | 30%→100% |
| `cyber/calib_result.proto` | 直接使用 | 100% |
| `cyber/shm_calibration_channel.h` | 直接使用 | 100% |

---

## 十七、开发阶段规划

### 第一阶段（1~2 周）：基础设施 + Component 化

**目标**：Cyber RT 构建系统就绪，Sensor Component 可独立运行。

| 任务 | 产出 | 涉及文件 |
|------|------|---------|
| CMake 增加 `ENABLE_CYBER` 选项 | 条件编译 | `calib_unified/CMakeLists.txt` |
| 启用 `cyber/auto_calib_component` | 取消 `#if 0` | `cyber/auto_calib_component.h/.cpp` |
| 启用 `calib_pipeline.h` 的 feed 接口 | 取消 `#if 0` | `include/unicalib/pipeline/calib_pipeline.h` |
| 实现 `cyber_data_adapter.h` 转换函数 | PointCloud2/Image/Imu → 内部 Frame | `cyber/cyber_data_adapter.h` |
| 创建 `modules/lidar/` 组件 | LiDAR Component | `modules/lidar/lidar_component.cpp` |
| 创建 `modules/imu/` 组件 | IMU Component | `modules/imu/imu_component.cpp` |
| 编写 DAG 文件 | `calib_debug.dag` | `dag/calib_debug.dag` |
| Docker 基础镜像 | `Dockerfile.runtime` | `docker/Dockerfile.runtime` |

### 第二阶段（2~4 周）：Sync + Calib Pipeline Component

**目标**：数据同步和标定流水线可在 Cyber RT 上运行。

| 任务 | 产出 | 涉及文件 |
|------|------|---------|
| Sync Component 实现 | 时间同步 + 帧发布 | `modules/sync/sync_component.cpp` |
| 新增 `SyncFrame` proto | 同步帧消息 | `proto/sync_frame.proto` |
| CalibPipelineComponent 实现 | 订阅 /sync/frame → 运行标定 | `modules/calib/calib_component.cpp` |
| 在线标定流程集成 | feed 接口 ↔ pipeline 对接 | `pipeline/calib_pipeline.cpp` |
| 新增 `CalibCommand` proto | 标定控制指令 | `proto/calib_command.proto` |
| 新增 `CalibStatus` proto | 标定状态消息 | `proto/calib_status.proto` |
| Config Manager 组件 | YAML 加载 + 参数校验 | `modules/config/config_manager.cpp` |

### 第三阶段（2~4 周）：在线监控 + 可视化 + 录播

**目标**：完整在线标定系统，含漂移检测和可视化。

| 任务 | 产出 | 涉及文件 |
|------|------|---------|
| 外参漂移检测模块 | 滑动窗口误差评估 | `modules/calib/online_monitor.cpp` |
| 实时重投影误差计算 | 错误回调机制 | `modules/calib/calib_component.cpp` |
| 自动回滚机制 | 失败恢复 | `pipeline/calib_pipeline.cpp` |
| Viz Component | 点云投影 + 误差曲线 | `modules/viz/viz_component.cpp` |
| Record Component | Cyber Record 封装 | `modules/record/record_component.cpp` |
| 完整 DAG 文件 | calib_full.dag | `dag/calib_full.dag` |
| Docker Compose 编排 | 多容器部署 | `docker/docker-compose.yml` |
| 域控部署脚本 | 启动/监控/日志 | `docker/scripts/` |

### 第四阶段（长期）：量产级系统

| 任务 | 说明 |
|------|------|
| Watchdog + Health Monitor | 组件健康检查、自动重启 |
| Web UI | 标定状态远程监控 |
| 多车型适配 | 参数模板 + 自动识别 |
| 云边协同 | 车端实时粗标定 + 云端大规模 BA |
| 自动工厂标定 | 插设备→自动标定→自动烧录 |

---

## 十八、未来扩展方向

### 18.1 多雷达标定

支持 front / left / right / back 多 LiDAR 同时标定（现有 `lidar_lidar_extrin` 支持多个 pair，但无实时版本）。

### 18.2 多相机标定

环视 / 鱼眼 / 双目相机阵列标定（现有 `cam_cam_extrin` 支持多 pair，可无缝迁移到 Cyber RT）。

### 18.3 自动工厂标定

实现"插设备 → 自动标定 → 自动烧录"的自动化流水线。

### 18.4 云边协同

- **车端**（域控）：实时漂移检测 + 局部优化
- **云端**：大规模联合 BA + 模型更新 + 参数下发

---

## 附录：关键文件清单

| 文件 | 作用 | 阶段 |
|------|------|------|
| `calib_unified/CMakeLists.txt` | 增加 `ENABLE_CYBER` 选项 | 阶段 1 |
| `calib_unified/cyber/auto_calib_component.h/.cpp` | 取消 `#if 0` | 阶段 1 |
| `calib_unified/cyber/cyber_data_adapter.h` | 补齐转换实现 | 阶段 1 |
| `calib_unified/cyber/calib_result.proto` | 已有，无需修改 | 阶段 1 |
| `include/unicalib/pipeline/calib_pipeline.h` | 取消 feed 接口的 `#if 0` | 阶段 1 |
| `modules/lidar/lidar_component.cpp` | LiDAR Cyber Component | 阶段 1 |
| `modules/imu/imu_component.cpp` | IMU Cyber Component | 阶段 1 |
| `modules/camera/camera_component.cpp` | Camera Cyber Component | 阶段 1 |
| `modules/sync/sync_component.cpp` | Sync Component | 阶段 2 |
| `modules/calib/calib_component.cpp` | CalibPipeline Component | 阶段 2 |
| `modules/calib/online_monitor.cpp` | 外参漂移检测 | 阶段 3 |
| `modules/viz/viz_component.cpp` | Visualization Component | 阶段 3 |
| `modules/record/record_component.cpp` | Record Component | 阶段 3 |
| `modules/config/config_manager.cpp` | Config Manager | 阶段 2 |
| `proto/*.proto` | 新增 protobuf 消息定义 | 阶段 2 |
| `dag/*.dag` | Cyber DAG 文件 | 阶段 1~3 |
| `docker/` | Docker 构建文件 | 阶段 1~3 |
