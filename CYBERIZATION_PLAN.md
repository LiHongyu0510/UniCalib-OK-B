# UniCalib Cyber 化完整转换方案

> 目标：将现有的离线标定工具升级为基于 Apollo Cyber RT 10.0 的模块化、实时化、可部署的在线标定平台。

---

## 一、现有 Cyber 资产盘点

项目已积累大量 Cyber 代码但**未编译**，需先理清家底：

| 资产目录 | 内容 | 状态 |
|---------|------|------|
| `calib_unified/cyber/` | `auto_calib_component` (主入口)、`calib_result.proto`、`cyber_data_adapter.h` (适配器)、`shm_calibration_channel.h` (共享内存通道) | 代码完整，被 `#if 0` 包裹 |
| `calib_unified/modules/` | 6 个 Cyber Component：`calib/`、`lidar/`、`imu/`、`sync/`、`viz/`、`record/` | 以 `#ifdef ENABLE_CYBER` 条件编译，实现为 stub |
| `calib_unified/proto/` | `calib_command.proto`、`calib_status.proto`、`sync_frame.proto` | 完整 protobuf 定义 |
| `calib_unified/dag/` | `calib_basic.dag`、`calib_full.dag` | XML 格式，引用 `libunicalib_cyber.so` |
| `calib_unified/docs/CYBER_MIGRATION_PLAN.md` | 详细迁移设计文档 (961 行) | 设计完成，未执行 |
| `calib_unified/include/unicalib/pipeline/calib_pipeline.h:403` | `feed_lidar_frame`、`feed_camera_frame`、`feed_imu_data`、`try_auto_calibrate` | 被 `#if 0` 包裹，需启用 |

### 代码成熟度评估

| 模块 | 可直接启用 | 需补齐实现 | 需重写 | 需新增 |
|------|-----------|-----------|-------|-------|
| `shm_calibration_channel.h` | ✅ 生产级 | | | |
| `calib_result.proto` | ✅ 完整 | | | |
| `calib_command.proto` | ✅ 完整 | | | |
| `calib_status.proto` | ✅ 完整 | | | |
| `sync_frame.proto` | ✅ 完整 | | | |
| `auto_calib_component.h/.cpp` | ✅ 骨架完整 | 需填充 `try_auto_calibrate`、`should_trigger_calibration`、`publish_result` | | |
| `cyber_data_adapter.h` | | 3 个转换函数全为空实现 | | |
| `modules/lidar/lidar_component.h` | | 仅有 header，无 `.cpp` | | |
| `modules/imu/imu_component.h` | | 仅为最小 trivially 实现 | | |
| `modules/sync/sync_component.cpp` | | 实现为 stub，TODO 未填充 | | |
| `modules/calib/calib_component.cpp` | | 实现为 stub | | |
| `modules/calib/online_monitor.cpp` | | 仅有打印日志 | | |
| `modules/viz/viz_component.cpp` | | 仅为占位 | | |
| `modules/record/record_component.cpp` | | 仅为占位 | | |
| `calib_pipeline.h` feed 接口 | | 被 `#if 0` 包裹 | | |
| CMakeLists.txt Cyber 构建 | | | 需新增 `ENABLE_CYBER` 选项 | |
| Docker Cyber SDK 安装 | | | | 需新增 |
| Camera Component | | | | 需新建 |
| Config Manager Component | | | | 需新建 |
| DAG 文件完善 | | | `calib_basic.dag` 仅 2 个组件 | 需补充 |
| 在线漂移检测逻辑 | | `online_monitor.cpp` 全空 | | |

---

## 二、环境依赖方案

### 2.1 Cyber RT SDK 安装

Apollo Cyber RT 10.0 依赖清单：

```bash
# ===== 必需依赖 =====
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libgoogle-glog-dev \
    libgflags-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libboost-all-dev \
    libevent-dev \
    libyaml-cpp-dev \
    libuuid1 \
    uuid-dev

# ===== Cyber RT 源码编译 =====
git clone https://github.com/ApolloAuto/apollo.git -b v10.0.0
cd apollo/cyber
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/apollo/cyber
make -j$(nproc)
sudo make install

# ===== 环境变量 =====
# 添加到 ~/.bashrc 或 setup.bash：
export CYBER_PATH=/opt/apollo/cyber
export PATH=$CYBER_PATH/bin:$PATH
export LD_LIBRARY_PATH=$CYBER_PATH/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=$CYBER_PATH:$CMAKE_PREFIX_PATH
```

### 2.2 Docker 镜像方案

当前 Dockerfile 基于 `nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04` + ROS2 Humble。

需新增 Cyber RT 安装层（建议在 ROS2 安装之后）：

```dockerfile
# === Dockerfile 新增 ===
# Cyber RT 10.0 开发库
COPY deps/cyber_10.0.0_amd64.deb /tmp/
RUN dpkg -i /tmp/cyber_10.0.0_amd64.deb || true && \
    apt-get install -f -y && \
    rm /tmp/cyber_10.0.0_amd64.deb

# 或从源码编译 Cyber（推荐，更可控）
COPY docker/scripts/build_cyber.sh /opt/scripts/
RUN /opt/scripts/build_cyber.sh
```

**推荐策略**：将 Cyber RT 预编译为 deb 包或 docker layer，避免每次构建都从源码编译。

### 2.3 环境检测脚本

新增 `verify_cyber_env.sh`，与现有 `verify_environment.sh` 并列：

```bash
#!/bin/bash
# 检测 Cyber RT 开发环境

# 1. 检测 cyber 头文件
if [ -f "$CYBER_PATH/include/cyber/cyber.h" ]; then
    echo "✓ Cyber RT headers: $CYBER_PATH"
else
    echo "✗ Cyber RT headers not found"
    echo "  Set CYBER_PATH or install Cyber RT SDK"
fi

# 2. 检测 protoc
PROTOC_VER=$(protoc --version 2>/dev/null | grep -oP '\d+\.\d+\.\d+')
echo "  protoc: ${PROTOC_VER:-not found}"

# 3. 检测 cyber_launch
if command -v cyber_launch &>/dev/null; then
    echo "✓ cyber_launch: $(which cyber_launch)"
fi
```

---

## 三、构建系统变更 (CMake)

### 3.1 CMakeLists.txt 修改方案

在 `calib_unified/CMakeLists.txt` 末尾新增 Cyber 构建块（与现有构建完全隔离）：

```cmake
# ===========================================================================
# Cyber RT 集成（可选，默认关闭）
# ===========================================================================
option(ENABLE_CYBER "Build Apollo Cyber RT integration" OFF)

if(ENABLE_CYBER)
    # 1. Cyber RT 依赖
    find_package(Cyber REQUIRED)
    find_package(Protobuf REQUIRED)

    # 2. PROTO 编译
    file(GLOB CYBER_PROTO_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto
        ${CMAKE_CURRENT_SOURCE_DIR}/cyber/*.proto)
    # 自定义 proto 编译规则...
    PROTOBUF_GENERATE_CPP(CYBER_PROTO_SRCS CYBER_PROTO_HDRS
        ${CYBER_PROTO_SRCS})

    # 3. Cyber Component 源文件
    set(CYBER_COMPONENT_SOURCES
        # core components
        modules/calib/calib_component.cpp
        modules/calib/online_monitor.cpp
        modules/lidar/lidar_component.cpp     # <-- 需新建
        modules/imu/imu_component.cpp          # <-- 需新建
        modules/sync/sync_component.cpp
        modules/viz/viz_component.cpp
        modules/record/record_component.cpp
        # camera component (需新建)
        modules/camera/camera_component.cpp
        # config manager (需新建)
        modules/config/config_manager.cpp
        # cyber adapter
        cyber/auto_calib_component.cpp
        # 生成的 proto 源文件
        ${CYBER_PROTO_SRCS}
    )

    # 4. 构建 Cyber 共享库 (供 DAG 加载)
    add_library(unicalib_cyber SHARED ${CYBER_COMPONENT_SOURCES})

    target_include_directories(unicalib_cyber PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/proto   # 生成的 proto 头文件
        ${Cyber_INCLUDE_DIRS}
    )

    target_link_libraries(unicalib_cyber PUBLIC
        unicalib                                # 链接现有核心库
        ${Cyber_LIBRARIES}                      # cyber, cyber_core
        protobuf::libprotobuf
    )

    target_compile_definitions(unicalib_cyber PUBLIC ENABLE_CYBER=1)

    # 5. DAG 文件安装
    install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/dag/calib_basic.dag
        ${CMAKE_CURRENT_SOURCE_DIR}/dag/calib_full.dag
        DESTINATION ${CMAKE_INSTALL_PREFIX}/dag
    )
endif()
```

### 3.2 build.sh 修改

增加 `--cyber` 参数：

```bash
# 新增参数
ENABLE_CYBER=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --cyber)       ENABLE_CYBER=1 ;;
        # ... 现有参数
    esac
done

# 调用 cmake 时增加
CMAKE_ARGS+=(-DENABLE_CYBER="$ENABLE_CYBER")

# 编译后额外检查 cyber 库
if [ "$ENABLE_CYBER" -eq 1 ]; then
    CYBER_LIB="${BUILD_DIR}/lib/libunicalib_cyber.so"
    if [ -f "$CYBER_LIB" ]; then
        info "  ✓ libunicalib_cyber.so ($(du -sh "$CYBER_LIB" | cut -f1))"
    fi
fi
```

### 3.3 Makefile 修改

新增 Cyber 相关 target：

```makefile
# === Makefile 新增 ===
CYBER_BUILD_DIR := calib_unified/build_cyber

.PHONY: build-cyber
build-cyber:
    mkdir -p $(CYBER_BUILD_DIR) && \
    cd $(CYBER_BUILD_DIR) && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_CYBER=ON && \
    make -j$(shell nproc)

.PHONY: run-cyber
run-cyber: build-cyber
    cyber_launch start calib_unified/dag/calib_basic.dag

.PHONY: run-cyber-online
run-cyber-online: build-cyber
    cyber_launch start calib_unified/dag/calib_full.dag
```

---

## 四、代码变更清单

### 第 1 阶段：基础设施（1-2 周）

| 文件 | 操作 | 说明 |
|------|------|------|
| `calib_unified/CMakeLists.txt` | 新增 `ENABLE_CYBER` 选项 + Cyber 构建块 | 条件编译 |
| `calib_unified/build.sh` | 新增 `--cyber` 参数 | 构建入口 |
| `calib_unified/Makefile` (或根 Makefile) | 新增 `build-cyber`、`run-cyber` target | 便捷调用 |
| `docker/Dockerfile` | 新增 Cyber RT SDK 安装层 | 环境依赖 |
| `verify_cyber_env.sh` | 新建 | 环境检测 |
| `.gitignore` | 新增 `build_cyber/`、`*.pb.h`、`*.pb.cc` | 版本控制 |

### 第 2 阶段：启用已有代码（1 周）

| 文件 | 操作 | 说明 |
|------|------|------|
| `cyber/auto_calib_component.h` | 取消 `#if 0` | 启用 Component |
| `cyber/auto_calib_component.cpp` | 取消 `#if 0`，填充 `try_auto_calibrate` | 实现触发逻辑 |
| `cyber/cyber_data_adapter.h` | 取消 `#if 0`，实现 3 个转换函数 | 消息转换 |
| `calib_pipeline.h:403` | 取消 `#if 0` | 启用 feed 接口 |
| `modules/*/*.h/.cpp` | 取消 `#ifdef ENABLE_CYBER` 包裹 | 启用所有 Component |

### 第 3 阶段：补齐实现（2-4 周）

| 文件 | 缺失内容 | 工作量 |
|------|---------|--------|
| `modules/lidar/lidar_component.cpp` | 需新建，封装 `lidar_packet_reader.cpp` 为 Component | 中 |
| `modules/imu/imu_component.cpp` | 需新建，封装 `oem7_imu_reader.cpp` | 中 |
| `modules/camera/camera_component.cpp` | 需新建，封装图像采集 + 解码 | 中 |
| `modules/sync/sync_component.cpp` | 补齐时间同步 + 组帧逻辑 | 高 |
| `modules/calib/calib_component.cpp` | 补齐 feed → pipeline 对接 | 高 |
| `modules/calib/online_monitor.cpp` | 滑动窗口漂移检测 | 高 |
| `modules/config/config_manager.cpp` | 需新建，封装 `yaml_io.cpp` | 中 |
| `modules/viz/viz_component.cpp` | 补齐 Pngolin/OpenCV 窗口 | 中 |
| `modules/record/record_component.cpp` | 补齐 Cyber Record 封装 | 低 |
| `cyber/cyber_data_adapter.h` | 3 个转换函数从 stub 到完整实现 | 中 |

### 第 4 阶段：集成测试与部署（2 周）

| 任务 | 说明 |
|------|------|
| DAG 文件完善 | 为每个 Component 编写 DAG 配置 |
| launch 脚本 | `start_calib.sh`、`start_online_monitor.sh` |
| Docker 多阶段构建 | `Dockerfile.runtime` (最小) + `Dockerfile.dev` (开发) |
| 域控部署脚本 | 启动/监控/日志/健康检查 |
| 流水线 CI | GitHub Actions 编译检查 |

---

## 五、详细文件变更对照

### 5.1 `calib_unified/CMakeLists.txt`

**当前**（1048 行）：无任何 Cyber 相关代码。

**变更**：在 `# 安装规则` 之后、文件末尾前，插入 ~60 行 Cyber 构建块（见第三节）。

### 5.2 `calib_unified/cyber/auto_calib_component.cpp`

**当前**：整体被 `#if 0` 包裹，关键逻辑为 TODO。

**变更**：
1. 取消 `#if 0` / `#endif`
2. 填充 `Init()`：读取配置 → 初始化 pipeline → 创建 Reader/Writer
3. 填充 `try_auto_calibrate()`：调用 `pipeline_->try_auto_calibrate()`
4. 填充 `should_trigger_calibration()`：实现静止检测、时间间隔判断
5. 填充 `publish_result()`：Cyber Writer 写 + SHM 通道写

### 5.3 `calib_unified/cyber/cyber_data_adapter.h`

**当前**：3 个转换函数全部返回 `false`。

**变更**：
```cpp
// from_cyber_pointcloud2: 遍历 PointCloud2 的 data 字段 → PointCloud
// from_cyber_image: 提取 Image 的 data + encoding → cv::Mat
// from_cyber_imu: 提取 Imu 的 angular_velocity/linear_acceleration → ImuData
```

### 5.4 `calib_unified/include/unicalib/pipeline/calib_pipeline.h`

**当前**：4 个流式接口被 `#if 0` 包裹。

**变更**：取消 `#if 0` / `#endif`，使接口可用。实现需维护内部环形缓冲区。

### 5.5 `modules/lidar/lidar_component.cpp` (需新建)

参考 `calib_unified/src/io/lidar_packet_reader.cpp` (899 行) 封装：
- 初始化时创建 `LidarPacketReader`
- 每帧从 LiDAR SDK 获取点云
- 转为 Cyber PointCloud2 发布

### 5.6 `modules/camera/camera_component.cpp` (需新建)

封装现有相机数据加载逻辑：
- 图像采集 (VideoCapture / 文件回放)
- h265 解码 (复用 `sensor_decode/h265_decode/`)
- 时间戳生成 → 发布 Image 消息

### 5.7 `modules/config/config_manager.cpp` (需新建)

封装 `calib_unified/src/io/yaml_io.cpp`：
- 读取 sensors.yaml + unicalib_example.yaml
- 参数校验
- 通过 Cyber Topic 发布配置热更新

---

## 六、依赖关系图

```
                    ┌──────────────────┐
                    │  Cyber RT 10.0   │
                    │  (新依赖)         │
                    └────────┬─────────┘
                             ▼
┌──────────────────────────────────────────────────┐
│              libunicalib_cyber.so                 │
│  ┌────────────┐  ┌──────────┐  ┌──────────────┐ │
│  │ Component  │  │  Proto   │  │   Adapter    │ │
│  │ 框架       │  │  消息     │  │   数据转换    │ │
│  └────────────┘  └──────────┘  └──────────────┘ │
└──────────────────────┬───────────────────────────┘
                       ▼
┌──────────────────────────────────────────────────┐
│              libunicalib.so (现有)                 │
│  Pipeline | Extrinsic | Intrinsic | Solver | Viz │
└──────────────────────────────────────────────────┘
```

### 新增依赖

| 依赖 | 版本 | 来源 | 用途 |
|------|------|------|------|
| Cyber RT | 10.0.0 | Apollo 源码编译或 deb | Component 框架 + 通信 |
| Protobuf | ≥ 3.14 | 系统 apt | 消息序列化 |
| google-glog | ≥ 0.6 | 系统 apt | Cyber 日志依赖 |
| gflags | ≥ 2.2 | 系统 apt | Cyber 参数依赖 |

### 现有依赖不受影响

所有现有依赖（Eigen3、Ceres、OpenCV、PCL、spdlog、yaml-cpp、fmt、Sophus 等）保持不变。

---

## 七、共存策略（重要）

Cyber 化必须**不影响现有离线功能**：

```mermaid
graph TD
    subgraph "ENABLE_CYBER=OFF (默认)"
        A[现有 CMake 构建] --> B[7 个离线 app]
        A --> C[libunicalib.so]
        B --> D[离线标定正常运行]
    end

    subgraph "ENABLE_CYBER=ON"
        E[cmake -DENABLE_CYBER=ON] --> F[libunicalib.so (不变)]
        E --> G[libunicalib_cyber.so (新增)]
        G --> H[Cyber Component]
        H --> I[cyber_launch 启动]
    end
```

**原则**：
1. `ENABLE_CYBER=OFF`（默认）：编译结果与当前完全一致，无任何变化
2. `ENABLE_CYBER=ON`：现有 app 仍然编译，额外新增 `libunicalib_cyber.so`
3. 所有 Cyber 代码通过 `#ifdef ENABLE_CYBER` 或条件编译隔离
4. 不允许 Cyber 代码影响现有离线功能路径

---

## 八、分阶段实施计划

### Stage 1：基础设施（预计 1-2 周）
```
[ ] 安装 Cyber RT 10.0 SDK 到开发环境 / Docker
[ ] 修改 CMakeLists.txt，新增 ENABLE_CYBER 选项
[ ] 验证：cmake -DENABLE_CYBER=ON 能正常生成构建系统
[ ] 创建最小的 "hello world" Cyber Component 验证链路通
```

### Stage 2：启用现有资产（预计 1 周）
```
[ ] 取消 cyber/* 的 #if 0，使其参与编译
[ ] 取消 modules/* 的 #ifdef ENABLE_CYBER，使其参与编译
[ ] 取消 calib_pipeline.h 中 feed 接口的 #if 0
[ ] 修改 proto 编译规则，使 proto 文件生成 C++ 源码
[ ] 验证：libunicalib_cyber.so 能正常链接和加载
```

### Stage 3：功能补齐（预计 2-4 周）
```
[ ] 实现 cyber_data_adapter.h 的 3 个转换函数
[ ] 创建 camera_component.cpp（需新建）
[ ] 创建 config_manager.cpp（需新建）
[ ] 补齐 sync_component.cpp 的时间同步逻辑
[ ] 补齐 calib_component.cpp 的 feed → pipeline 对接
[ ] 补齐 auto_calib_component.cpp 的触发逻辑
[ ] 补齐 lidar_component.cpp（封装 lidar_packet_reader）
[ ] 补齐 imu_component.cpp（封装 oem7_imu_reader）
```

### Stage 4：在线监控 + 可视化（预计 2-4 周）
```
[ ] 补齐 online_monitor.cpp 的滑动窗口漂移检测
[ ] 补齐 viz_component.cpp 的实时可视化
[ ] 补齐 record_component.cpp 的录播功能
[ ] 完善 DAG 文件（calib_basic.dag、calib_full.dag）
[ ] 编写 launch 启动脚本
```

### Stage 5：部署 + 测试（预计 2 周）
```
[ ] Docker 多阶段构建（runtime + dev 镜像分离）
[ ] 域控部署脚本
[ ] 集成测试（离线回归测试 + Cyber 在线测试）
[ ] 性能测试（延迟、吞吐量、CPU/内存）
```

---

## 九、风险与缓解措施

| 风险 | 影响 | 概率 | 缓解措施 |
|------|------|------|---------|
| Cyber RT 10.0 与 CUDA 11.8 兼容性 | Docker 构建失败 | 中 | 单独构建 Cyber 为 deb，分步安装 |
| Cyber RT Component 调度导致 CPU 争抢 | 标定性能下降 | 中 | 使用 CYBER_SCHED_BACKGROUND 降低标定优先级 |
| feed_* 接口实现复杂度超预期 | 工期延长 | 高 | 第一版仅支持文件回放模式，实时模式第二版实现 |
| 现有 pipeline 与 feed 接口不兼容 | 代码冲突 | 中 | 保持离线 pipeline 不变，feed 路径为新增代码 |
| proto 版本冲突 (protobuf) | 编译失败 | 低 | 固定 protobuf 版本 (3.14+) |

---

## 十、验证方法

### 10.1 构建验证
```bash
# 离线模式（默认）
cd calib_unified && mkdir -p build_offline && cd build_offline
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
# → 所有现有 app 正常编译，无 Cyber 相关代码

# Cyber 模式
cd .. && mkdir -p build_cyber && cd build_cyber
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_CYBER=ON
make -j$(nproc)
# → libunicalib.so + libunicalib_cyber.so + 所有 app 正常编译
```

### 10.2 运行验证
```bash
# 1. 离线 app 运行（回归测试）
./build_offline/bin/unicalib_imu_intrinsic --help
./build_offline/bin/unicalib_joint --help
# → 与之前行为完全一致

# 2. Cyber Component 加载测试
source /opt/apollo/cyber/setup.bash
cyber_launch start calib_unified/dag/calib_basic.dag
# → Component Init 成功，无崩溃

# 3. 共享内存通道测试
# 单进程测试 shm_calibration_channel（写 + 读在同一程序验证 seqlock）
```

### 10.3 回归测试
```bash
# 使用现有测试数据，与离线模式结果对比
# 对同一数据集运行相同的标定算法：
#   离线模式: ./build/bin/unicalib_imu_lidar --config config/unicalib_example.yaml
#   Cyber 模式: cyber_launch start calib_debug.dag
# 标定结果 YAML 应一致
```

---

## 十一、总结

### 核心原则
1. **零侵入**：Cyber 代码完全隔离，默认关闭，不影响现有离线功能
2. **渐进式**：分 5 个阶段逐步推进，每个阶段独立可验证
3. **高复用**：~90% 的代码直接复用现有 `libunicalib.so`，Cyber 层仅做组件化封装

### 投入估算

| 阶段 | 内容 | 预估人月 |
|------|------|---------|
| Stage 1 | 基础设施 | 0.5 |
| Stage 2 | 启用现有资产 | 0.3 |
| Stage 3 | 功能补齐 | 1.0-1.5 |
| Stage 4 | 在线监控 + 可视化 | 1.0 |
| Stage 5 | 部署 + 测试 | 0.5 |
| **合计** | | **3.3-3.8 人月** |

### 产出物
- 新增 `libunicalib_cyber.so`（Cyber RT Component 动态库）
- 8 个 Cyber Component（camera / lidar / imu / sync / calib / viz / record / config）
- 4 个 protobuf 消息定义
- 2 个 DAG 拓扑文件
- Docker 多阶段镜像
- 域控部署脚本
