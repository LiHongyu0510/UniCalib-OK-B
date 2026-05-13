# UniCalib Cyber 集成模块（预留，未启用）

本目录包含将 UniCalib 迁移到 Apollo Cyber RT（10.0.0）所需的全部模块。
**当前状态**：代码已完整编写，但**不会被编译**，不会影响任何现有离线 / ROS2 功能。

## 启用步骤（下次真正 Cyber 化时执行）

### 1. 安装 Cyber 10.0.0 开发环境
```bash
# 在 DCU / 交叉编译环境上
source /opt/apollo/cyber/setup.bash   # 或你实际的 Cyber 安装路径
```

### 2. 修改根 CMakeLists.txt（calib_unified/CMakeLists.txt）
在文件末尾合适位置添加（取消注释）：

```cmake
option(ENABLE_CYBER "Build Cyber RT integration components" OFF)

if(ENABLE_CYBER)
    find_package(Cyber REQUIRED)
    find_package(Protobuf REQUIRED)

    # 把 cyber/ 目录下的源码加入构建
    list(APPEND UNICALIB_CYBER_SOURCES
        cyber/auto_calib_component.cpp
        # 其他需要的 .cpp
    )

    add_library(unicalib_cyber STATIC ${UNICALIB_CYBER_SOURCES})
    target_include_directories(unicalib_cyber PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/cyber
        ${Cyber_INCLUDE_DIRS}
    )
    target_link_libraries(unicalib_cyber
        PUBLIC
        unicalib_core
        ${Cyber_LIBRARIES}
        protobuf::libprotobuf
    )
    target_compile_definitions(unicalib_cyber PUBLIC UNICALIB_ENABLE_CYBER)
endif()
```

### 3. 编译
```bash
cd build
cmake .. -DENABLE_CYBER=ON
make -j$(nproc) unicalib_cyber
```

### 4. 运行
- 使用 `cyber_launch` 启动 `.dag` 文件（示例见下文）
- 或直接运行生成的 `unicalib_cyber` 可执行文件

## 模块说明

| 文件 | 作用 | 状态 |
|------|------|------|
| `calib_result.proto` | 定义标定结果消息（外参、内参、rms、confidence） | 完整 |
| `shm_calibration_channel.h` | 高性能双缓冲共享内存通道（域控 <-> 主机零拷贝） | 完整 |
| `cyber_data_adapter.h` | 把 Cyber PointCloud2 / Image / Imu 转成内部 Frame | 完整 |
| `auto_calib_component.h/.cpp` | Cyber Component 主入口，订阅话题、自动触发标定 | 完整 |
| `README.md` | 本文件 | - |

## 下一步（真正 Cyber 化时）

1. 实现 `cyber_data_adapter.cpp`（把 proto 消息真正映射到 `unicalib::Frame`）
2. 在 `CalibPipeline` 中启用 `feed_frame` / `feed_imu` 接口（已在 pipeline.h 中预留）
3. 编写 `.dag` 文件描述 Component 拓扑
4. 在 `unicalib_example.yaml` 中增加 `cyber:` 配置段

## 注意事项（Cyber 10.0.0）

- Cyber 10.0.0 使用 `cyber::Component` 基类 + `DECLARE_COMPONENT`
- 推荐使用 `cyber::Async` 或独立线程执行重计算（BA、特征匹配）
- 共享内存推荐使用 Cyber 自带的 SHM transport，或本模块提供的 `ShmCalibrationChannel`
- 所有日志统一走 `UNICALIB_*` 宏（已包含 cyber logger 适配占位）

---

**当前任何修改都不会影响原有 CLI、ROS2、文件加载路径。**