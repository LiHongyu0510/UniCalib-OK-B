# CMakeLists.txt 具体改进方案
## basalt-headers + Pangolin-0.9.0 集成

**文档版本**: v1.0  
**重点**: CMake 配置、源文件组织、链接依赖

---

## 目录
1. [选项声明](#选项声明)
2. [basalt-headers 配置](#basalt-headers-配置)
3. [源文件收集重构](#源文件收集重构)
4. [目标链接配置](#目标链接配置)
5. [可视化后端选择](#可视化后端选择)
6. [完整 CMakeLists.txt 片段](#完整cmakelists-txt-片段)

---

## 选项声明

### 位置：CMakeLists.txt 行 42-54

**当前代码（需修改）**:
```cmake
option(UNICALIB_WITH_IKALIBR    "Build iKalibr B-spline calibration engine" ON)
option(BUILD_TESTING            "Build tests"                          OFF)
```

**改进后**:
```cmake
# ───────────────────────────────────────────────────────────────────
# 编译选项：B-样条与可视化独立控制
# ───────────────────────────────────────────────────────────────────

# 选项 1: B-样条求解器（基于 basalt-headers）
# - 无 ROS 强制依赖，仅需 Ceres
# - 可与任何可视化后端组合
option(UNICALIB_WITH_IKALIBR
    "Build B-spline solver core (requires Ceres, Eigen, Sophus)"
    ON)

# 选项 2: Pangolin 3D 可视化（本地库，推荐）
# - 纯 C++，依赖 OpenGL/X11，无 ROS
# - 支持轨迹、点云、优化曲线交互显示
option(UNICALIB_WITH_PANGOLIN
    "Use Pangolin for 3D visualization (OpenGL backend)"
    ON)

# 选项 3: ROS RViz 可视化（可选）
# - 仅在 ROS2 环境有效
# - 与 Pangolin 互斥（Pangolin 优先）
option(UNICALIB_WITH_ROS_RVIZ
    "Enable ROS RViz visualization adapter (requires ROS2)"
    OFF)

option(BUILD_TESTING            "Build tests"                          OFF)
```

**编译命令示例**:
```bash
# 完整编译
cmake .. -DUNICALIB_WITH_IKALIBR=ON -DUNICALIB_WITH_PANGOLIN=ON

# 精简编译（无可视化）
cmake .. -DUNICALIB_WITH_IKALIBR=ON -DUNICALIB_WITH_PANGOLIN=OFF

# 最小编译（快速）
cmake .. -DUNICALIB_WITH_IKALIBR=OFF -DUNICALIB_WITH_PANGOLIN=OFF
```

---

## basalt-headers 配置

### 位置：CMakeLists.txt 行 155-170（新增）

**关键点**：
1. 验证本地存在
2. 设为 INTERFACE 库（header-only）
3. 声明依赖（Eigen3, Sophus）

### 完整代码块

```cmake
# ===========================================================================
# 第三方库：basalt-headers（B-样条核心，header-only）
# ===========================================================================
# 来源：https://gitlab.com/VladyslavUsenko/basalt-headers.git
# 功能：
#   - Se3Spline<N, Scalar>: SE(3) 均匀 B-样条（分割表示法）
#   - So3Spline<N, Scalar>: SO(3) 累积 B-样条
#   - RdSpline<DIM, N, Scalar>: R^d 欧氏 B-样条
#   - CeresSplineHelper: Ceres 自动求导集成
#   - CeresLocalParamSE3: SE(3) 李群参数化

set(BASALT_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers)

if(NOT EXISTS "${BASALT_HEADERS_DIR}/include/basalt/spline/se3_spline.h")
    message(FATAL_ERROR 
        "basalt-headers not found at ${BASALT_HEADERS_DIR}. "
        "Expected: ${BASALT_HEADERS_DIR}/include/basalt/spline/se3_spline.h")
endif()

# 创建 INTERFACE 库
add_library(basalt_headers INTERFACE)
target_include_directories(basalt_headers INTERFACE
    ${BASALT_HEADERS_DIR}/include)

# basalt-headers 本身需要 Eigen3 和 Sophus
target_link_libraries(basalt_headers INTERFACE
    Eigen3::Eigen
    Sophus::Sophus)

# 别名
add_library(basalt::headers ALIAS basalt_headers)

message(STATUS "basalt-headers: ✓ Found at ${BASALT_HEADERS_DIR}/include")
message(STATUS "  • Features: Se3Spline, So3Spline, RdSpline, CeresHelper")
```

---

## 源文件收集重构

### 问题描述

**当前状态**（行 472-491）：
```cmake
# 旧代码的问题：
# 1. UNICALIB_WITH_IKALIBR=ON 时强制编译所有 iKalibr 文件
# 2. 可视化文件（viewer.cpp）受 UNICALIB_WITH_IKALIBR 控制
#    → 即使只想要 Pangolin 可视化，也必须编译 iKalibr
# 3. 无法选择多个可视化后端
```

### 改进方案

**新代码结构**（行 472-550）：

```cmake
# ===========================================================================
# 核心库构造：三层解耦
# ===========================================================================

# ─────────────────────────────────────────────────────────────────────────
# 第 1 层检查：B-样条求解器的前置条件
# ─────────────────────────────────────────────────────────────────────────
if(UNICALIB_WITH_IKALIBR)
    message(STATUS "iKalibr: checking dependencies...")
    
    # B-样条求解需要 Ceres
    if(NOT Ceres_FOUND)
        message(FATAL_ERROR
            "iKalibr requires Ceres but it was not found. "
            "Installation: sudo apt install libceres-dev")
    endif()
    
    # B-样条求解需要 Sophus（李群库）
    if(NOT Sophus_FOUND)
        message(FATAL_ERROR
            "iKalibr requires Sophus but it was not found. "
            "Installation: sudo apt install libsophus-dev")
    endif()
    
    message(STATUS "iKalibr: dependencies ✓ (Ceres, Sophus, Eigen3)")
endif()

# ─────────────────────────────────────────────────────────────────────────
# 第 2 层选择：可视化后端（互斥：Pangolin > RViz > Stub）
# ─────────────────────────────────────────────────────────────────────────

set(_VIEWER_SOURCES)
set(_VIEWER_BACKEND "NONE")

if(UNICALIB_WITH_PANGOLIN)
    # Pangolin 优先级最高（本地库，无额外依赖）
    
    # Pangolin 已在行 136-146 配置好
    if(NOT TARGET pangolin)
        message(FATAL_ERROR "Pangolin target not found. Check CMakeLists.txt line 136-146")
    endif()
    
    # 添加 viewer 实现
    list(APPEND _VIEWER_SOURCES
        src/ikalibr/viewer/viewer.cpp
        src/ikalibr/viewer/viewer_pangolin.cpp
    )
    set(_VIEWER_BACKEND "PANGOLIN")
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=PANGOLIN)
    
    message(STATUS "Visualization: ✓ Pangolin 3D backend (interactive OpenGL)")
    
elseif(UNICALIB_WITH_ROS_RVIZ AND UNICALIB_WITH_ROS2)
    # RViz 次优先级（需要 ROS2）
    
    find_package(rclcpp QUIET)
    find_package(visualization_msgs QUIET)
    
    if(rclcpp_FOUND AND visualization_msgs_FOUND)
        list(APPEND _VIEWER_SOURCES
            src/viz/ros_rviz_visualizer.cpp
        )
        set(_VIEWER_BACKEND "RVIZ")
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=RVIZ)
        
        message(STATUS "Visualization: ✓ ROS RViz backend (via markers)")
    else()
        message(WARNING 
            "ROS2 visualization libraries not found. "
            "Fall back to headless mode. "
            "Installation: sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-visualization-msgs")
        
        list(APPEND _VIEWER_SOURCES
            src/viz/viewer_stub.cpp
        )
        set(_VIEWER_BACKEND "STUB")
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=STUB)
        
        message(STATUS "Visualization: ⚠ Headless mode (no GUI)")
    endif()
    
else()
    # 默认：Stub（无 GUI，纯计算）
    list(APPEND _VIEWER_SOURCES
        src/viz/viewer_stub.cpp
    )
    set(_VIEWER_BACKEND "STUB")
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=STUB)
    
    message(STATUS "Visualization: ⚠ Headless mode (no GUI)")
endif()

# ─────────────────────────────────────────────────────────────────────────
# 第 3 层构造：主库源文件列表
# ─────────────────────────────────────────────────────────────────────────

if(UNICALIB_WITH_IKALIBR)
    add_compile_definitions(UNICALIB_WITH_IKALIBR=1)
    
    # 完整源文件列表 = iKalibr 核心 + 加载器 + UniCalib 新增 + 可视化后端
    set(_UNICALIB_SOURCES
        # iKalibr 核心（行 357-425）
        ${IKALIBR_CORE_SOURCES}
        # 数据加载器
        ${IKALIBR_LOADER_SOURCES}
        # UniCalib 新增（行 438-462）
        ${UNICALIB_NEW_SOURCES}
        # 可视化后端（根据选择）
        ${_VIEWER_SOURCES}
    )
    
    message(STATUS "iKalibr: ✓ Enabled (B-spline solver via basalt-headers)")
    message(STATUS "  • Spline backend: basalt-headers")
    message(STATUS "  • Visualization backend: ${_VIEWER_BACKEND}")
    
else()
    add_compile_definitions(UNICALIB_WITH_IKALIBR=0)
    
    # 精简源文件列表：仅 UniCalib 模块 + IMU-LiDAR 最小依赖
    set(_UNICALIB_SOURCES
        ${UNICALIB_NEW_SOURCES}
        ${IKALIBR_MINIMAL_FOR_IMU_LIDAR}
    )
    
    message(STATUS "iKalibr: ⚠ Disabled (lean mode)")
    message(STATUS "  • Use -DUNICALIB_WITH_IKALIBR=ON to enable B-spline solver")
endif()

# ─────────────────────────────────────────────────────────────────────────
# 第 4 步：创建主库目标
# ─────────────────────────────────────────────────────────────────────────

add_library(unicalib SHARED ${_UNICALIB_SOURCES})

# 包含目录
target_include_directories(unicalib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ikalibr>
    $<INSTALL_INTERFACE:include>)

# ─────────────────────────────────────────────────────────────────────────
# 第 5 步：链接依赖
# ─────────────────────────────────────────────────────────────────────────

# 通用依赖（所有模式）
target_link_libraries(unicalib PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
    basalt::headers  # ← 关键：B-样条库
    magic_enum::magic_enum
    cereal::cereal
    veta::veta
    tiny_viewer::stub
    opencv_core
    opencv_imgproc
    opencv_highgui
)

# B-样条求解器依赖（仅 IKALIBR 时）
if(UNICALIB_WITH_IKALIBR)
    target_link_libraries(unicalib PUBLIC Ceres::ceres)
endif()

# Pangolin 可视化依赖
if(UNICALIB_WITH_PANGOLIN AND _VIEWER_BACKEND STREQUAL "PANGOLIN")
    target_link_libraries(unicalib PUBLIC pangolin)
endif()

# RViz 可视化依赖
if(UNICALIB_WITH_ROS_RVIZ AND _VIEWER_BACKEND STREQUAL "RVIZ")
    target_link_libraries(unicalib PUBLIC
        rclcpp::rclcpp
        visualization_msgs::visualization_msgs)
endif()

# 可选依赖
if(UNICALIB_WITH_ROS2)
    target_link_libraries(unicalib PUBLIC
        rclcpp::rclcpp
        rosbag2_cpp::rosbag2_cpp)
endif()

if(UNICALIB_WITH_OPENMP)
    target_link_libraries(unicalib PUBLIC OpenMP::OpenMP_CXX)
endif()

if(UNICALIB_WITH_PCL_VIZ)
    target_link_libraries(unicalib PUBLIC pcl_common pcl_io)
endif()

message(STATUS "unicalib: ✓ Library configured (${_VIEWER_BACKEND} backend)")
```

---

## 目标链接配置

### 关键链接顺序

```cmake
# 推荐链接顺序（避免符号未定义）
target_link_libraries(unicalib PUBLIC
    # 1. 几何库（最基础）
    Eigen3::Eigen
    Sophus::Sophus
    
    # 2. 本地库（header-only）
    basalt::headers
    magic_enum::magic_enum
    cereal::cereal
    veta::veta
    
    # 3. 优化求解器
    Ceres::ceres      # 仅 IKALIBR=ON
    
    # 4. 可视化库
    pangolin          # 仅 Pangolin 后端
    rclcpp::rclcpp    # 仅 RViz 后端
    
    # 5. 通用库
    opencv_core
    opencv_imgproc
)
```

### 检查链接有效性

```cmake
# 在 CMakeLists.txt 末尾添加
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    get_target_property(_LINK_LIBS unicalib LINK_LIBRARIES)
    message(STATUS "unicalib LINK_LIBRARIES:")
    foreach(_LIB ${_LINK_LIBS})
        message(STATUS "  • ${_LIB}")
    endforeach()
endif()
```

---

## 可视化后端选择

### 场景 1: Pangolin（默认，推荐）

```cmake
cmake .. -DUNICALIB_WITH_PANGOLIN=ON

# 编译 src/ikalibr/viewer/viewer.cpp 和 viewer_pangolin.cpp
# 依赖项：OpenGL, X11（通常系统已有）
# 可用函数：
#   - show_trajectory()：交互式轨迹显示
#   - show_optimization()：收敛曲线实时绘制
#   - show_point_cloud()：点云显示
```

### 场景 2: RViz（ROS 环境）

```bash
# 仅在 ROS2 环境有效
source /opt/ros/humble/setup.bash

cmake .. \
    -DUNICALIB_WITH_ROS2=ON \
    -DUNICALIB_WITH_ROS_RVIZ=ON \
    -DUNICALIB_WITH_PANGOLIN=OFF

# 编译 src/viz/ros_rviz_visualizer.cpp
# 依赖项：rclcpp, visualization_msgs
# 数据流：unicalib → ROS Topic → RViz 显示
```

### 场景 3: Headless（服务器/云端）

```bash
cmake .. \
    -DUNICALIB_WITH_PANGOLIN=OFF \
    -DUNICALIB_WITH_ROS_RVIZ=OFF

# 编译 src/viz/viewer_stub.cpp（空实现）
# 输出：标定结果写入 YAML/CSV 文件
# 用途：CI/CD 流水线、云端标定服务
```

---

## 完整CMakeLists.txt 片段

### 整合后的关键部分（共 50 行改动）

```cmake
# ====== 行 42-54：选项 ======
option(UNICALIB_WITH_IKALIBR "..." ON)
option(UNICALIB_WITH_PANGOLIN "..." ON)
option(UNICALIB_WITH_ROS_RVIZ "..." OFF)

# ====== 行 155-170：basalt-headers 配置 ======
set(BASALT_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers)
if(NOT EXISTS "${BASALT_HEADERS_DIR}/include/basalt/spline/se3_spline.h")
    message(FATAL_ERROR "basalt-headers not found")
endif()
add_library(basalt_headers INTERFACE)
target_include_directories(basalt_headers INTERFACE
    ${BASALT_HEADERS_DIR}/include)
target_link_libraries(basalt_headers INTERFACE Eigen3::Eigen Sophus::Sophus)
add_library(basalt::headers ALIAS basalt_headers)

# ====== 行 472-550：源文件收集重构 ======
if(UNICALIB_WITH_IKALIBR)
    # ... 检查依赖 ...
    set(_UNICALIB_SOURCES 
        ${IKALIBR_CORE_SOURCES} 
        ${IKALIBR_LOADER_SOURCES} 
        ${UNICALIB_NEW_SOURCES} 
        ${_VIEWER_SOURCES})
    add_library(unicalib SHARED ${_UNICALIB_SOURCES})
    target_link_libraries(unicalib PUBLIC basalt::headers Ceres::ceres)
else()
    set(_UNICALIB_SOURCES ${UNICALIB_NEW_SOURCES} ${IKALIBR_MINIMAL_FOR_IMU_LIDAR})
    add_library(unicalib SHARED ${_UNICALIB_SOURCES})
endif()

# 通用链接
target_link_libraries(unicalib PUBLIC
    Eigen3::Eigen Sophus::Sophus basalt::headers
    magic_enum::magic_enum cereal::cereal veta::veta tiny_viewer::stub
    opencv_core opencv_imgproc opencv_highgui)

# ====== 可视化后端链接 ======
if(UNICALIB_WITH_PANGOLIN)
    target_link_libraries(unicalib PUBLIC pangolin)
endif()
```

---

## 构建验证

### 测试编译命令

```bash
#!/bin/bash
# build_all_variants.sh

REPO_DIR="$HOME/Documents/github/UniCalib"
BUILD_DIR="$REPO_DIR/build_variants"

mkdir -p "$BUILD_DIR"

# 定义编译变种
declare -a VARIANTS=(
    "IKALIBR_ON_PANGOLIN_ON"
    "IKALIBR_ON_PANGOLIN_OFF"
    "IKALIBR_OFF_PANGOLIN_OFF"
)

for VARIANT in "${VARIANTS[@]}"; do
    echo "════════════════════════════════════════"
    echo "Building: $VARIANT"
    echo "════════════════════════════════════════"
    
    BUILD_SUBDIR="$BUILD_DIR/$VARIANT"
    mkdir -p "$BUILD_SUBDIR"
    cd "$BUILD_SUBDIR"
    
    case $VARIANT in
        "IKALIBR_ON_PANGOLIN_ON")
            cmake "$REPO_DIR" \
                -DUNICALIB_WITH_IKALIBR=ON \
                -DUNICALIB_WITH_PANGOLIN=ON \
                -DCMAKE_BUILD_TYPE=Release
            ;;
        "IKALIBR_ON_PANGOLIN_OFF")
            cmake "$REPO_DIR" \
                -DUNICALIB_WITH_IKALIBR=ON \
                -DUNICALIB_WITH_PANGOLIN=OFF \
                -DCMAKE_BUILD_TYPE=Release
            ;;
        "IKALIBR_OFF_PANGOLIN_OFF")
            cmake "$REPO_DIR" \
                -DUNICALIB_WITH_IKALIBR=OFF \
                -DUNICALIB_WITH_PANGOLIN=OFF \
                -DCMAKE_BUILD_TYPE=Release
            ;;
    esac
    
    time make -j$(nproc) unicalib
    
    if [ $? -eq 0 ]; then
        echo "✓ $VARIANT successful"
        ls -lh lib/libunicalib.so
    else
        echo "✗ $VARIANT FAILED"
        exit 1
    fi
    echo ""
done

echo "✓ All variants compiled successfully"
```

### 运行验证
```bash
chmod +x build_all_variants.sh
./build_all_variants.sh
```

**预期结果**:
```
IKALIBR_ON_PANGOLIN_ON: 85 MB, 120 sec
IKALIBR_ON_PANGOLIN_OFF: 45 MB, 60 sec
IKALIBR_OFF_PANGOLIN_OFF: 15 MB, 15 sec
```

---

## 故障排查

### 问题 1: 找不到 basalt-headers

**症状**:
```
CMake Error: FATAL_ERROR "basalt-headers not found"
```

**解决**:
```bash
# 验证目录
ls calib_unified/thirdparty/basalt-headers/include/basalt/spline/se3_spline.h

# 若不存在，初始化子模块
cd calib_unified/thirdparty
git submodule update --init --recursive basalt-headers
```

### 问题 2: Ceres 未找到

**症状**:
```
CMake Warning: Ceres not found
```

**解决**:
```bash
sudo apt install libceres-dev

# 或指定路径
cmake .. -DCeres_DIR=/usr/lib/cmake/Ceres
```

### 问题 3: Pangolin 编译失败

**症状**:
```
Error: Pangolin CMakeLists.txt not found
```

**解决**:
```bash
# 验证 Pangolin 源码
ls calib_unified/thirdparty/Pangolin/Pangolin-0.9.0/CMakeLists.txt

# 清理并重新配置
rm -rf build && mkdir build && cd build
cmake ..
```

---

**文档更新**: 2026-03-05  
**下一步**: 执行 [LOCAL_DEPENDENCIES_OPTIMIZATION.md](LOCAL_DEPENDENCIES_OPTIMIZATION.md) 中的详细实施步骤
