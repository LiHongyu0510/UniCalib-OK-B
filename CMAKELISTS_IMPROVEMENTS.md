# CMakeLists.txt 改进补丁

## 改动摘要

分离 B 样条编译选项（`UNICALIB_WITH_IKALIBR`）和可视化后端选择（`UNICALIB_WITH_PANGOLIN`、`UNICALIB_WITH_ROS_RVIZ`）

---

## 差异对比

### 位置: CMakeLists.txt 第 42-54 行

**旧版本**:
```cmake
# ---------------------------------------------------------------------------
# 选项
# ---------------------------------------------------------------------------
option(UNICALIB_BUILD_APPS      "Build application executables"        ON)
option(UNICALIB_WITH_ROS2       "Enable optional ROS2 bag reading"     OFF)
option(UNICALIB_WITH_PCL_VIZ    "Enable PCL visualizer"                ON)
option(UNICALIB_WITH_OPENMP     "Enable OpenMP parallelism"            ON)
option(UNICALIB_WITH_PYTHON_BRIDGE "Enable AI coarse calib (Python)"   ON)
option(UNICALIB_BUILD_TESTS      "Build unit tests"                     OFF)
# iKalibr B-spline engine: requires velodyne_msgs, full ROS2, ufomap, opengv.
# Default ON; set OFF for quick lean builds (e.g. -DUNICALIB_WITH_IKALIBR=OFF).
option(UNICALIB_WITH_IKALIBR    "Build iKalibr B-spline calibration engine" ON)
option(BUILD_TESTING            "Build tests"                          OFF)
```

**新版本**:
```cmake
# ---------------------------------------------------------------------------
# 选项
# ---------------------------------------------------------------------------
option(UNICALIB_BUILD_APPS      "Build application executables"        ON)
option(UNICALIB_WITH_ROS2       "Enable optional ROS2 bag reading"     OFF)
option(UNICALIB_WITH_PCL_VIZ    "Enable PCL visualizer"                ON)
option(UNICALIB_WITH_OPENMP     "Enable OpenMP parallelism"            ON)
option(UNICALIB_WITH_PYTHON_BRIDGE "Enable AI coarse calib (Python)"   ON)
option(UNICALIB_BUILD_TESTS      "Build unit tests"                     OFF)

# B-spline 核心求解器 (依赖: Ceres, opengv, ufomap, ctraj, basalt)
# 可独立于可视化后端编译
option(UNICALIB_WITH_IKALIBR    "Build iKalibr B-spline solver (Phase 3)" ON)

# 可视化后端选择 (互斥)
# - PANGOLIN: 本地 3D GUI (requires Pangolin library)
# - ROS_RVIZ: ROS 环境 (requires ROS2 + tf2)
# - OFF: 无 GUI，仅输出文件
option(UNICALIB_WITH_PANGOLIN   "Use Pangolin for 3D GUI visualization" ON)
option(UNICALIB_WITH_ROS_RVIZ   "Use ROS RViz for visualization"       OFF)

option(BUILD_TESTING            "Build tests"                          OFF)
```

---

### 位置: CMakeLists.txt 第 356-491 行

**旧版本**:
```cmake
# 收集 iKalibr 核心源文件
set(IKALIBR_CORE_SOURCES ...)
set(IKALIBR_LOADER_SOURCES ...)
set(UNICALIB_NEW_SOURCES ...)
set(IKALIBR_MINIMAL_FOR_IMU_LIDAR ...)

# iKalibr sources are optional
if(UNICALIB_WITH_IKALIBR)
    add_compile_definitions(UNICALIB_WITH_IKALIBR=1)
    message(STATUS "iKalibr: enabled (full B-spline engine)")
    set(_UNICALIB_SOURCES ${IKALIBR_CORE_SOURCES} ${IKALIBR_LOADER_SOURCES} ${UNICALIB_NEW_SOURCES})
    if(UNICALIB_WITH_PANGOLIN)
      list(APPEND _UNICALIB_SOURCES
        src/ikalibr/viewer/viewer.cpp
        src/ikalibr/viewer/viewer_pangolin.cpp
      )
    endif()
    add_library(unicalib SHARED ${_UNICALIB_SOURCES})
else()
    message(STATUS "iKalibr: disabled (lean build)")
    add_compile_definitions(UNICALIB_WITH_IKALIBR=0)
    add_library(unicalib SHARED
        ${UNICALIB_NEW_SOURCES}
        ${IKALIBR_MINIMAL_FOR_IMU_LIDAR}
    )
endif()
```

**新版本**:
```cmake
# 收集 iKalibr 源文件 (分层设计)
set(IKALIBR_CORE_SOURCES ...)       # 核心算法 + 求解器
set(IKALIBR_LOADER_SOURCES ...)     # 数据加载器
set(UNICALIB_NEW_SOURCES ...)       # UniCalib 新增接口
set(IKALIBR_MINIMAL_FOR_IMU_LIDAR ...) # 轻量化 IMU-LiDAR (utils, lidar, odometer)

# ===================================================================
# 第 1 步: 验证 B 样条编译前置条件
# ===================================================================
if(UNICALIB_WITH_IKALIBR)
    # 检查必需的依赖 (iKalibr 求解器需要的库)
    find_package(Ceres QUIET)
    if(NOT Ceres_FOUND)
        message(FATAL_ERROR 
            "Ceres not found, required for B-spline solver. "
            "Either install Ceres or disable with -DUNICALIB_WITH_IKALIBR=OFF")
    endif()
    
    # opengv, ufomap, ctraj, basalt 的检查(已有，保持不变)
    # ...
endif()

# ===================================================================
# 第 2 步: 选择编译的源文件组合
# ===================================================================

set(_VIEWER_SOURCES)  # 可视化后端源文件

# 验证可视化后端选择的互斥性
if(UNICALIB_WITH_PANGOLIN AND UNICALIB_WITH_ROS_RVIZ)
    message(WARNING "Both PANGOLIN and ROS_RVIZ are ON, using PANGOLIN as primary")
    set(UNICALIB_WITH_ROS_RVIZ OFF)
endif()

# 选择可视化后端
if(UNICALIB_WITH_IKALIBR)
    if(UNICALIB_WITH_PANGOLIN)
        # Pangolin 3D GUI
        message(STATUS "Viewer backend: Pangolin (local 3D GUI)")
        list(APPEND _VIEWER_SOURCES
            src/ikalibr/viewer/viewer.cpp
            src/ikalibr/viewer/viewer_pangolin.cpp
        )
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=IKALIBR_VIEWER_PANGOLIN)
        
    elseif(UNICALIB_WITH_ROS_RVIZ)
        # ROS RViz (需要 ROS2)
        if(NOT rclcpp_FOUND)
            message(WARNING "ROS2 not found, disabling RViz visualization")
            set(UNICALIB_WITH_ROS_RVIZ OFF)
        else()
            message(STATUS "Viewer backend: ROS RViz (Markers + TF2)")
            list(APPEND UNICALIB_NEW_SOURCES
                src/viz/ros_rviz_visualizer.cpp
            )
            add_compile_definitions(IKALIBR_VIEWER_BACKEND=IKALIBR_VIEWER_ROS_RVIZ)
        endif()
        
    else()
        # 无 GUI Stub (轻量级后端)
        message(STATUS "Viewer backend: None (stub implementation)")
        list(APPEND _VIEWER_SOURCES
            src/ikalibr/viewer/viewer_stub.cpp
        )
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=IKALIBR_VIEWER_STUB)
    endif()
endif()

# ===================================================================
# 第 3 步: 组合最终编译源文件集合
# ===================================================================

if(UNICALIB_WITH_IKALIBR)
    # B 样条 + 选择的可视化后端
    message(STATUS "iKalibr: ENABLED (B-spline solver + selected viewer backend)")
    add_compile_definitions(UNICALIB_WITH_IKALIBR=1)
    
    set(_UNICALIB_SOURCES
        ${IKALIBR_CORE_SOURCES}
        ${IKALIBR_LOADER_SOURCES}
        ${_VIEWER_SOURCES}
        ${UNICALIB_NEW_SOURCES}
    )
else()
    # 精简版 (仅 Phase 1-2)
    message(STATUS "iKalibr: DISABLED (lean build, Phase 1-2 only)")
    add_compile_definitions(UNICALIB_WITH_IKALIBR=0)
    
    set(_UNICALIB_SOURCES
        ${IKALIBR_MINIMAL_FOR_IMU_LIDAR}
        ${UNICALIB_NEW_SOURCES}
    )
endif()

# 添加库
add_library(unicalib SHARED ${_UNICALIB_SOURCES})
```

---

## 头文件新增（可选）

**文件**: `include/unicalib/viz/ros_rviz_visualizer.h`

```cpp
#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ns_unicalib {

/**
 * @brief ROS RViz 可视化适配层
 * 将 iKalibr 标定结果发布为 ROS 消息
 * 
 * 用法:
 *   auto rviz = std::make_shared<RosRvizVisualizer>(ros_node);
 *   rviz->PublishTrajectory("map", poses, "lidar_odometry");
 *   rviz->PublishCoordinateFrame(T_target_in_ref, "ref_frame", "target_frame");
 */
class RosRvizVisualizer {
public:
    using Ptr = std::shared_ptr<RosRvizVisualizer>;
    
    explicit RosRvizVisualizer(rclcpp::Node::SharedPtr node);
    ~RosRvizVisualizer() = default;

    /**
     * @brief 发布轨迹 (点云 + 连线)
     * @param frame_id 参考坐标系
     * @param poses SE3 位姿序列
     * @param namespace_id RViz marker namespace
     */
    void PublishTrajectory(
        const std::string& frame_id,
        const std::vector<Sophus::SE3d>& poses,
        const std::string& namespace_id);

    /**
     * @brief 发布坐标系 (TF2)
     * @param pose SE3 变换
     * @param frame_id 父坐标系
     * @param child_frame_id 子坐标系
     */
    void PublishCoordinateFrame(
        const Sophus::SE3d& pose,
        const std::string& frame_id,
        const std::string& child_frame_id);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace ns_unicalib
```

---

## 使用示例

### ROS 节点集成示例

**文件**: `apps/joint_calib_ros/main.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include "unicalib/solver/joint_calib_solver.h"
#include "unicalib/viz/ros_rviz_visualizer.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::make_shared<rclcpp::Node>("unicalib_joint_calib");
    
    // 运行标定
    ns_unicalib::JointCalibSolver solver;
    auto summary = solver.run();
    
    // 发布到 RViz
    if (summary.success) {
        auto rviz = std::make_shared<ns_unicalib::RosRvizVisualizer>(node);
        
        // 发布结果轨迹
        for (const auto& [ref_id, target_id] : extrinsic_pairs) {
            if (params->extrinsics.count(key)) {
                auto ext = params->extrinsics[key];
                rviz->PublishCoordinateFrame(
                    ext->SE3_TargetInRef(),
                    ref_id,
                    target_id);
            }
        }
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

---

## 测试命令

```bash
# 测试 1: B 样条 + Pangolin
cmake .. -DUNICALIB_WITH_IKALIBR=ON \
         -DUNICALIB_WITH_PANGOLIN=ON \
         -DUNICALIB_WITH_ROS_RVIZ=OFF
make -j

# 测试 2: B 样条 + RViz
cmake .. -DUNICALIB_WITH_IKALIBR=ON \
         -DUNICALIB_WITH_PANGOLIN=OFF \
         -DUNICALIB_WITH_ROS_RVIZ=ON \
         -DUNICALIB_WITH_ROS2=ON
make -j

# 测试 3: B 样条 + 无 GUI
cmake .. -DUNICALIB_WITH_IKALIBR=ON \
         -DUNICALIB_WITH_PANGOLIN=OFF \
         -DUNICALIB_WITH_ROS_RVIZ=OFF
make -j

# 测试 4: 精简版 (无 B 样条)
cmake .. -DUNICALIB_WITH_IKALIBR=OFF \
         -DUNICALIB_WITH_PANGOLIN=OFF \
         -DUNICALIB_WITH_ROS_RVIZ=OFF
make -j
```

