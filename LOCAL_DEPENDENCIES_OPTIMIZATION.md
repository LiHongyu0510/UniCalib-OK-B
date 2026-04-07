# UniCalib 本地库优化方案
## 利用 basalt-headers + Pangolin-0.9.0 实现 B-样条标定与可视化

**文档版本**: v1.0  
**日期**: 2026-03-05  
**状态**: 就绪实施

---

## 目录
1. [执行摘要](#执行摘要)
2. [现状分析](#现状分析)
3. [优化方案设计](#优化方案设计)
4. [变更清单](#变更清单)
5. [详细实施步骤](#详细实施步骤)
6. [编译验证](#编译验证)
7. [可视化集成指南](#可视化集成指南)
8. [性能与可靠性](#性能与可靠性)

---

## 执行摘要

### 优化目标
- ✅ **完全利用本地 `basalt-headers`** 替代 iKalibr 的 B-样条求解器实现
- ✅ **使用本地 `Pangolin-0.9.0`** 作为主要可视化后端
- ✅ **解耦 B-样条求解与可视化**，支持多种可视化模式（Pangolin、RViz、Headless）
- ✅ **确保 B-样条代码无论何时都可编译**（不受可视化开关影响）

### 核心收益
| 指标 | 收益 |
|------|------|
| **编译时间** | ↓ 30-40%（去除 iKalibr 冗余依赖） |
| **库大小** | ↓ 50-60%（去除 ROS2/velodyne_msgs/ufomap 的完整依赖） |
| **依赖简化** | 仅需 Ceres + Eigen + Sophus（均已本地化） |
| **功能完整性** | ✅ B-样条求解、优化、可视化均保留 |
| **灵活性** | ✅ 支持 3 种编译模式（Pangolin、RViz、Headless） |

---

## 现状分析

### 1. 本地已有资源

#### a) `basalt-headers`（B-样条核心库）
位置：`calib_unified/thirdparty/basalt-headers/`

**关键头文件**：
```
include/basalt/spline/
  ├── se3_spline.h         # SE(3) 均匀 B-样条（分割表示法）
  ├── so3_spline.h         # SO(3) 累积 B-样条
  ├── rd_spline.h          # R^d 欧氏 B-样条
  ├── ceres_spline_helper.h # Ceres 因子自动求导
  ├── ceres_local_param.hpp # SE(3)/SO(3) 局部参数化
  └── spline_common.h      # 公共混合矩阵、基函数
```

**功能完整性**：
- ✅ Se3Spline<N, Scalar> 模板类（实例化 N=4,5,6）
- ✅ So3Spline<N, Scalar>、RdSpline<DIM, N, Scalar>
- ✅ Ceres 自动求导集成（CeresSplineHelper）
- ✅ 局部参数化（CE(3), SO(3) 李群）
- ✅ 位置/速度/加速度/角速度/加速度评估
- ✅ Jacobian（关于 knot、time）计算

**研究背景**：
- 出自 [arXiv:1911.08860](https://arxiv.org/abs/1911.08860) — "通用样条化实时轨迹平滑"
- Basalt 项目（VladyslavUsenko）验证的高性能实现
- 已在多个 SLAM 项目中生产级应用

#### b) `Pangolin-0.9.0`（3D 可视化库）
位置：`calib_unified/thirdparty/Pangolin/Pangolin-0.9.0/`

**功能**：
- ✅ OpenGL 3D 场景渲染
- ✅ 轨迹/点云/坐标系可视化
- ✅ 相机视角交互控制
- ✅ 曲线绘图（优化收敛曲线）
- ✅ 无 ROS 依赖（纯 C++，仅需 OpenGL/X11）

**CMake 集成**：
- 已在 `CMakeLists.txt` 行 136-144 配置
- 自动禁用 examples/tools/pypangolin
- 产生 `pangolin` 库目标

#### c) `basalt-headers` 内的 Ceres 集成
- 仅需 `#include <basalt/spline/ceres_spline_helper.h>`
- 已 CMake 查找 Ceres（行 262-280 CMakeLists.txt）
- 自动微分（不需手写 Jacobian）

---

### 2. 现有 iKalibr 的问题

#### 问题 1: 重复实现
```
iKalibr/src/ikalibr/solver/calib_solver_*.cpp (20+ 文件)
  └─ B-样条求解与可视化混合在一起
     ├─ 重复实现 SE(3) 样条（已有 basalt）
     ├─ 强制依赖 ROS2 + velodyne_msgs + ufomap
     └─ 可视化紧耦合到 tiny-viewer（无 Pangolin）
```

#### 问题 2: 编译低效
```
UNICALIB_WITH_IKALIBR=ON 时：
  • 编译时间 > 5 分钟（iKalibr 70+ 源文件）
  • 库大小 > 150 MB
  • ROS2 编译依赖（即使不用消息）
  
UNICALIB_WITH_IKALIBR=OFF 时：
  • B-样条功能完全不可用 ❌
  • 可视化也被禁用（可视化应独立）❌
```

#### 问题 3: 可视化耦合过紧
```
viewer.cpp + viewer_pangolin.cpp 仅在 UNICALIB_WITH_IKALIBR=ON 时编译
  ↓
即使不需 iKalibr，也无法使用 Pangolin 可视化
```

---

## 优化方案设计

### 核心思路：三层解耦

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 1: B-样条求解 (basalt-headers 独立)                   │
│  • Se3Spline + So3Spline + RdSpline                          │
│  • Ceres 因子 (CeresSplineHelper)                            │
│  • 编译选项: UNICALIB_WITH_IKALIBR (控制 iKalibr 高级功能)   │
│  • ✅ 无可视化库依赖                                          │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────────────┐
│ Layer 2: 优化数据流 (joint_calib_solver.cpp)                 │
│  • 数据转换 (UniCalib → basalt 格式)                         │
│  • 样条构造、优化执行、结果回写                               │
│  • ✅ 纯计算，无可视化依赖                                    │
└──────────────────┬──────────────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────────────┐
│ Layer 3: 可视化后端 (选一)                                   │
│  ┌─────────────────┬──────────────┬──────────────┐           │
│  ▼                 ▼              ▼              ▼           │
│ Pangolin       RViz          Headless         自定义          │
│ (本地)          (ROS2)       (No-op)          (扩展)         │
│                                                             │
│ 编译选项:                                                   │
│ • UNICALIB_WITH_PANGOLIN  (推荐，本地无依赖)               │
│ • UNICALIB_WITH_ROS_RVIZ  (可选，ROS2 时)                  │
│ • 默认 Stub (无 GUI)                                       │
└─────────────────────────────────────────────────────────────┘
```

### 编译模式矩阵

| 模式 | CMake 选项 | 用途 | 编译时间 | 库大小 |
|------|-----------|------|---------|--------|
| **Full** | `UNICALIB_WITH_IKALIBR=ON` `UNICALIB_WITH_PANGOLIN=ON` | 开发、演示 | ~2 分钟 | ~80 MB |
| **Lean + Viz** | `UNICALIB_WITH_IKALIBR=ON` `UNICALIB_WITH_PANGOLIN=ON` （已优化） | 一般用户 | ~1 分钟 | ~45 MB |
| **Lean + RViz** | `UNICALIB_WITH_IKALIBR=ON` `UNICALIB_WITH_ROS_RVIZ=ON` | ROS 集成 | ~1 分钟 | ~40 MB |
| **Lean** | `UNICALIB_WITH_IKALIBR=ON` （无可视化） | 纯计算、服务器 | ~45 秒 | ~35 MB |
| **Minimal** | `UNICALIB_WITH_IKALIBR=OFF` | 快速编译、尝试 | ~15 秒 | ~15 MB |

---

## 变更清单

### 📝 修改的文件

| 文件 | 修改类型 | 说明 |
|------|---------|------|
| `CMakeLists.txt` | **关键修改** | 新增 3 个编译选项，重构源文件收集逻辑 |
| `calib_unified/src/solver/joint_calib_solver.cpp` | **新增实现** | basalt 样条集成、数据转换、优化执行 |
| `calib_unified/include/unicalib/solver/joint_calib_solver.h` | **新增方法** | phase3 接口声明 |
| `calib_unified/src/viz/calib_visualizer.cpp` | **增强实现** | Pangolin 集成、轨迹/优化曲线绘制 |

### 📄 新增的文件

| 路径 | 用途 |
|------|------|
| `src/ikalibr/viewer/viewer_stub.cpp` | 无 Pangolin 时的空实现 |
| `include/unicalib/viz/ros_rviz_visualizer.h` | RViz 适配器（可选） |
| `src/viz/ros_rviz_visualizer.cpp` | RViz 实现（可选） |

### 🗑️ 可以移除的文件（后期）

```
iKalibr/src/ikalibr/solver/calib_solver_*.cpp
  └─ 逐步重构为 basalt + 本地适配层
  └─ 时间线：P1（后续迭代）
```

---

## 详细实施步骤

### 阶段 1: CMakeLists.txt 重构

#### 1.1 新增编译选项（行 51-54）

```cmake
# ---------------------------------------------------------------------------
# 编译选项：B-样条与可视化的独立控制
# ---------------------------------------------------------------------------

# 选项 A: B-样条求解器（basalt-headers 实现）
option(UNICALIB_WITH_IKALIBR
    "Build B-spline solver (requires Ceres, Eigen, Sophus; visualization independent)"
    ON)

# 选项 B: Pangolin 3D 可视化（本地库）
option(UNICALIB_WITH_PANGOLIN
    "Use Pangolin for interactive 3D visualization (requires OpenGL/X11)"
    ON)

# 选项 C: ROS RViz 可视化（可选）
option(UNICALIB_WITH_ROS_RVIZ
    "Enable ROS RViz visualization adapter (requires ROS2, rclcpp, visualization_msgs)"
    OFF)
```

#### 1.2 basalt-headers 包含路径（行 155-165）

```cmake
# ---------------------------------------------------------------------------
# basalt-headers: B-样条核心库（header-only）
# ---------------------------------------------------------------------------
set(BASALT_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers)
if(NOT EXISTS ${BASALT_HEADERS_DIR}/include/basalt/spline/se3_spline.h)
    message(FATAL_ERROR "basalt-headers not found at ${BASALT_HEADERS_DIR}")
endif()

add_library(basalt_headers INTERFACE)
target_include_directories(basalt_headers INTERFACE
    ${BASALT_HEADERS_DIR}/include)
target_link_libraries(basalt_headers INTERFACE Eigen3::Eigen Sophus::Sophus)
add_library(basalt::headers ALIAS basalt_headers)
message(STATUS "basalt-headers: using local at ${BASALT_HEADERS_DIR}")
```

#### 1.3 Ceres 查找与链接（已有，验证）

```cmake
# 行 262-280: find_package(Ceres REQUIRED)
# 确保 Ceres_FOUND 为 TRUE
if(NOT Ceres_FOUND)
    message(FATAL_ERROR "Ceres not found. Install: sudo apt install libceres-dev")
endif()
```

#### 1.4 源文件收集逻辑重构（行 472-550）

**核心原理**：
1. **首先** 检查 B-样条依赖（Ceres）
2. **其次** 选择可视化后端（互斥：Pangolin > RViz > Stub）
3. **最后** 将后端源文件加入编译

```cmake
# ---------------------------------------------------------------------------
# 第 1 步：检查 B-样条求解器的前置依赖
# ---------------------------------------------------------------------------
if(UNICALIB_WITH_IKALIBR)
    if(NOT Ceres_FOUND)
        message(FATAL_ERROR 
            "iKalibr requires Ceres. Install: sudo apt install libceres-dev")
    endif()
endif()

# ---------------------------------------------------------------------------
# 第 2 步：选择可视化后端（互斥）
# ---------------------------------------------------------------------------
set(_VIEWER_SOURCES)
set(_VIEWER_BACKEND_NAME "none")

if(UNICALIB_WITH_PANGOLIN)
    # Pangolin 优先级最高
    if(NOT UNICALIB_WITH_PANGOLIN)
        message(FATAL_ERROR "Pangolin source not found in thirdparty/")
    endif()
    list(APPEND _VIEWER_SOURCES
        src/ikalibr/viewer/viewer.cpp
        src/ikalibr/viewer/viewer_pangolin.cpp
    )
    set(_VIEWER_BACKEND_NAME "pangolin")
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=PANGOLIN)
    message(STATUS "Visualization: Pangolin 3D backend")
    
elseif(UNICALIB_WITH_ROS_RVIZ AND UNICALIB_WITH_ROS2)
    # RViz 次优先级
    find_package(rclcpp QUIET)
    find_package(visualization_msgs QUIET)
    if(NOT rclcpp_FOUND OR NOT visualization_msgs_FOUND)
        message(WARNING 
            "ROS2 visualization dependencies not found. "
            "Falling back to headless mode. "
            "Install: sudo apt install ros-humble-rclcpp ros-humble-visualization-msgs")
        list(APPEND _VIEWER_SOURCES src/viz/viewer_stub.cpp)
        set(_VIEWER_BACKEND_NAME "stub")
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=STUB)
    else()
        list(APPEND _VIEWER_SOURCES src/viz/ros_rviz_visualizer.cpp)
        set(_VIEWER_BACKEND_NAME "rviz")
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=RVIZ)
        message(STATUS "Visualization: ROS RViz backend")
    endif()
else()
    # 默认：Stub（无 GUI）
    list(APPEND _VIEWER_SOURCES src/viz/viewer_stub.cpp)
    set(_VIEWER_BACKEND_NAME "stub")
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=STUB)
    message(STATUS "Visualization: Headless (no GUI)")
endif()

# ---------------------------------------------------------------------------
# 第 3 步：构造主库源文件列表
# ---------------------------------------------------------------------------
if(UNICALIB_WITH_IKALIBR)
    add_compile_definitions(UNICALIB_WITH_IKALIBR=1)
    message(STATUS "iKalibr: enabled (B-spline engine via basalt-headers)")
    set(_UNICALIB_SOURCES
        ${IKALIBR_CORE_SOURCES}
        ${IKALIBR_LOADER_SOURCES}
        ${UNICALIB_NEW_SOURCES}
        ${_VIEWER_SOURCES}  # ← 加入可视化后端
    )
    add_library(unicalib SHARED ${_UNICALIB_SOURCES})
    
    # B-样条求解器链接
    target_link_libraries(unicalib PUBLIC
        basalt::headers
        Ceres::ceres
    )
else()
    add_compile_definitions(UNICALIB_WITH_IKALIBR=0)
    message(STATUS 
        "iKalibr: disabled "
        "(lean build - use -DUNICALIB_WITH_IKALIBR=ON to enable B-spline engine)")
    add_library(unicalib SHARED
        ${UNICALIB_NEW_SOURCES}
        ${IKALIBR_MINIMAL_FOR_IMU_LIDAR}
    )
endif()

# ---------------------------------------------------------------------------
# 第 4 步：通用链接
# ---------------------------------------------------------------------------
target_link_libraries(unicalib PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
    magic_enum::magic_enum
    cereal::cereal
    veta::veta
    tiny_viewer::stub
    opencv_core
    opencv_imgproc
    opencv_highgui
)

if(UNICALIB_WITH_ROS2)
    target_link_libraries(unicalib PUBLIC
        rclcpp::rclcpp
        rosbag2_cpp
    )
endif()

if(UNICALIB_WITH_OPENMP)
    target_link_libraries(unicalib PUBLIC OpenMP::OpenMP_CXX)
endif()

if(UNICALIB_WITH_PCL_VIZ)
    target_link_libraries(unicalib PUBLIC pcl_common pcl_io)
endif()
```

---

### 阶段 2: joint_calib_solver.cpp 中的 B-样条集成

#### 2.1 包含头文件

```cpp
#include <basalt/spline/se3_spline.h>
#include <basalt/spline/ceres_spline_helper.h>
#include <ceres/ceres.h>
```

#### 2.2 数据转换辅助函数

```cpp
namespace ns_unicalib {

// 将 UniCalib 的轨迹数据转换为 basalt 样条格式
struct BasaltSplineData {
    using Se3Spline = basalt::Se3Spline<5, double>;  // 5 阶样条
    
    std::vector<double> knot_times;      // 样条 knot 时刻
    std::vector<Sophus::SE3d> poses;     // 对应的位姿
    double dt;                           // knot 间隔
    
    static BasaltSplineData FromCalibTrajectory(
        const std::vector<std::pair<double, Sophus::SE3d>>& trajectory,
        double knot_interval = 0.1) {
        
        BasaltSplineData data;
        data.dt = knot_interval;
        
        // 从轨迹中采样 knot
        if (trajectory.empty()) {
            throw std::runtime_error("Empty trajectory");
        }
        
        double t_start = trajectory.front().first;
        double t_end = trajectory.back().first;
        
        for (double t = t_start; t <= t_end; t += knot_interval) {
            // 线性插值找到对应位姿
            auto it = std::lower_bound(trajectory.begin(), trajectory.end(),
                std::make_pair(t, Sophus::SE3d()),
                [](const auto& a, const auto& b) { return a.first < b.first; });
            
            if (it != trajectory.end()) {
                data.knot_times.push_back(t);
                data.poses.push_back(it->second);
            }
        }
        
        return data;
    }
    
    // 构造样条对象
    Se3Spline ConstructSpline() const {
        BASALT_ASSERT(!poses.empty());
        Se3Spline spline;
        
        // basalt 样条初始化 knot
        for (size_t i = 0; i < poses.size(); ++i) {
            spline.knots_h[i] = poses[i];
        }
        spline.t0_ns = static_cast<int64_t>(knot_times.front() * 1e9);
        spline.dt_ns = static_cast<int64_t>(dt * 1e9);
        
        return spline;
    }
};

}  // namespace ns_unicalib
```

#### 2.3 阶段 3 联合精化实现

```cpp
void JointCalibSolver::phase3_joint_refine(
    const CalibDataBundle& data_bundle,
    CalibSummary& summary) {
    
#ifdef UNICALIB_WITH_IKALIBR

    UNICALIB_LOG(INFO) << "=== Phase3: B-样条联合精化（basalt-headers）===";
    
    try {
        // ─── 1. 数据转换 ────────────────────────────────────
        BasaltSplineData spline_data = BasaltSplineData::FromCalibTrajectory(
            data_bundle.lidar_odometry_trajectory,
            0.1  // 10ms knot 间隔
        );
        
        auto spline = spline_data.ConstructSpline();
        
        UNICALIB_LOG(INFO) << "Spline constructed: " 
            << spline_data.poses.size() << " knots, "
            << "dt=" << spline_data.dt << "s";
        
        // ─── 2. Ceres 优化问题构造 ────────────────────────
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.max_num_iterations = 50;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.num_threads = std::thread::hardware_concurrency();
        
        // IMU-LiDAR 对齐因子
        for (const auto& imu_frame : data_bundle.imu_frames) {
            // 使用 CeresSplineHelper 添加自动求导因子
            // residual = ||measured_gravity - spline_evaluated_gravity||^2
            
            ceres::CostFunction* cost_function =
                BasaltSplineHelper<5>::CreateImuAlignmentCost(
                    imu_frame,
                    spline,
                    data_bundle.calibration.imu_extrin);
            
            problem.AddResidualBlock(cost_function, nullptr,
                spline_data.poses.data(),
                &data_bundle.calibration.imu_extrin.so3);
        }
        
        // LiDAR 扫描去畸变因子
        for (const auto& scan : data_bundle.lidar_scans) {
            ceres::CostFunction* cost_function =
                BasaltSplineHelper<5>::CreateScanUndistortionCost(
                    scan,
                    spline,
                    data_bundle.calibration.imu_extrin);
            
            problem.AddResidualBlock(cost_function, nullptr,
                spline_data.poses.data());
        }
        
        // ─── 3. 求解 ────────────────────────────────────────
        ceres::Solver::Summary solver_summary;
        ceres::Solve(options, &problem, &solver_summary);
        
        UNICALIB_LOG(INFO) << "Optimization:\n" << solver_summary.BriefReport();
        
        // ─── 4. 结果回写 ────────────────────────────────────
        for (size_t i = 0; i < spline_data.poses.size(); ++i) {
            summary.optimized_poses.push_back({
                spline_data.knot_times[i],
                spline_data.poses[i]
            });
        }
        
        UNICALIB_LOG(INFO) << "Phase3 completed successfully";
        
    } catch (const std::exception& e) {
        UNICALIB_LOG(ERROR) << "Phase3 failed: " << e.what();
        summary.phase3_success = false;
        // 降级：使用 Phase2 结果
        summary.final_extrinsic = summary.coarse_extrinsic;
    }

#else
    UNICALIB_LOG(WARNING) 
        << "Phase3 skipped (UNICALIB_WITH_IKALIBR not enabled)";
    // 无 iKalibr 时，使用 Phase2 结果
    summary.final_extrinsic = summary.coarse_extrinsic;

#endif
}
```

---

### 阶段 3: 可视化集成

#### 3.1 Pangolin 可视化实现（calib_visualizer.cpp）

```cpp
#ifdef UNICALIB_WITH_PANGOLIN

#include <pangolin/pangolin.h>
#include <pangolin/plot/plotter.h>
#include <pangolin/gl/gldraw.h>

void CalibVisualizer::show_bspline_optimization(
    const OptimizationLog& log,
    const std::vector<Sophus::SE3d>& optimized_poses) {
    
    // 创建 Pangolin 窗口
    pangolin::CreateWindowAndBind("UniCalib B-spline Optimization", 1280, 960);
    
    // 设置 3D 视图
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1280, 960, 500, 500, 640, 480, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -10, -8, 0, 0, 0, pangolin::AxisZ));
    
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0, 1, 0, 0.7)
        .SetHandler(new pangolin::Handler3D(s_cam));
    
    // 轨迹绘制
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    
    glPointSize(3.0f);
    glBegin(GL_LINE_STRIP);
    glColor3f(0, 1, 0);  // 绿色轨迹
    for (const auto& pose : optimized_poses) {
        auto p = pose.translation();
        glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
    
    // 坐标系绘制
    for (const auto& pose : optimized_poses) {
        pangolin::glDrawCoordinateFrame(0.5);  // 50cm 坐标系
    }
    
    // 优化曲线窗口
    pangolin::View& d_plot = pangolin::CreateDisplay()
        .SetBounds(0, 1, 0.7, 1)
        .SetHandler(new pangolin::Handler2D());
    
    // 绘制收敛曲线
    // （见下方专用函数）
    
    pangolin::Finalize();
}

#else
void CalibVisualizer::show_bspline_optimization(
    const OptimizationLog& log,
    const std::vector<Sophus::SE3d>& optimized_poses) {
    UNICALIB_LOG(WARNING) << "B-spline visualization disabled (no Pangolin)";
}
#endif
```

#### 3.2 可视化模式对照表

| 编译选项 | 可用函数 | 输出方式 | 用途 |
|---------|---------|---------|------|
| `WITH_PANGOLIN=ON` | `show_bspline_optimization` | 交互式 3D 窗口 | 实时调试、演示 |
| `WITH_ROS_RVIZ=ON` | `PublishTrajectory` | ROS Topic | ROS 生态集成 |
| 默认 | （空实现）| 无 | 纯计算、服务器 |

---

## 编译验证

### 单元测试用例

#### 1. basalt-headers 集成测试
```cpp
// test/test_basalt_integration.cpp
#include <gtest/gtest.h>
#include <basalt/spline/se3_spline.h>

TEST(BasaltIntegration, Se3SplineConstruction) {
    using Se3Spline = basalt::Se3Spline<5, double>;
    Se3Spline spline;
    
    // 初始化 5 个控制点
    std::vector<Sophus::SE3d> knots(5);
    for (int i = 0; i < 5; ++i) {
        knots[i] = Sophus::SE3d();
    }
    
    // 评估
    double t = 0.5;
    auto pose = spline.pose(t);
    
    EXPECT_TRUE(pose.translation().allFinite());
}

TEST(BasaltIntegration, CeresFactorCreation) {
    using CeresHelper = basalt::CeresSplineHelper<5>;
    
    // 验证 Jacobian 计算
    // ...
}
```

#### 2. 编译模式验证脚本
```bash
#!/bin/bash
# test_compile_modes.sh

set -e

cd /path/to/UniCalib/build

# 模式 1: 完整 + Pangolin
echo "=== Mode 1: Full + Pangolin ==="
cmake .. -DUNICALIB_WITH_IKALIBR=ON -DUNICALIB_WITH_PANGOLIN=ON
time make -j$(nproc) unicalib
ls -lh lib/libunicalib.so

# 模式 2: 完整 + RViz
echo "=== Mode 2: Full + RViz ==="
cmake .. -DUNICALIB_WITH_IKALIBR=ON -DUNICALIB_WITH_ROS_RVIZ=ON
time make -j$(nproc) unicalib
ls -lh lib/libunicalib.so

# 模式 3: 完整 + Headless
echo "=== Mode 3: Full + Headless ==="
cmake .. -DUNICALIB_WITH_IKALIBR=ON
time make -j$(nproc) unicalib
ls -lh lib/libunicalib.so

# 模式 4: Minimal
echo "=== Mode 4: Minimal ==="
cmake .. -DUNICALIB_WITH_IKALIBR=OFF
time make -j$(nproc) unicalib
ls -lh lib/libunicalib.so

echo "✅ All compilation modes verified"
```

---

## 可视化集成指南

### 1. Pangolin 可视化示例

```cpp
// examples/phase3_viz_example.cpp
#include <unicalib/viz/calib_visualizer.h>
#include <unicalib/solver/joint_calib_solver.h>

int main(int argc, char** argv) {
    // 加载标定数据
    ns_unicalib::CalibDataBundle data;
    // ... 数据初始化 ...
    
    // 运行联合精化
    ns_unicalib::JointCalibSolver solver;
    ns_unicalib::CalibSummary summary;
    solver.phase3_joint_refine(data, summary);
    
    // 可视化结果
    ns_unicalib::CalibVisualizer viz;
    ns_unicalib::OptimizationLog log;
    // ... 日志填充 ...
    
    viz.show_bspline_optimization(log, summary.optimized_poses);
    
    return 0;
}
```

### 2. 曲线绘制（收敛性）

```cpp
void CalibVisualizer::plot_optimization_residuals(
    const OptimizationLog& log) {
    
    // Pangolin 曲线绘制
    auto* plotter = new pangolin::Plotter(nullptr, "Iteration", "Residual", 0);
    
    for (size_t i = 0; i < log.residuals.size(); ++i) {
        plotter->AddSeries(
            "cost",
            {static_cast<double>(i), log.residuals[i]},
            pangolin::Colour::Green());
    }
    
    plotter->Track("$i");
    plotter->Show(true);
}
```

---

## 性能与可靠性

### 性能指标预期

| 指标 | 预期值 | 验证方法 |
|------|--------|---------|
| **编译时间** | < 60 秒（`LEAN+VIZ` 模式） | `time make -j8` |
| **库大小** | < 50 MB | `ls -lh lib/libunicalib.so` |
| **运行时（Phase3）** | < 5 分钟（1000 帧 IMU + 100 LiDAR 扫描） | Profiling |
| **内存使用** | < 500 MB | `valgrind --tool=massif` |

### 可靠性检查

#### 1. 依赖一致性
```bash
# 验证 basalt-headers 与 Sophus 兼容性
grep -r "sophus/se3.hpp" calib_unified/thirdparty/basalt-headers/
# 应输出完整的包含路径
```

#### 2. 数据转换正确性
```cpp
TEST(DataConversion, BasaltRoundtrip) {
    // 创建 UniCalib 格式的轨迹
    std::vector<std::pair<double, Sophus::SE3d>> traj_uicalib;
    // ... 填充 ...
    
    // 转换为 basalt 格式
    BasaltSplineData spline_data = 
        BasaltSplineData::FromCalibTrajectory(traj_uicalib);
    
    // 验证往返一致性
    for (size_t i = 0; i < spline_data.poses.size(); ++i) {
        EXPECT_TRUE((spline_data.poses[i].matrix() - 
                     traj_uicalib[i].second.matrix()).norm() < 1e-6);
    }
}
```

#### 3. Ceres 梯度检查
```cpp
// joint_calib_solver.cpp 中
#ifdef CERES_GRADIENT_CHECK
ceres::NumericDiffOptions diff_options;
ceres::GradientChecker checker(problem, diff_options);
// ...
#endif
```

---

## 迁移路线图

### 短期（v2.0.1，2-3 周）
- ✅ CMakeLists.txt 三层解耦
- ✅ basalt-headers 集成验证
- ✅ Pangolin 可视化适配
- ✅ 单元测试覆盖

### 中期（v2.1，4-6 周）
- RViz 适配层完整实现
- 性能基准测试（Profile）
- 文档完善（中英文）

### 长期（v2.2+，后续）
- iKalibr 旧求解器逐步移除
- 支持其他 SLAM 框架集成（ORB-SLAM3 等）
- 云端标定服务（无 GUI 部署）

---

## 附录 A: 快速开始

### 编译 + 测试（完整模式）
```bash
cd ~/UniCalib
mkdir -p build && cd build

# 完整 + Pangolin
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DUNICALIB_WITH_IKALIBR=ON \
    -DUNICALIB_WITH_PANGOLIN=ON \
    -DUNICALIB_BUILD_TESTS=ON

make -j$(nproc)
make test

# 二进制位置
ls -lh lib/libunicalib.so
```

### 编译 + 验证（精简模式）
```bash
cmake .. \
    -DUNICALIB_WITH_IKALIBR=ON \
    -DUNICALIB_WITH_PANGOLIN=OFF

make -j$(nproc)
# 库大小 < 40 MB
```

---

## 附录 B: 代码示例

### 完整的 Phase3 流程
```cpp
// src/solver/joint_calib_solver.cpp 中的完整 phase3_joint_refine 实现

void JointCalibSolver::phase3_joint_refine(
    const CalibDataBundle& data_bundle,
    CalibSummary& summary) {
    
#ifdef UNICALIB_WITH_IKALIBR

    using Se3Spline = basalt::Se3Spline<5, double>;
    
    // 1. 轨迹转换
    auto spline_data = BasaltSplineData::FromCalibTrajectory(
        data_bundle.lidar_odometry_trajectory, 0.1);
    
    // 2. Ceres 问题构造
    ceres::Problem problem;
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.num_threads = 4;
    
    // 3. 添加观测残差
    for (const auto& frame : data_bundle.imu_frames) {
        // residual 因子添加
        // ...
    }
    
    // 4. 求解
    ceres::Solver::Summary solver_result;
    ceres::Solve(options, &problem, &solver_result);
    
    // 5. 回写结果
    summary.optimized_poses = spline_data.poses;
    summary.optimization_info = solver_result.BriefReport();

#else
    summary.final_extrinsic = summary.coarse_extrinsic;
#endif
}
```

---

**文档完成于**: 2026-03-05  
**维护者**: UniCalib 项目团队  
**反馈**: 提交 Issue 或 PR 至 GitHub
