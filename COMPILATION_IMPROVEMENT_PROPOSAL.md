# UniCalib 编译策略改进方案

## 目标

改进当前的编译条件设计，使得用户能够：
1. **编译 B 样条完整功能**（即使 Pangolin 不可用）
2. **灵活选择可视化后端**（Pangolin、RViz、或无 GUI）
3. **支持多种部署场景**（服务器、容器、嵌入式、ROS）

---

## 当前设计的问题

```
现状 (不合理):
├─ UNICALIB_WITH_IKALIBR=ON  → 编译 B 样条 + 必须用 Pangolin
└─ UNICALIB_WITH_IKALIBR=OFF → 跳过 B 样条 + 降级到粗外参
              ↑
           两个概念混淆
```

---

## 改进方案：分离两个独立选项

### 方案对比表

```
模式                  IKALIBR  PANGOLIN  RVIZ   功能                        编译时间
═══════════════════════════════════════════════════════════════════════════════════
精简版                 OFF      OFF      OFF   Phase 1-2 + 粗外参         30-60s
B样条+无GUI            ON       OFF      OFF   Phase 1-4 完整，无可视化   1-2min
B样条+RViz             ON       OFF      ON    Phase 1-4 完整，ROS可视化   2-5min
完整版 (Pangolin)      ON       ON       -     Phase 1-4 完整，GUI可视化   5-15min
═══════════════════════════════════════════════════════════════════════════════════
```

---

## 实现步骤

### Step 1: 修改 CMakeLists.txt 选项定义

**当前**:
```cmake
option(UNICALIB_WITH_IKALIBR "Build iKalibr B-spline engine" ON)
```

**改进后**:
```cmake
# Option 1: 是否编译 B 样条求解器 (独立控制)
option(UNICALIB_WITH_IKALIBR    
    "Build iKalibr B-spline solver (requires Ceres, opengv, ufomap; visualization backend independent)" 
    ON)

# Option 2: 是否使用 Pangolin 可视化 (可选)
option(UNICALIB_WITH_PANGOLIN   
    "Use Pangolin for 3D visualization (requires Pangolin library)" 
    ON)

# Option 3: 是否支持 ROS RViz 可视化 (可选)
option(UNICALIB_WITH_ROS_RVIZ   
    "Enable ROS RViz visualization support (requires ROS)" 
    OFF)
```

---

### Step 2: 分离编译源文件集合

**改进前**:
```cmake
if(UNICALIB_WITH_IKALIBR)
    set(_UNICALIB_SOURCES 
        ${IKALIBR_CORE_SOURCES}      # 79 个文件
        ${IKALIBR_LOADER_SOURCES}    # 6 个文件
        ${UNICALIB_NEW_SOURCES})
    # Pangolin viewer.cpp + viewer_pangolin.cpp 自动包含
else()
    set(_UNICALIB_SOURCES 
        ${IKALIBR_MINIMAL_FOR_IMU_LIDAR}  # 3 个文件
        ${UNICALIB_NEW_SOURCES})
endif()
```

**改进后**:
```cmake
# Step 1: 核心 B 样条求解器 (始终可选)
set(IKALIBR_BSPLINE_SOURCES
    # 求解器核心 (20 个文件)
    src/ikalibr/solver/calib_solver_common_impl.cpp
    src/ikalibr/solver/calib_solver_proc_impl.cpp
    src/ikalibr/solver/calib_solver_da_impl.cpp
    src/ikalibr/solver/calib_solver_bo_impl.cpp
    src/ikalibr/solver/calib_solver_init_so3_spline_impl.cpp
    src/ikalibr/solver/calib_solver_init_scale_spline_impl.cpp
    src/ikalibr/solver/calib_solver_init_sen_inertial_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_li_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ri_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_pci_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_vci_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_di_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_circle_based_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_line_based_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ii_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_bo_impl.cpp
    src/ikalibr/solver/calib_solver_io.cpp
    # 其他核心算法 (50+ 个文件)
    ${IKALIBR_CORE_SOURCES}
    ${IKALIBR_LOADER_SOURCES}
)

# Step 2: 可视化后端 (独立选择)
if(UNICALIB_WITH_PANGOLIN)
    list(APPEND IKALIBR_BSPLINE_SOURCES
        src/ikalibr/viewer/viewer.cpp
        src/ikalibr/viewer/viewer_pangolin.cpp
    )
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=PANGOLIN)
elseif(UNICALIB_WITH_ROS_RVIZ)
    # ROS RViz 可视化模块 (新增)
    list(APPEND UNICALIB_NEW_SOURCES
        src/viz/ros_rviz_visualizer.cpp  # 新增 ROS 适配层
    )
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=ROS_RVIZ)
else()
    # 无 GUI 后端 (stub)
    list(APPEND IKALIBR_BSPLINE_SOURCES
        src/ikalibr/viewer/viewer_stub.cpp  # 空 stub 实现
    )
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=STUB)
endif()

# Step 3: 组合最终编译集合
if(UNICALIB_WITH_IKALIBR)
    # B 样条 + 选择的可视化
    set(_UNICALIB_SOURCES 
        ${IKALIBR_BSPLINE_SOURCES}
        ${UNICALIB_NEW_SOURCES})
    add_compile_definitions(UNICALIB_WITH_IKALIBR=1)
else()
    # 精简版 (无 B 样条)
    set(_UNICALIB_SOURCES 
        ${IKALIBR_MINIMAL_FOR_IMU_LIDAR}
        ${UNICALIB_NEW_SOURCES})
    add_compile_definitions(UNICALIB_WITH_IKALIBR=0)
endif()
```

---

### Step 3: 创建可视化 Stub 实现

**新建文件**: `src/ikalibr/viewer/viewer_stub.cpp`

```cpp
/**
 * iKalibr 可视化 Stub 实现 (无 Pangolin/无 GUI)
 * 用于在 UNICALIB_WITH_PANGOLIN=OFF 时提供空实现
 */

#include "ikalibr/viewer/viewer.h"

namespace ns_ikalibr {

class ViewerImpl : public std::enable_shared_from_this<ViewerImpl> {
public:
    ViewerImpl() = default;
    ~ViewerImpl() = default;

    static Ptr Create(const std::string& title) {
        return std::make_shared<Viewer>(title);
    }

    // 所有可视化方法为空实现 (no-op)
    void AddPointCloud(const VetaPtr& veta) {}
    void AddTrajectory(const SplineBundlePtr& spline) {}
    void ShowTrajectories(const std::string& topic, const Colour& colour) {}
    void ClearViewer() {}
    void Spin() {}
    void SpinOnce(int ms = 100) {}
    bool WaitKey(int ms = -1) { return false; }
};

}  // namespace ns_ikalibr
```

---

### Step 4: 创建 ROS RViz 适配层

**新建文件**: `src/viz/ros_rviz_visualizer.cpp`

```cpp
/**
 * UniCalib ROS RViz 可视化适配层
 * 将 iKalibr 数据发布为 ROS 消息，由 RViz 显示
 */

#include "unicalib/viz/ros_rviz_visualizer.h"

#if defined(UNICALIB_WITH_ROS_RVIZ) && defined(UNICALIB_WITH_ROS2)
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace ns_unicalib {

class RosRvizVisualizer {
public:
    RosRvizVisualizer(rclcpp::Node::SharedPtr node) : node_(node) {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/unicalib/markers", 100);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    }

    void PublishTrajectory(
        const std::string& frame_id,
        const std::vector<Sophus::SE3d>& poses,
        const std::string& namespace_id) {
        
        visualization_msgs::msg::MarkerArray markers;
        
        // 轨迹点
        for (size_t i = 0; i < poses.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.ns = namespace_id;
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.pose.position.x = poses[i].translation().x();
            marker.pose.position.y = poses[i].translation().y();
            marker.pose.position.z = poses[i].translation().z();
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            markers.markers.push_back(marker);
        }
        
        // 连接线
        if (poses.size() > 1) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame_id;
            line.ns = namespace_id + "_line";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.color.r = 1.0f;
            line.color.a = 1.0f;
            line.scale.x = 0.01;
            
            for (const auto& pose : poses) {
                geometry_msgs::msg::Point p;
                p.x = pose.translation().x();
                p.y = pose.translation().y();
                p.z = pose.translation().z();
                line.points.push_back(p);
            }
            markers.markers.push_back(line);
        }
        
        marker_pub_->publish(markers);
    }

    void PublishCoordinateFrame(
        const Sophus::SE3d& pose,
        const std::string& frame_id,
        const std::string& child_frame_id) {
        
        // 发布 TF2 变换
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = node_->now();
        t.header.frame_id = frame_id;
        t.child_frame_id = child_frame_id;
        
        t.transform.translation.x = pose.translation().x();
        t.transform.translation.y = pose.translation().y();
        t.transform.translation.z = pose.translation().z();
        
        auto q = pose.unit_quaternion();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace ns_unicalib

#else

// 无 ROS 时的空实现
namespace ns_unicalib {
class RosRvizVisualizer {
public:
    RosRvizVisualizer(void*) {}
    void PublishTrajectory(const std::string&, const std::vector<Sophus::SE3d>&, const std::string&) {}
    void PublishCoordinateFrame(const Sophus::SE3d&, const std::string&, const std::string&) {}
};
}

#endif
```

---

### Step 5: 修改 Phase3 代码（无需改动）

Phase3 的 `#ifdef UNICALIB_WITH_IKALIBR` 不需要改变，但现在的语义改为：

```cpp
void JointCalibSolver::phase3_joint_refine(...) {
#ifdef UNICALIB_WITH_IKALIBR
    // B 样条完整优化 (无论可视化后端是什么)
    solver->Process();
    // 可视化取决于构建时的选择:
    // - PANGOLIN=ON → 3D GUI
    // - ROS_RVIZ=ON → ROS Marker + TF2
    // - 都 OFF → 无 GUI，仅输出文件
#else
    // 降级到粗外参
    UNICALIB_WARN("B样条求解器未编译，使用 Phase2 粗外参");
#endif
}
```

---

## 新的编译命令

### 场景 1: 快速验证（无 B 样条）

```bash
cmake .. -DUNICALIB_WITH_IKALIBR=OFF \
         -DUNICALIB_WITH_PANGOLIN=OFF \
         -DUNICALIB_WITH_ROS_RVIZ=OFF
make -j

# 编译时间: 30-60s
# 输出: Phase 1-2 完整
# 可视化: PCL 查看器
```

### 场景 2: B 样条 + 无 GUI（服务器/容器）

```bash
cmake .. -DUNICALIB_WITH_IKALIBR=ON \
         -DUNICALIB_WITH_PANGOLIN=OFF \
         -DUNICALIB_WITH_ROS_RVIZ=OFF
make -j

# 编译时间: 1-2 分钟
# 输出: Phase 1-4 完整 B 样条优化
# 可视化: 无 GUI，输出 YAML/JSON 结果
```

### 场景 3: B 样条 + ROS RViz（ROS 环境）

```bash
cmake .. -DUNICALIB_WITH_IKALIBR=ON \
         -DUNICALIB_WITH_PANGOLIN=OFF \
         -DUNICALIB_WITH_ROS_RVIZ=ON \
         -DUNICALIB_WITH_ROS2=ON
make -j

# 编译时间: 2-5 分钟
# 输出: Phase 1-4 完整 B 样条优化
# 可视化: ROS RViz (点云、轨迹、坐标系)
```

### 场景 4: 完整版 + Pangolin（本地开发）

```bash
cmake .. -DUNICALIB_WITH_IKALIBR=ON \
         -DUNICALIB_WITH_PANGOLIN=ON \
         -DUNICALIB_WITH_ROS_RVIZ=OFF
make -j

# 编译时间: 5-15 分钟
# 输出: Phase 1-4 完整 B 样条优化
# 可视化: Pangolin 3D GUI
```

---

## 文件变更清单

### 需修改

| 文件 | 修改内容 |
|------|---------|
| `CMakeLists.txt` | 增加 `UNICALIB_WITH_PANGOLIN` 和 `UNICALIB_WITH_ROS_RVIZ` 选项；分离编译源文件逻辑 |
| `src/solver/joint_calib_solver.cpp` | 无需改动（#ifdef 逻辑保持不变） |
| `include/unicalib/viz/calib_visualizer.h` | 增加 ROS RViz 支持类声明（可选） |

### 需新建

| 文件 | 内容 |
|------|------|
| `src/ikalibr/viewer/viewer_stub.cpp` | Viewer 的空实现 (Pangolin 不可用时用) |
| `include/unicalib/viz/ros_rviz_visualizer.h` | ROS RViz 适配层头文件 |
| `src/viz/ros_rviz_visualizer.cpp` | ROS RViz 适配层实现 |
| `launch/unicalib_rviz.launch.py` | ROS 启动文件示例 |

---

## 构建摘要示例

```
========= UniCalib 构建配置 (改进后) =========
  Build type:        Release
  iKalibr B-spline:  ON  ← B 样条完整功能
  Pangolin viewer:   OFF ← 无 Pangolin 依赖
  ROS RViz viewer:   ON  ← 使用 ROS 可视化
  Visualization:     ROS_RVIZ (Markers + TF2)
==============================================
```

---

## 优势

✅ **B 样条功能不再依赖 Pangolin**  
✅ **支持多种可视化后端**（GUI/RViz/无GUI）  
✅ **编译灵活性大幅提升**  
✅ **容器部署友好**（选择轻量级后端）  
✅ **ROS 环境原生支持**  
✅ **降级路径仍保留**（可选 OFF 保持快速）  

---

## 迁移指南（为现有项目）

1. 备份现有 CMakeLists.txt
2. 应用上述修改
3. 测试三种场景编译
4. 更新文档和启动脚本
5. Phase3 代码无需修改

