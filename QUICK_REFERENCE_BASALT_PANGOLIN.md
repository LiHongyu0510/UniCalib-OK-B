# 快速参考卡：basalt-headers + Pangolin 集成
## UniCalib B-样条求解与可视化统一方案

---

## 📋 架构总览

```
┌─────────────────────────────────────────────────────┐
│        UniCalib 多传感器统一标定系统                │
├─────────────────────────────────────────────────────┤
│                                                       │
│  Phase 1-2: 内参/外参标定 (联合)                     │
│            ↓                                         │
│  Phase 3: B-样条联合精化                             │
│    ├─ 数据源：IMU + LiDAR 轨迹 (Phase 2 结果)       │
│    │                                                 │
│    ├─ 求解核心                                      │
│    │  ├─ 底层：basalt-headers/spline/ ✓ 本地         │
│    │  │   └─ Se3Spline<5, double>                  │
│    │  │   └─ CeresSplineHelper (自动求导)          │
│    │  │                                             │
│    │  └─ 上层：joint_calib_solver (UniCalib)        │
│    │      └─ 数据转换 / 优化执行 / 结果回写         │
│    │                                                 │
│    ├─ 可视化后端（选一）                            │
│    │  ├─ Pangolin（推荐）→ 交互式 3D 显示          │
│    │  ├─ RViz（ROS）→ ROS Topic 发布               │
│    │  └─ Stub（Headless）→ 无 GUI                  │
│    │                                                 │
│    └─ 输出：精化后的外参/IMU 内参/标定精度评估      │
│                                                       │
└─────────────────────────────────────────────────────┘
```

---

## 🔧 编译命令速查表

| 用途 | 编译命令 | 库大小 | 编译时间 | 可视化 |
|------|---------|--------|---------|--------|
| **开发/演示** | `cmake .. -UNICALIB_WITH_IKALIBR=ON -UNICALIB_WITH_PANGOLIN=ON` | ~85 MB | ~2 min | ✅ Pangolin |
| **一般用户** | `cmake .. -UNICALIB_WITH_IKALIBR=ON -UNICALIB_WITH_PANGOLIN=ON` | ~45 MB | ~1 min | ✅ Pangolin |
| **ROS 集成** | `cmake .. -UNICALIB_WITH_IKALIBR=ON -UNICALIB_WITH_ROS_RVIZ=ON` | ~40 MB | ~1 min | ✅ RViz |
| **纯计算** | `cmake .. -UNICALIB_WITH_IKALIBR=ON` (无可视化) | ~35 MB | ~45 sec | ❌ 无 |
| **快速尝试** | `cmake .. -UNICALIB_WITH_IKALIBR=OFF` | ~15 MB | ~15 sec | ❌ 无 |

---

## 📁 关键文件对应表

### basalt-headers（本地库）
```
calib_unified/thirdparty/basalt-headers/
├── include/basalt/spline/
│   ├── se3_spline.h              ← SE(3) 样条
│   ├── so3_spline.h              ← SO(3) 样条
│   ├── rd_spline.h               ← R^d 样条
│   ├── ceres_spline_helper.h     ← Ceres 集成 ⭐
│   ├── ceres_local_param.hpp     ← 李群参数化
│   └── spline_common.h           ← 公共函数
```

### Pangolin（本地库）
```
calib_unified/thirdparty/Pangolin/Pangolin-0.9.0/
├── components/
│   ├── pango_display/           ← 3D 显示
│   ├── pango_opengl/            ← OpenGL 渲染
│   ├── pango_plot/              ← 曲线绘图 ⭐
│   └── pango_core/              ← 核心库
```

### UniCalib 改动点
```
calib_unified/
├── CMakeLists.txt               ← 改：行 42-54, 155-170, 472-550 ⭐
├── src/solver/
│   └── joint_calib_solver.cpp   ← 改：phase3_joint_refine() ⭐
├── include/unicalib/solver/
│   └── joint_calib_solver.h     ← 改：方法声明
├── src/viz/
│   ├── calib_visualizer.cpp     ← 改：Pangolin 集成
│   ├── viewer_stub.cpp          ← 新：无 GUI 实现 ⭐
│   └── ros_rviz_visualizer.cpp  ← 新：RViz 适配（可选）
└── include/unicalib/viz/
    └── ros_rviz_visualizer.h    ← 新：RViz 头文件（可选）
```

---

## 🎯 三个核心要点

### 1️⃣ B-样条求解独立于可视化

```cpp
// 编译时逻辑
if (UNICALIB_WITH_IKALIBR) {
    // 无论可视化是什么，B-样条代码都被编译
    // 取决于：Ceres, Eigen, Sophus（都已本地化）✓
    include(basalt-headers + Ceres)
}

// 独立控制可视化
if (UNICALIB_WITH_PANGOLIN) { include(viewer_pangolin.cpp) }
else if (UNICALIB_WITH_ROS_RVIZ) { include(ros_rviz_visualizer.cpp) }
else { include(viewer_stub.cpp) }  // 无 GUI
```

### 2️⃣ 编译依赖最小化

| 配置 | 依赖 | 优势 |
|------|------|------|
| iKalibr ON + Pangolin | Ceres, Eigen, Sophus, OpenGL | 本地库齐全，快速 |
| iKalibr ON | Ceres, Eigen, Sophus | 纯计算，无 GUI |
| iKalibr OFF | Eigen, Sophus | 超快，测试专用 |

**对比旧方案**:
- ❌ 旧：iKalibr ON → 必须编译 ROS2 + velodyne_msgs + ufomap
- ✅ 新：iKalibr ON → 仅需 Ceres + 本地库

### 3️⃣ 可视化后端互斥选择

```
优先级：Pangolin > RViz > Stub（Default）
      (本地)     (ROS)    (无 GUI)

选择规则：
1. UNICALIB_WITH_PANGOLIN=ON              → 使用 Pangolin
2. 否则，UNICALIB_WITH_ROS_RVIZ=ON 且 ROS → 使用 RViz
3. 否则                                     → 使用 Stub（无 GUI）
```

---

## 🚀 快速开始

### 第一次编译（完整 + Pangolin）
```bash
cd ~/UniCalib
mkdir -p build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DUNICALIB_WITH_IKALIBR=ON \
    -DUNICALIB_WITH_PANGOLIN=ON

make -j$(nproc) unicalib

# 验证
ls -lh lib/libunicalib.so
file lib/libunicalib.so
```

### 后续编译（精简 + 无可视化）
```bash
cmake .. -DUNICALIB_WITH_IKALIBR=ON  # Pangolin 自动 ON（如本地可用）
make -j$(nproc)
```

### 最小编译（用于快速测试）
```bash
cmake .. -DUNICALIB_WITH_IKALIBR=OFF
make -j$(nproc)  # < 15 秒
```

---

## 🧪 验证检查清单

- [ ] `basalt-headers/include/basalt/spline/se3_spline.h` 存在
- [ ] `cmake ..` 输出：`basalt-headers: ✓ Found`
- [ ] `cmake ..` 输出：`Visualization: ✓ Pangolin 3D backend`
- [ ] `make` 完成，无 linker 错误
- [ ] `lib/libunicalib.so` 文件大小合理（45-85 MB）
- [ ] 可选：`nm lib/libunicalib.so | grep Se3Spline` 有输出 ✓

---

## 📊 性能预期

| 指标 | 值 |
|------|-----|
| 编译时间（Full） | 60-120 秒 |
| 编译时间（Lean） | 30-45 秒 |
| 库大小（Full） | 45-85 MB |
| 库大小（Lean） | 35-45 MB |
| Phase3 运行时（1000 IMU + 100 LiDAR） | < 5 分钟 |
| 内存使用（优化中） | < 500 MB |

---

## 🐛 常见问题与解决

### Q1: "basalt-headers not found"
```bash
# 检查
ls calib_unified/thirdparty/basalt-headers/include/basalt/spline/se3_spline.h

# 若不存在，初始化
cd calib_unified/thirdparty && git submodule update --init basalt-headers
```

### Q2: "Ceres not found"
```bash
sudo apt install libceres-dev

# 或指定路径
cmake .. -DCeres_DIR=/usr/lib/cmake/Ceres
```

### Q3: Pangolin 编译失败
```bash
# 清理并重新配置
rm -rf build && mkdir build && cd build
cmake ..
make -j4  # 试试 4 线程（Pangolin 有时多线程有问题）
```

### Q4: "IKALIBR_VIEWER_BACKEND undefined"
```bash
# 在 CMakeLists.txt 中检查行 472-550
# 确保所有分支都定义了 IKALIBR_VIEWER_BACKEND
add_compile_definitions(IKALIBR_VIEWER_BACKEND=...)
```

---

## 📝 代码片段

### 在代码中检查编译模式
```cpp
#ifdef UNICALIB_WITH_IKALIBR
    // B-样条求解可用
    UNICALIB_LOG(INFO) << "B-spline solver enabled";
    auto spline = basalt::Se3Spline<5, double>();
#else
    // 使用降级方案
    UNICALIB_LOG(WARNING) << "B-spline disabled, using Phase2 result";
#endif

// 可视化检查
#if defined(IKALIBR_VIEWER_BACKEND)
    #if IKALIBR_VIEWER_BACKEND == PANGOLIN
        viz.show_bspline_3d();
    #elif IKALIBR_VIEWER_BACKEND == RVIZ
        viz.publish_to_rviz();
    #else
        // Stub: no-op
    #endif
#endif
```

### 在 CMake 中查询
```cmake
# CMakeLists.txt 中
if(UNICALIB_WITH_IKALIBR)
    message(STATUS "B-spline solver: ENABLED")
    target_link_libraries(myapp PUBLIC basalt::headers Ceres::ceres)
else()
    message(STATUS "B-spline solver: DISABLED")
endif()
```

---

## 📚 相关文档

| 文档 | 内容 |
|------|------|
| `LOCAL_DEPENDENCIES_OPTIMIZATION.md` | 完整设计方案（60 页） |
| `CMAKELISTS_BASALT_PANGOLIN_INTEGRATION.md` | CMake 配置详解（40 页） |
| `PHASE3_IMPLEMENTATION_SUMMARY.md` | Phase3 实现细节（已有） |
| `COMPILATION_CONDITIONAL_ANALYSIS.md` | 编译条件分析（已有） |

---

## ✅ 完成度指标

- ✅ basalt-headers 本地集成
- ✅ Pangolin 本地可视化
- ✅ 编译选项独立化
- ✅ 可视化后端解耦
- ⏳ 单元测试覆盖（待实施）
- ⏳ RViz 适配器完整实现（待选择性补充）

---

## 🔗 快速链接

- **CMakeLists.txt 改动**: 行 42-54, 155-170, 472-550
- **核心实现**: `src/solver/joint_calib_solver.cpp` → `phase3_joint_refine()`
- **可视化**: `src/viz/calib_visualizer.cpp` → `show_bspline_optimization()`
- **测试**: `test/test_basalt_integration.cpp`（待补充）

---

**快速参考卡版本**: v1.0  
**更新时间**: 2026-03-05  
**维护者**: UniCalib 项目团队
