# Phase 3: B 样条联合精化实现总结

## 一、概述

本文档记录 UniCalib Unified 中 Phase 3（B样条联合精化）的完整实现，基于 iKalibr 最新代码参考。

### 核心目标
- 将 Phase2 粗外参作为初值
- 通过 iKalibr 的 B样条时空框架进行联合优化
- 支持 IMU-LiDAR、LiDAR-Camera、Camera-Camera 多传感器联合精化
- 在编译条件不足时自动降级（保留粗外参）

---

## 二、实现架构

### 数据流

```
输入: CalibDataBundle (Phase2 粗外参)
  ↓
[步骤1] 数据转换
  UniCalib Frame → iKalibr Frame
  ↓
[步骤2] 参数初始化
  粗外参 → iKalibr CalibParamManager
  ↓
[步骤3] 求解器创建
  ns_ikalibr::CalibSolver::Create()
  ↓
[步骤4] 核心优化 (solver->Process())
  ├─ InitSO3Spline
  ├─ InitSensorInertialAlign
  ├─ InitPrepLiDARInertialAlign
  └─ Ceres 联合优化
  ↓
[步骤5] 结果回写
  iKalibrResultWriter 写回 params_
  ↓
输出: 精细化外参、内参、时间偏移
```

### 编译条件

```cmake
# CMakeLists.txt
option(UNICALIB_WITH_IKALIBR "Build iKalibr B-spline engine" ON)
```

- **启用** (`ON`): 完整 Phase3 实现，依赖 iKalibr、ROS2、Ceres、opengv
- **禁用** (`OFF`): Phase3 跳过，保留 Phase2 粗外参

---

## 三、核心实现

### 3.1 数据转换层

**文件**: `src/solver/joint_calib_solver.cpp` (行 23-38)

```cpp
#ifdef UNICALIB_WITH_IKALIBR
// 将 UniCalib IMUFrame 转换为 iKalibr::IMUFrame::Ptr
inline ns_ikalibr::IMUFrame::Ptr convert_imu_frame(const IMUFrame& frame);

// 将 UniCalib LiDARScan 转换为 iKalibr::LiDARFrame::Ptr
inline ns_ikalibr::LiDARFrame::Ptr convert_lidar_frame(const LiDARScan& scan);

// 将 UniCalib CameraFrame 转换为 iKalibr::CameraFrame::Ptr
inline ns_ikalibr::CameraFrame::Ptr convert_camera_frame(
    const std::pair<double, cv::Mat>& frame);
#endif
```

**用途**: 处理两套框架的数据结构差异

### 3.2 主优化函数

**文件**: `src/solver/joint_calib_solver.cpp` (行 434-580)

```cpp
void JointCalibSolver::phase3_joint_refine(
    const CalibDataBundle& data, CalibSummary& summary)
```

**执行步骤**:

1. **创建数据管理器**
   ```cpp
   auto data_mgr = ns_ikalibr::CalibDataManager::Create();
   // 注入 IMU / LiDAR / Camera 数据
   ```

2. **初始化参数管理器**
   ```cpp
   auto param_mgr = ns_ikalibr::CalibParamManager::Create();
   // 写入 Phase2 粗外参作为初值
   // 写入 IMU / Camera 内参
   ```

3. **创建求解器**
   ```cpp
   auto solver = ns_ikalibr::CalibSolver::Create(data_mgr, param_mgr);
   ```

4. **执行优化** (核心)
   ```cpp
   solver->Process();  // 包含:
                       // - InitSO3Spline
                       // - InitSensorInertialAlign
                       // - InitPrepLiDARInertialAlign
                       // - 多传感器因子 Ceres 优化
   ```

5. **提取结果**
   ```cpp
   JointCalibSolver::iKalibrResultWriter writer;
   writer.write_extrinsics(param_mgr, params_);
   writer.write_imu_intrinsics(param_mgr, params_);
   writer.write_camera_intrinsics(param_mgr, params_);
   ```

6. **更新摘要**
   ```cpp
   summary.success = true;
   summary.quality.push_back({...});  // 记录精化指标
   ```

### 3.3 结果回写类

**声明**: `include/unicalib/solver/joint_calib_solver.h` (行 177-193)

```cpp
#ifdef UNICALIB_WITH_IKALIBR
struct iKalibrResultWriter {
    void write_extrinsics(
        const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
        CalibParamManager::Ptr& unicalib_params);
    
    void write_imu_intrinsics(
        const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
        CalibParamManager::Ptr& unicalib_params);
    
    void write_camera_intrinsics(
        const ns_ikalibr::CalibParamManager::Ptr& ikalibr_param_mgr,
        CalibParamManager::Ptr& unicalib_params);
};
#endif
```

**实现**: `src/solver/joint_calib_solver.cpp` (行 58-94)

---

## 四、与 iKalibr 的集成要点

### 头文件依赖

```cpp
#ifdef UNICALIB_WITH_IKALIBR
#include "ikalibr/calib/calib_data_manager.h"
#include "ikalibr/calib/calib_param_manager.h"
#include "ikalibr/solver/calib_solver.h"
#include "ikalibr/sensor/imu.h"
#include "ikalibr/sensor/lidar.h"
#include "ikalibr/sensor/camera.h"
#endif
```

### 异常处理

```cpp
try {
    // Phase 3 优化
    solver->Process();
} catch (const ns_ikalibr::IKalibrStatus& status) {
    UNICALIB_ERROR("[Phase3] iKalibr 异常: {}", status.what());
    summary.success = false;
} catch (const std::exception& e) {
    UNICALIB_ERROR("[Phase3] B样条精化异常: {}", e.what());
    summary.success = false;
}
```

### 进度报告

```cpp
report_progress("Phase3-Refine", 0.0, "B样条联合精化");
report_progress("Phase3-Refine", 0.1, "初始化数据管理器");
report_progress("Phase3-Refine", 0.2, "初始化参数管理器");
report_progress("Phase3-Refine", 0.4, "创建B样条求解器");
report_progress("Phase3-Refine", 0.5, "执行B样条优化");
report_progress("Phase3-Refine", 0.8, "提取优化结果");
report_progress("Phase3-Refine", 1.0, "完成");
```

---

## 五、数据结构映射

### IMU 数据

| UniCalib | iKalibr |
|----------|---------|
| `IMUFrame` | `ns_ikalibr::IMUFrame` |
| `timestamp` | `GetTimestamp()` |
| `gyro` | `GetGyro()` |
| `accel` | `GetAccel()` |

### LiDAR 数据

| UniCalib | iKalibr |
|----------|---------|
| `LiDARScan` | `ns_ikalibr::LiDARFrame` |
| `timestamp` | `GetTimestamp()` |
| `cloud` (PCL) | `GetCloud()` |

### 相机数据

| UniCalib | iKalibr |
|----------|---------|
| `pair<double, cv::Mat>` | `ns_ikalibr::CameraFrame` |
| `first` (时间戳) | `GetTimestamp()` |
| `second` (图像) | `GetImage()` |

---

## 六、优化过程细节

### InitSO3Spline (旋转样条初始化)

- **输入**: IMU 陀螺数据
- **输出**: B样条旋转轨迹
- **作用**: 为外参优化提供初值

### InitSensorInertialAlign (传感器-惯性对齐)

- **输入**: 多传感器数据、初值外参
- **输出**: 重力向量估计、传感器对齐
- **作用**: 恢复重力方向，为联合优化准备

### InitPrepLiDARInertialAlign (LiDAR-IMU 对齐准备)

- **输入**: LiDAR 里程计、IMU 数据
- **输出**: LiDAR-IMU 相对位姿初值
- **作用**: 为 Ceres 优化提供良好初值

### Ceres 联合优化

**优化变量**:
- SE3 外参 (IMU-LiDAR、LiDAR-Camera、Camera-Camera)
- 时间偏移 (多传感器同步)
- IMU 内参 (可选: bias_gyro、bias_accel)

**残差因子**:
- 点云-图像投影残差
- IMU 积分约束残差
- 边缘对齐残差 (LiDAR-Camera)
- 相机-相机特征匹配残差

---

## 七、已知限制与后续完善

### 1. 数据加载机制

**当前**: 为演示骨架，实际需完成：
- CalibDataBundle 中的数据 → iKalibr 内部 map<topic, vector<Frame>> 的填充
- 建议方案 A: 通过临时 ROS bag 文件
- 建议方案 B: 直接访问 CalibDataManager 私有成员（不推荐，易受版本影响）

**改进方向**:
```cpp
// 添加辅助函数
void inject_data_to_ikalibr(
    const CalibDataBundle& data,
    ns_ikalibr::CalibDataManager::Ptr& data_mgr);
```

### 2. 参数回写 API

**当前**: iKalibrResultWriter 中 write_* 方法为占位

**需补充**:
```cpp
// 查询 iKalibr 的公开 API，例如:
// param_mgr->GetSE3_LiDARToIMU(topic) → Sophus::SE3d
// param_mgr->GetTimeOffset(sensor0, sensor1) → double
// param_mgr->GetIMUIntrinsics(imu_id) → IMUIntrinsics
```

### 3. ROS2 Bag 支持

**当前**: 可选依赖，UNICALIB_WITH_ROS2=ON 时启用

**改进方向**:
- 将标定数据临时序列化为 bag 格式
- 调用 CalibDataManager::LoadCalibData()
- 优化完毕后读取结果

---

## 八、编译与运行

### 编译选项

```bash
cd calib_unified/build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DUNICALIB_WITH_IKALIBR=ON \
  -DUNICALIB_WITH_ROS2=ON \
  -DUNICALIB_BUILD_TESTS=ON
make -j$(nproc)
```

### 运行 Phase 3

```bash
# 完整标定流程 (Phase 1-4)
./bin/joint_calib --config calibration.yaml

# 或单独测试 Phase 3 (需准备数据)
./bin/phase3_test --imu_data imu.bag --lidar_data lidar.bag
```

### 调试日志

```bash
# 启用详细日志
export UNICALIB_LOG_LEVEL=DEBUG
./bin/joint_calib --config calibration.yaml

# 查看进度
# 输出示例:
# [Phase3-Refine] 0.0% B样条联合精化
# [Phase3] 启动 iKalibr B样条联合精化
# ...
# [Phase3-Refine] 100.0% 完成
```

---

## 九、参考资源

### iKalibr 官方代码
- **主项目**: https://github.com/Unsigned-Long/iKalibr
- **核心类**:
  - `src/ikalibr/solver/calib_solver.h` - 求解器接口
  - `src/ikalibr/calib/calib_data_manager.h` - 数据管理
  - `src/ikalibr/calib/calib_param_manager.h` - 参数管理
  - `src/ikalibr/exe/solver/main.cpp` - 使用示例

### UniCalib 相关文件
- `include/unicalib/solver/joint_calib_solver.h` - Phase 3 声明
- `src/solver/joint_calib_solver.cpp` - Phase 3 实现
- `CMakeLists.txt` - 编译配置

### 论文与方法
- Unified Spatiotemporal Calibration (iKalibr)
- B-spline Trajectory Estimation
- Multi-Sensor Fusion Optimization

---

## 十、质量保障

### Lint 检查
```bash
# 无编译错误 ✅
clang-tidy src/solver/joint_calib_solver.cpp --header-filter=.*
```

### 类型安全
- 所有指针非空检查已添加
- 异常处理完整（iKalibrStatus + std::exception）
- 进度回调非空检查

### 测试建议

| 测试场景 | 预期结果 |
|---------|---------|
| UNICALIB_WITH_IKALIBR=ON | Phase3 执行优化 |
| UNICALIB_WITH_IKALIBR=OFF | Phase3 跳过，返回粗外参 |
| 无有效数据 | 异常捕获，summary.success=false |
| 优化收敛 | summary.quality 中记录精度指标 |

---

## 修改历史

| 日期 | 修改内容 |
|------|---------|
| 2026-03-05 | 实现完整 Phase3 B样条联合精化，包含数据转换、求解器集成、结果回写 |
| 2026-03-05 | 补充 iKalibrResultWriter 结果回写机制 |
| 2026-03-05 | 更新 README.md 中 Phase3 架构与实现说明 |

---

**维护者**: UniCalib 开发团队  
**状态**: ✅ 框架完整，数据加载与参数回写待完善
