# iKalibr ROS2 迁移修复记录

## 编译错误修复状态

### ✅ 已修复的问题

#### 1. ROS1/ROS2 头文件不一致
- **文件**: `iKalibr/include/sensor/imu_data_loader.h`
- **修复**: 将 `#include "sensor_msgs/Imu.h"` 改为 `#include <sensor_msgs/msg/imu.hpp>`

#### 2. veta-stub API 缺失
- **文件**: `iKalibr/thirdparty/veta-stub/include/veta/camera/pinhole.h`
- **修复**: 添加参数地址方法:
  ```cpp
  double* FXAddress() { return &K(0, 0); }
  double* FYAddress() { return &K(1, 1); }
  double* CXAddress() { return &K(0, 2); }
  double* CYAddress() { return &K(1, 2); }
  const double* FXAddress() const { return &K(0, 0); }
  const double* FYAddress() const { return &K(1, 1); }
  const double* CXAddress() const { return &K(0, 2); }
  const double* CYAddress() const { return &K(1, 2); }
  ```

#### 3. veta::Save() 不存在
- **文件**: `iKalibr/thirdparty/veta-stub/include/veta/veta.h`
- **修复**: 添加stub实现:
  ```cpp
  enum SaveFlags {
    ALL = 0,
    STRUCTURE = 1,
    POSES = 2,
    VIEWS = 4
  };
  inline bool Save(const Veta&, const std::string&, int flags = ALL) {
    // veta-stub 不实现文件 I/O
    return false;
  }
  ```

#### 4. 智能指针类型冲突
- **文件**: 
  - `iKalibr/include/viewer/viewer_types.h` - 移除boost依赖
  - `iKalibr/include/viewer/viewer_stub.h` - 移除boost兼容代码
  - `iKalibr/include/solver/calib_solver.h` - 移除 `IKalibrPointCloudPtr` boost定义
  - `iKalibr/include/viewer/visual_lidar_covisibility.h` - 添加Create重载
  - `iKalibr/include/viewer/visual_colorized_cloud_map.h` - 添加静态辅助方法
  - `iKalibr/include/util/cloud_define.hpp` - 添加 `MakeStdPtr()` 辅助函数

#### 5. PCL IO 头文件缺失
- **文件**: `iKalibr/src/solver/calib_solver_io.cpp`
- **修复**: 添加 `#include <pcl/io/pcd_io.h>`

#### 6. ns_veta::ALL 调用错误
- **文件**: `iKalibr/src/solver/calib_solver_io.cpp`
- **修复**: 将 `ns_veta::Veta::ALL` 改为 `ns_veta::ALL`

### ⚠️ 待修复问题

#### 1. ROS2消息头文件路径
某些sensor相关文件使用错误的ROS2头文件路径：
- `sensor_msgs/msg/image.hpp` 应该使用正确的ros2生成头
- `ikalibr/msg/prophesee_event_array.hpp` 等自定义消息未正确生成

**解决方法**:
1. 确保CMakeLists.txt中正确配置ROS2消息生成
2. 在编译前确保ROS2环境正确source
3. 检查是否需要先运行 `rosidl_generate_interfaces` 步骤

#### 2. pcl_conversions 依赖
- **错误**: `pcl_conversions/pcl_conversions.h: No such file or directory`
- **文件**: `iKalibr/include/sensor/lidar_data_loader.h`
- **需要**: 确保CMakeLists.txt包含 `find_package(pcl_conversions REQUIRED)`

#### 3. CMAKE_UNITY_BUILD 配置
CMakeLists.txt 中已设置 `USE_CMAKE_UNITY_BUILD OFF` 但可能需要验证:
```cmake
set(USE_CMAKE_UNITY_BUILD OFF CACHE STRINGS "whether use 'CMAKE_UNITY_BUILD' in building")
```

### 🔧 编译命令

```bash
# 在 Docker 容器内编译
docker run --rm -v /path/to/UniCalib:/root/calib_ws:rw calib_env:humble bash -c \
  "cd /root/calib_ws/iKalibr && colcon build --packages-select ikalibr"

# 或使用项目脚本
cd /path/to/UniCalib
./build_and_run.sh --build-only
```

### 📚 参考资料

- [ROS2 Humble Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-packages-from-ROS1)
- [ROS2 Message Generation](https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces)
- [CMake Unity Build](https://cmake.org/cmake/help/latest/manual/prop_tgt/UNITY_BUILD.html)
