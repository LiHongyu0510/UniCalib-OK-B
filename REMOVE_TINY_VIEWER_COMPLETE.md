# 彻底移除 tiny-viewer 依赖 - 完整方案

## 问题诊断

从编译日志中发现了以下问题：

### 问题 1：ctraj 源代码中大量引用 tiny-viewer 头文件

```
/root/calib_ws/iKalibr/thirdparty/ctraj/src/include/ctraj/view/traj_viewer.h:9:10:
    #include "tiny-viewer/core/viewer.h"
    
/root/calib_ws/iKalibr/thirdparty/ctraj/src/include/ctraj/utils/sophus_utils.hpp:13:10:
    #include "tiny-viewer/entity/utils.h"
    
/root/calib_ws/iKalibr/thirdparty/ctraj/src/include/ctraj/utils/eigen_utils.hpp:13:10:
    #include "tiny-viewer/entity/utils.h"
    
/root/calib_ws/iKalibr/thirdparty/ctraj/src/include/ctraj/core/pose.hpp:12:
    #include "tiny-viewer/entity/utils.h"
    
/root/calib_ws/iKalibr/thirdparty/ctraj/src/include/ctraj/factor/marginalization_factor.h:11:
    #include "tiny-viewer/entity/utils.h"
```

### 问题 2：ctraj 的 CMakeLists.txt 中有 tiny-viewer 的配置

```
find_dependency(tiny-viewer REQUIRED)
set(tiny-viewer_DIR ...)
```

## 解决方案

### 方案 1：完全重写 ctraj CMakeLists.txt（已实施）✅

**文件**：`iKalibr/build_thirdparty_ros2.sh`

**变更**：`build_ctraj()` 函数

**新方法**：完全重写 CMakeLists.txt，而不是使用 `sed` 命令

```bash
cat > "${ctraj_cmake}" << 'EOF'
cmake_minimum_required(VERSION 3.10)
project(ctraj VERSION 1.0)

# 设置库名称和空间
set(LIBRARY_NAME ctraj)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_FLAGS "\${CMAKE_C_FLAGS} -Wall -O3 -Wno-unused-function")
set(CMAKE_CXX_FLAGS "\${CMAKE_CXX_FLAGS} -Wall -O3 -Wno-unused-function")
set(CMAKE_C_FLAGS_RELEASE "\${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "\${CMAKE_CXX_FLAGS_RELEASE} -march=native")

add_subdirectory(\${CMAKE_CURRENT_SOURCE_DIR}/src)

# Configuration
set(CONFIG_INSTALL_DIR "lib/cmake/\${LIBRARY_NAME}")
set(INCLUDE_INSTALL_DIR "include")
set(VERSION_CONFIG "\${CMAKE_CURRENT_BINARY_DIR}/\${LIBRARY_NAME}ConfigVersion.cmake")
set(PROJ_CONFIG "\${CMAKE_CURRENT_BINARY_DIR}/\${LIBRARY_NAME}Config.cmake")

# Include module with function 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
write_basic_package_version_file("\${VERSION_CONFIG}" COMPATIBILITY SameMajorVersion)

# Configure '<PROJECT-NAME>Config.cmake'
configure_package_config_file(
    \${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in
    "\${PROJ_CONFIG}"
    INSTALL_DESTINATION "\${CONFIG_INSTALL_DIR}"
)

# Export '<LIBRARY_NAME>Targets.cmake' to build dir
export(
    TARGETS \${LIBRARY_NAME}
    FILE "\${CMAKE_CURRENT_BINARY_DIR}/\${TARGETS_EXPORT_NAME}.cmake"
)

# Targets:
install(
    TARGETS \${LIBRARY_NAME}
    EXPORT "\${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "\${INCLUDE_INSTALL_DIR}"
)

# Config
install(
    FILES "\${PROJ_CONFIG}" "\${VERSION_CONFIG}"
    DESTINATION "\${CONFIG_INSTALL_DIR}"
)

install(
    EXPORT "\${TARGETS_EXPORT_NAME}"
    NAMESPACE "\${namespace}"
    DESTINATION "\${CONFIG_INSTALL_DIR}"
)

# Headers
install(
    DIRECTORY \${CMAKE_CURRENT_SOURCE_DIR}/src/include/\${LIBRARY_NAME}
    DESTINATION \${INCLUDE_INSTALL_DIR}
    FILES_MATCHING PATTERN "*.h"
    PATTERN "*.hpp"
)
EOF
```

**优点**：
- ✅ 完全移除所有 tiny-viewer 引用
- ✅ 简单清晰，易于维护
- ✅ 避免复杂的 sed 命令

### 方案 2：修改 ctraj 源代码（需要手动或额外脚本）

**问题**：ctraj 的源代码中有大量 `#include "tiny-viewer/..."` 引用

**解决方法**：
1. **注释所有 tiny-viewer 头文件引用**：
   ```bash
   cd iKalibr/thirdparty/ctraj
   # 批量注释所有 tiny-viewer 的 #include
   find src -type f \( -name "*.h" -o -name "*.hpp" \) -exec sed -i 's/#include "tiny-viewer/\/\/#include "tiny-viewer/' {} \;
   ```

2. **修改相关源文件**：
   - `src/view/traj_viewer.cpp`：移除 tiny-viewer 功能
   - `src/utils/sophus_utils.hpp`：移除 tiny-viewer 实体工具
   - 等等...

**注意**：这需要大量的源代码修改，可能影响 ctraj 的其他功能。

### 方案 3：创建空的 stub 头文件（推荐用于临时解决）

**方法**：创建空的 tiny-viewer 头文件，避免编译错误

```bash
cd iKalibr/thirdparty/ctraj

# 创建 stub 目录
mkdir -p src/include/tiny-viewer

# 创建空的 stub 头文件
cat > src/include/tiny-viewer/core/viewer.h << 'EOF'
// Stub header for tiny-viewer (removed due to unavailability)
// This is a placeholder to avoid compilation errors
namespace tiny_viewer {
    // Empty namespace
}
EOF

cat > src/include/tiny-viewer/object/landmark.h << 'EOF'
// Stub header
namespace tiny_viewer {
    struct landmark {};
}
EOF

cat > src/include/tiny-viewer/object/camera.h << 'EOF'
// Stub header
namespace tiny_viewer {
    struct camera {};
}
EOF

cat > src/include/tiny-viewer/core/pose.hpp << 'EOF'
// Stub header
namespace tiny_viewer {
    using Pose = Eigen::Isometry3d;
}
EOF

cat > src/include/tiny-viewer/entity/arrow.h << 'EOF'
// Stub header
namespace tiny_viewer {
    struct arrow {};
}
EOF

cat > src/include/tiny-viewer/object/surfel.h << 'EOF'
// Stub header
namespace tiny_viewer {
    struct surfel {};
}
EOF

cat > src/include/tiny-viewer/entity/utils.h << 'EOF'
// Stub header
namespace tiny_viewer {
    namespace entity {
        // Empty namespace
    }
}
EOF
```

**优点**：
- ✅ 快速解决编译错误
- ✅ 最小化代码修改
- ✅ 易于后续维护

**缺点**：
- ⚠️  ctraj 的可视化功能将无法工作
- ⚠️ 可能影响某些依赖 tiny-viewer 的算法

## 推荐方案

### 短期（立即可用）✅

**使用方案 1**：重写 CMakeLists.txt + 方案 3：创建 stub 头文件

这个组合方案可以：
1. 立即解决编译错误（方案 3）
2. 完全移除 tiny-viewer 的 CMake 依赖（方案 1）

**实施步骤**：
```bash
cd iKalibr/thirdparty/ctraj

# 1. 创建 stub 头文件（方案 3）
bash create_stub_headers.sh

# 2. 重新编译
cd ../../
./build_thirdparty_ros2.sh
```

### 长期（可选）

如果需要完整的 ctraj 功能，可以考虑：

1. **寻找替代库**：找到其他轻量级 3D 可视化库
2. **集成 RViz2**：直接使用 ROS2 的可视化工具
3. **自定义实现**：实现基本的三维渲染功能

## 验证计划

### 编译验证

| 验证项 | 方法 | 预期结果 |
|--------|------|---------|
| ctraj 编译成功 | 检查 build.log | ✅ 无 tiny-viewer 相关错误 |
| 库文件生成 | `ls thirdparty-install/ctraj-install/lib` | ✅ libctraj.so 存在 |
| CMake 配置正确 | `ls thirdparty-install/ctraj-install/lib/cmake` | ✅ ctrajConfig.cmake 存在 |

### 功能验证

| 验证项 | 方法 | 预期结果 |
|--------|------|---------|
| iKalibr 编译 | `colcon build` | ✅ 无错误 |
| 可执行文件生成 | `ls install/lib/ikalibr/` | ✅ ikalibr_prog 等存在 |
| 不链接 tiny-viewer | `ldd ikalibr_prog` | ✅ 无 tiny-viewer 引用 |

## 当前状态

### 已完成的工作

✅ **iKalibr/build_thirdparty_ros2.sh**：已更新，使用完整重写方法

✅ **iKalibr/CMakeLists.txt**：已禁用 viewer 模块

✅ **build_external_tools.sh**：已移除 tiny-viewer 环境变量

✅ **verify_ikalibr_build.sh**：已更新，移除 tiny-viewer 检查

### 待完成的工作

⏳ **ctraj stub 头文件创建**：需要创建空的 tiny-viewer 头文件

⏳ **ctraj 源代码注释**：可选，注释源代码中的 tiny-viewer 引用

## 下一步行动

### 立即执行

1. ✅ **创建 stub 头文件**：
   ```bash
   cd iKalibr/thirdparty/ctraj
   # 创建必要的 stub 头文件
   mkdir -p src/include/tiny-viewer/{core,object,entity}
   # 创建空的头文件...
   ```

2. ✅ **重新编译**：
   ```bash
   cd /home/wqs/Documents/github/UniCalib
   ./build_and_run.sh --build-external-only 2>&1 | tee build.log
   ```

3. ✅ **验证编译结果**：
   ```bash
   cd iKalibr
   ./verify_ikalibr_build.sh
   ```

---

## 总结

### 关键发现

1. **问题根源**：
   - ctraj 的 CMakeLists.txt 中有 tiny-viewer 的 find_dependency
   - ctraj 源代码中有大量 `#include "tiny-viewer/..."` 引用
   - `sed` 命令没有完全移除所有引用

2. **影响**：
   - 编译失败，无法找到 tiny-viewer 头文件
   - 需要创建 stub 头文件或修改源代码

3. **解决思路**：
   - 短期：重写 CMakeLists.txt + 创建 stub 头文件
   - 长期：注释源代码中的引用或寻找替代库

### 推荐方案

**短期（推荐）**：
1. 重写 ctraj/CMakeLists.txt（已完成）✅
2. 创建 stub 头文件（待完成）⏳
3. 重新编译测试

**长期（可选）**：
1. 注释 ctraj 源代码中的 tiny-viewer 引用
2. 寻找替代的可视化库
3. 集成 RViz2 可视化

---

**文档版本**: 2.0  
**最后更新**: 2026-03-01  
**状态**: ✅ CMakeLists.txt 重写完成，等待 stub 头文件创建  
**维护者**: UniCalib Team
