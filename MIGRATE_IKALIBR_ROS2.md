# 🔄 iKalibr ROS1 → ROS2 Humble 迁移方案

## 📋 问题说明

**当前状态**：
- iKalibr 使用 ROS1 (catkin) 构建系统
- Docker 镜像是 ROS2 Humble (ament/colcon)
- 两者**不兼容**

**错误信息**：
```
CMake Error at CMakeLists.txt:63 (find_package):
  By not providing "Findcatkin.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "catkin", but
  CMake did not find one.
```

---

## 🔍 ROS1 vs ROS2 差异

### 构建系统

| 特性 | ROS1 (catkin) | ROS2 (ament/colcon) |
|-----|----------------|-------------------|
| 构建工具 | catkin_make | colcon |
| CMake 宏 | `catkin_package()` | `ament_package()` |
| 消息生成 | `generate_messages()` | 自动通过 `rosidl` |
| Python 包 | `catkin_python_setup()` | 自动 |
| 安装规则 | `catkin_install()` | `install()` |

### 关键替换

| ROS1 (catkin) | ROS2 (ament/colcon) | 说明 |
|---------------|-------------------|------|
| `find_package(catkin REQUIRED)` | `find_package(rosidl_default_generators REQUIRED)` | 消息生成 |
| `catkin_package()` | `ament_package()` | 声明包 |
| `catkin_include_directories()` | `ament_target_dependencies()` | 包依赖 |
| `catkin_install()` | `install()` | 安装规则 |
| `${catkin_INCLUDE_DIRS}` | `${rosidl_default_generators_INCLUDE_DIRS}` | 包含目录 |
| `${catkin_LIBRARIES}` | - | 移除（ROS2 不需要）|

---

## 🎯 迁移方案

### 方案 A：手动修改 CMakeLists.txt（推荐）

**步骤**：

1. **移除 catkin 相关代码**
   ```cmake
   # 删除
   find_package(catkin REQUIRED COMPONENTS ...)
   catkin_package()
   catkin_python_setup()
   ```

2. **添加 ament 相关代码**
   ```cmake
   # 添加
   find_package(rosidl_default_generators REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rcl REQUIRED)
   ```

3. **替换包声明**
   ```cmake
   # 替换
   ament_package()
   ```

4. **更新安装规则**
   ```cmake
   # 替换 catkin_install() 为 install()
   install(DIRECTORY ...
   ```

### 方案 B：使用自动迁移工具

**推荐工具**：
- **ros2_migration**: https://github.com/ros2/ros2_migration
- **catkin_to_ament**: 自动转换脚本

---

## 📝 迁移步骤详解

### 步骤 1：更新 CMakeLists.txt

#### 1.1 移除 catkin 查找

**删除**：
```cmake
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    rosbag
    cv_bridge
    message_generation
    velodyne_msgs
    velodyne_pointcloud)
```

**添加**：
```cmake
## ROS2 dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
```

#### 1.2 替换包声明

**删除**：
```cmake
catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
    # DEPENDS system_lib
)
```

**添加**：
```cmake
ament_package()
```

#### 1.3 更新消息生成

**删除**：
```cmake
## Generate dynamic reconfigure parameters in 'cfg' folder
# generate_dynamic_reconfigure_options()
```

**添加**：
```cmake
## Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  "srv/MyService.srv"
)
```

#### 1.4 更新安装规则

**删除**：
```cmake
## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
install(TARGETS
    ${PROJECT_NAME}_prog
    ...
)

## Install libraries
install(TARGETS
    ${PROJECT_NAME}_util
    ...
)
```

**添加**：
```cmake
install(TARGETS
    ${PROJECT_NAME}_prog
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    scripts/
    DESTINATION lib/${PROJECT_NAME}
)
```

### 步骤 2：更新 package.xml

**ROS1 格式**：
```xml
<package format="2">
  <name>ikalibr</name>
  <version>0.0.0</version>
  <description>The ikalibr package</description>
  <maintainer>csl</maintainer>
  <license>TODO</license>

  <!-- Dependencies -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  ...
</package>
```

**ROS2 格式**：
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema-instance"?>
<package format="3">
  <name>ikalibr</name>
  <version>0.0.0</version>
  <description>The ikalibr package</description>
  <maintainer email="csl@todo.todo">csl</maintainer>
  <license>TODO</license>

  <!-- ROS2 dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>cv_bridge</depend>
  ...
</package>
```

**关键变化**：
- `format="2"` → `format="3"`
- `buildtool_depend>catkin` → `buildtool_depend>ament_cmake`
- `build_depend>` → `<depend>`

### 步骤 3：迁移 Python 代码

#### 3.1 ROS1 → ROS2 变量映射

| ROS1 (rospy) | ROS2 (rclpy) | 说明 |
|---------------|---------------|------|
| `import rospy` | `import rclpy` | Python 库 |
| `rospy.init_node()` | `rclpy.create_node()` | 节点初始化 |
| `rospy.Publisher()` | `rclpy.create_publisher()` | 发布者 |
| `rospy.Subscriber()` | `rclpy.create_subscription()` | 订阅者 |
| `rospy.Time()` | `builtin_interfaces.msg.Time` | 时间 |
| `rospy.Rate()` | `rclpy.Rate()` | 频率 |

#### 3.2 示例：节点初始化

**ROS1**：
```python
import rospy

def main():
    rospy.init_node('ikalibr_node')
    rospy.loginfo("Node initialized")
    rospy.spin()
```

**ROS2**：
```python
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('ikalibr_node')
    node.get_logger().info("Node initialized")
    rclpy.spin(node)
```

### 步骤 4：更新 launch 文件

**ROS1**：
```xml
<launch>
  <node pkg="ikalibr" type="ikalibr_node" name="ikalibr" output="screen"/>
</launch>
```

**ROS2**：
```xml
<launch>
  <node pkg="ikalibr" exec="ikalibr_node" name="ikalibr" output="screen"/>
</launch>
```

---

## 🚀 实施计划

### 阶段 1：CMakeLists.txt 迁移（2-3 天）

- [ ] 移除所有 `find_package(catkin)` 调用
- [ ] 移除 `catkin_package()` 调用
- [ ] 移除 `catkin_install()` 调用
- [ ] 添加 `find_package(rclcpp)` 等 ROS2 依赖
- [ ] 添加 `ament_package()` 调用
- [ ] 更新 `install()` 规则
- [ ] 测试编译

### 阶段 2：package.xml 更新（1 天）

- [ ] 修改 `format="2"` 为 `format="3"`
- [ ] 移除 `buildtool_depend>catkin`
- [ ] 添加 `buildtool_depend>ament_cmake`
- [ ] 将所有 `build_depend>` 改为 `<depend>`
- [ ] 验证 package.xml 语法

### 阶段 3：Python 代码迁移（3-5 天）

- [ ] 搜索所有 `import rospy`
- [ ] 替换为 `import rclpy`
- [ ] 更新节点初始化
- [ ] 更新发布者/订阅者
- [ ] 更新日志调用
- [ ] 测试节点运行

### 阶段 4：测试验证（1-2 天）

- [ ] 使用 `colcon build --symlink-install` 编译
- [ ] 验证可执行文件生成
- [ ] 测试节点运行
- [ ] 测试话题发布/订阅

---

## 🐛 常见问题

### 问题 1：CMake 找不到 rosidl_default_generators

**解决**：
```bash
# 确保已安装
apt-cache depends ros-humble-rosidl-default-generators
```

### 问题 2：Python 导入错误

**错误**：`ModuleNotFoundError: No module named 'rospy'`

**解决**：
```bash
# 检查代码中是否还有 import rospy
grep -r "import rospy" .
# 替换为 import rclpy
```

### 问题 3：launch 文件错误

**错误**：`<node type="...">` not supported

**解决**：
```xml
<!-- 错误 -->
<node type="ikalibr_node"/>

<!-- 正确 -->
<node exec="ikalibr_node"/>
```

---

## 📚 参考资源

### 官方文档
- [ROS2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-packages-from-ROS1.html)
- [ament_cmake Documentation](https://docs.ros.org/en/humble/Tutorials/Ament-CMake-Documentation.html)
- [colcon Documentation](https://colcon.readthedocs.io/)

### 迁移工具
- [ros2_migration](https://github.com/ros2/ros2_migration)
- [catkin_to_ament](https://github.com/nobleo/catkin_to_ament)

### 示例项目
- [ROS2 Example Packages](https://github.com/ros2/examples)
- [geometry2 Tutorials](https://github.com/ros2/geometry2)

---

## 📊 迁移检查清单

### CMakeLists.txt 检查

| 检查项 | ROS1 | ROS2 | 状态 |
|---------|------|------|------|
| 使用 `find_package(catkin)` | ✓ | ✗ | 需移除 |
| 使用 `catkin_package()` | ✓ | ✗ | 需移除 |
| 使用 `catkin_install()` | ✓ | ✗ | 需移除 |
| 使用 `ament_package()` | ✗ | ✓ | 需添加 |
| 使用 `rosidl_generate_interfaces()` | ✗ | ✓ | 需添加（如有消息） |
| 使用 `find_package(rclcpp)` | ✗ | ✓ | 需添加 |

### package.xml 检查

| 检查项 | ROS1 | ROS2 | 状态 |
|---------|------|------|------|
| `format="2"` | ✓ | ✗ | 需改为 `"3"` |
| `buildtool_depend>catkin` | ✓ | ✗ | 需改为 `ament_cmake` |
| `build_depend>` | ✓ | ✗ | 需改为 `<depend>` |
| `depend>` | ✗ | ✓ | 需添加（ROS2） |

### Python 代码检查

| 检查项 | ROS1 | ROS2 | 状态 |
|---------|------|------|------|
| `import rospy` | ✓ | ✗ | 需替换为 `rclpy` |
| `rospy.init_node()` | ✓ | ✗ | 需替换为 `rclpy.create_node()` |
| `rospy.Publisher()` | ✓ | ✗ | 需替换为 `create_publisher()` |
| `rospy.Subscriber()` | ✓ | ✗ | 需替换为 `create_subscription()` |

---

## 🎯 立即开始迁移

### 步骤 1：备份原始代码

```bash
cd /home/wqs/Documents/github/UniCalib/iKalibr

# 创建备份分支
git checkout -b ros2-migration

# 或手动备份
cp CMakeLists.txt CMakeLists.txt.ros1
cp package.xml package.xml.ros1
```

### 步骤 2：修改 CMakeLists.txt

```bash
# 使用 sed 批量替换（谨慎使用）
sed -i 's/find_package(catkin/find_package(rosidl_default_generators/g' CMakeLists.txt
sed -i 's/catkin_package(/ament_package(/g' CMakeLists.txt
sed -i 's/\${catkin_INCLUDE_DIRS}/\${rosidl_default_generators_INCLUDE_DIRS}/g' CMakeLists.txt
```

### 步骤 3：修改 package.xml

```bash
# 修改 format
sed -i 's/format="2"/format="3"/g' package.xml

# 修改 buildtool_depend
sed -i 's/<buildtool_depend>catkin/<buildtool_depend>ament_cmake/g' package.xml

# 修改所有 build_depend 为 depend
sed -i 's/<build_depend>/<depend>/g' package.xml
```

### 步骤 4：测试编译

```bash
cd /home/wqs/Documents/github/UniCalib

# 清理旧的编译产物
rm -rf iKalibr/build iKalibr/install iKalibr/log

# 使用 colcon 编译
docker run --rm \
    -v $(pwd):/root/calib_ws \
    -w /root/calib_ws/iKalibr \
    calib_env:humble \
    bash -c "cd /root/calib_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
```

### 步骤 5：验证结果

```bash
# 检查编译产物
ls -la iKalibr/build/ikalibr/lib/

# 检查 Python 节点
ls -la iKalibr/install/ikalibr/lib/

# 运行节点测试
docker run --rm -it \
    -v $(pwd):/root/calib_ws \
    calib_env:humble \
    bash -c "source install/setup.bash && ros2 run ikalibr_prog --help"
```

---

## 📞 总结

### 迁移规模

- **CMakeLists.txt**: ~830 行，需要大量修改
- **package.xml**: ~100 行，需要修改依赖声明
- **Python 代码**: ~50+ 文件，需要逐一检查和修改
- **launch 文件**: ~10+ 文件，需要更新语法

### 预估工作量

- **CMakeLists.txt 迁移**: 2-3 天
- **package.xml 更新**: 1 天
- **Python 代码迁移**: 3-5 天
- **测试和调试**: 1-2 天
- **总计**: 7-11 天

### 风险评估

| 风险 | 影响 | 缓解措施 |
|-----|------|---------|
| CMake 语法错误 | 编译失败 | 逐步测试，使用 cmake --trace |
| Python 运行时错误 | 节点崩溃 | 单元测试，逐步迁移 |
| 依赖不兼容 | 功能缺失 | 参考官方文档，使用等效 API |
| 性能回归 | 运行变慢 | 性能测试，优化关键路径 |

---

## 🚀 建议行动

### 短期（本周）

1. ✅ 创建备份分支
2. ✅ 迁移 CMakeLists.txt
3. ✅ 更新 package.xml
4. ✅ 测试编译
5. ✅ 运行基本测试

### 中期（下周）

1. ✅ 迁移 Python 代码
2. ✅ 迁移 launch 文件
3. ✅ 完整集成测试
4. ✅ 性能优化

### 长期（本月）

1. ✅ 添加 ROS2 特性
2. ✅ 更新文档
3. ✅ CI/CD 集成
4. ✅ 发布 ROS2 版本

---

## 📞 需要帮助？

### 官方资源
- [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-packages-from-ROS1.html)
- [ament_cmake Documentation](https://docs.ros.org/en/humble/Tutorials/Ament-CMake-Documentation.html)
- [colcon Documentation](https://colcon.readthedocs.io/)

### 社区支持
- [ROS Answers](https://answers.ros.org/)
- [ROS Discourse](https://discourse.ros.org/)
- [ROS2 GitHub Discussions](https://github.com/ros2/ros2/discussions)

---

## 📝 下一步

**立即可执行**：
```bash
cd /home/wqs/Documents/github/UniCalib/iKalibr

# 1. 创建备份
git checkout -b ros2-migration

# 2. 开始迁移（参考上述步骤）
# 3. 测试编译
# 4. 提交修改
```

**预计结果**：iKalibr 成功在 ROS2 Humble 上编译和运行！🎉
