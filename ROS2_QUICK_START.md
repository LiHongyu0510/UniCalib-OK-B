# ROS2 集成快速使用指南

## 快速开始

### 方式一：使用 ROS2 bag 文件（推荐）

```bash
# 1. 配置 bag 文件路径（编辑配置文件）
vim calib_unified/config/unicalib_example.yaml

# 在 ros2 部分添加：
ros2:
  use_ros2_bag: true
  ros2_bag_file: /path/to/your/data.bag
  lidar_topic: /velodyne_points
  camera_topic: /cam_left/image_raw
  max_frames: 100

# 2. 运行标定
./calib_unified_run.sh --run --task lidar-cam \
    --config calib_unified/config/unicalib_example.yaml
```

### 方式二：使用实时 ROS2 话题订阅

```bash
# 1. 在一个终端播放 bag 或启动传感器
ros2 bag play your_data.bag --topics /velodyne_points /cam_left/image_raw

# 2. 在另一个终端运行标定
./unicalib_lidar_camera \
    --config calib_unified/config/unicalib_example.yaml \
    --ros2-topic \
    --lidar-topic /velodyne_points \
    --camera-topic /cam_left/image_raw
```

## 配置示例

### 对于 `nya_02_ros2` 数据集

```yaml
ros2:
  use_ros2_bag: true
  ros2_bag_file: /path/to/nya_02_ros2.bag
  lidar_topic: /velodyne_points
  camera_topic: /cam_left/image_raw
  max_frames: 100
  sample_interval: 0.0

lidar_camera:
  method: edge
  prefer_targetfree: true
```

## 验证 ROS2 环境

```bash
# 检查 ROS2 是否安装
which ros2

# 查看可用话题
ros2 topic list

# 查看 bag 信息
ros2 bag info your_data.bag
```

## 常见问题

### Q1: 编译错误 "rosbag2_cpp not found"

**A**: 安装 ROS2 和相关依赖
```bash
sudo apt-get install -y ros-humble-ros2bag2-cpp \
    ros-humble-ros2bag2-storage \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions
```

### Q2: 运行时错误 "未找到话题"

**A**: 检查话题名称是否正确
```bash
# 查看所有话题
ros2 topic list

# 查看特定话题详情
ros2 topic echo /velodyne_points --once
```

### Q3: 数据加载超时

**A**: 增加等待时间或减少帧数要求
```yaml
ros2:
  max_wait_time: 60.0  # 增加到60秒
  max_frames: 50       # 减少帧数要求
```

### Q4: 内存不足

**A**: 限制加载的帧数
```yaml
ros2:
  max_frames: 50       # 从100减少到50
  sample_interval: 0.1  # 每100ms采样一帧
```

## 性能优化建议

1. **采样间隔**：设置 `sample_interval: 0.1` 减少冗余数据
2. **帧数限制**：根据机器内存调整 `max_frames`
3. **时间窗口**：使用 `time_window_start/end` 只加载需要的时间段
4. **数据过滤**：在配置中启用距离和 NaN 过滤

## 详细文档

完整的 ROS2 集成文档请参阅：
- [ROS2_INTEGRATION.md](calib_unified/docs/ROS2_INTEGRATION.md)
- [CALIB_UNIFIED_USAGE.md](CALIB_UNIFIED_USAGE.md)
- [README.md](calib_unified/README.md)
