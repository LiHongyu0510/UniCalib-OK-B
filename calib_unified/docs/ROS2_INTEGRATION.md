# ROS2 话题订阅支持

## Executive Summary

为 UniCalib 标定工程添加了 ROS2 话题订阅功能，支持从 ROS2 bag 文件或实时话题订阅 LiDAR 点云和相机图像数据进行标定。这项改进消除了需要预先将 bag 转换为 PCD 和图像文件的限制，实现了真正的 ROS2 兼容。

---

## 1. 背景 & 目标

### 问题
- 原 UniCalib 仅支持从文件（PCD 点云 + 图像文件）加载数据
- 用户需要先从 ROS2 bag 导出 PCD 和图像，再进行标定
- 对于在线标定场景，需要额外的数据转换步骤

### 目标
- 支持直接从 ROS2 bag 文件读取数据
- 支持实时 ROS2 话题订阅（在线标定）
- 保持向后兼容（文件模式仍可用）
- 最小化代码侵入性

---

## 2. 功能概述

### 2.1 数据源类型

UniCalib 现支持三种数据源：

```cpp
enum class DataSourceType {
    FILES,      // PCD 文件 + 图像文件 (原有)
    ROS2_BAG,   // ROS2 bag 文件 (新增)
    ROS2_TOPIC  // ROS2 实时话题订阅 (新增)
};
```

### 2.2 支持的 ROS2 消息类型

| 传感器类型 | ROS2 消息类型 | 话题示例 |
|-----------|----------------|----------|
| LiDAR | `sensor_msgs/PointCloud2` | `/velodyne_points` |
| 相机 | `sensor_msgs/Image` | `/cam_left/image_raw` |
| IMU | `sensor_msgs/Imu` | `/imu/data` |

### 2.3 关键特性

- **Bag 文件模式**：支持 ROS2 sqlite3 格式 bag 文件
- **实时模式**：使用 rclcpp 订阅话题，支持在线标定
- **数据过滤**：支持距离范围、NaN 点过滤、时间窗口过滤
- **采样控制**：可配置采样间隔和最大帧数限制
- **话题映射**：支持多个传感器 ID 到话题的映射

---

## 3. 方案设计

### 3.1 架构设计

```
┌─────────────────────────────────────────────────────────────────┐
│                  UnifiedDataLoader                        │
│  ┌────────────┬─────────────┬───────────────┐        │
│  │ 文件模式  │ Bag 模式   │ 实时模式       │        │
│  └────────────┴─────────────┴───────────────┘        │
└─────────────────────────────────────────────────────────────────┘
         │              │                │
         ▼              ▼                ▼
┌───────────────────┐  ┌──────────────┐  ┌─────────────────┐
│  FileLoader      │  │  Ros2Bag    │  │  Ros2Realtime  │
│  (PCL + OpenCV)  │  │  DataSource  │  │   DataSource   │
└───────────────────┘  └──────────────┘  └─────────────────┘
                              │                │
                              ▼                ▼
                        ┌──────────────────────────┐
                        │   RosDataSourceBase    │
                        │   (统一接口)          │
                        └──────────────────────────┘
```

### 3.2 核心类

#### UnifiedDataLoader
统一的加载数据接口，支持文件和 ROS2 两种模式：

```cpp
class UnifiedDataLoader {
public:
    enum class SourceType {
        FILES, ROS2_BAG, ROS2_TOPIC
    };
    
    struct Config {
        SourceType source_type = SourceType::FILES;
        
        // 文件模式配置
        std::string lidar_data_dir;
        std::string camera_images_dir;
        
        // ROS2 模式配置
        RosDataSourceConfig ros_config;
        
        // 通用参数
        size_t max_frames = 100;
        double sample_interval = 0.0;
    };
    
    // 统一接口
    std::vector<LiDARScanRos> get_lidar_scans(const std::string& sensor_id);
    std::vector<CameraFrameRos> get_camera_frames(const std::string& sensor_id);
    
    // 转换为标定器所需格式
    std::vector<LiDARScan> to_lidar_scans(const std::string& sensor_id);
    std::vector<std::pair<double, cv::Mat>> to_camera_frames(const std::string& sensor_id);
};
```

#### Ros2BagDataSource
从 ROS2 bag 文件读取数据：

```cpp
class Ros2BagDataSource : public RosDataSourceBase {
private:
    std::unique_ptr<rosbag2_cpp::Reader> bag_reader_;
    std::map<std::string, std::vector<LiDARScanRos>> lidar_data_;
    std::map<std::string, std::vector<CameraFrameRos>> camera_data_;
    std::map<std::string, std::vector<IMUFrameRos>> imu_data_;
    
public:
    bool load();  // 遍历 bag，收集所有消息
    std::vector<LiDARScanRos> get_lidar_scans(const std::string& sensor_id);
};
```

#### Ros2RealtimeDataSource
实时订阅 ROS2 话题：

```cpp
class Ros2RealtimeDataSource : public RosDataSourceBase {
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<rclcpp::exec::SingleThreadedExecutor> executor_;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    std::mutex data_mutex_;
    std::map<std::string, std::queue<LiDARScanRos>> lidar_queues_;
    std::atomic<bool> running_{false};
    
public:
    bool start();  // 创建节点和订阅
    void stop();   // 清理 ROS2 资源
    bool wait_for_data(double timeout_sec);  // 等待数据就绪
};
```

---

## 4. 使用方法

### 4.1 命令行参数

#### 文件模式（原有）

```bash
./unicalib_lidar_camera \
    --config calib_unified/config/unicalib_example.yaml \
    --data-dir /path/to/data
```

#### ROS2 Bag 模式（新增）

```bash
./unicalib_lidar_camera \
    --config calib_unified/config/unicalib_example.yaml \
    --ros2-bag /path/to/data.bag
```

#### ROS2 实时话题模式（新增）

```bash
# 启动 ROS2 节点和发布 bag
ros2 bag play data.bag --topics /velodyne_points /cam_left/image_raw &

# 启动标定（订阅话题）
./unicalib_lidar_camera \
    --config calib_unified/config/unicalib_example.yaml \
    --ros2-topic \
    --lidar-topic /velodyne_points \
    --camera-topic /cam_left/image_raw
```

### 4.2 YAML 配置文件

在 `unicalib_example.yaml` 中添加 ROS2 配置：

```yaml
ros2:
  # 使用 ROS2 bag 文件
  use_ros2_bag: false
  ros2_bag_file: /path/to/data.bag
  
  # 使用 ROS2 实时话题订阅
  use_ros2_topics: false
  
  # ROS2 话题名称
  lidar_topic: /velodyne_points
  camera_topic: /cam_left/image_raw
  imu_topic: /imu/data
  
  # ROS2 数据采集参数
  max_wait_time: 30.0      # 实时模式最大等待时间(秒)
  sample_interval: 0.0      # 数据采样间隔(秒, 0=不跳过)
  max_frames: 100          # 最大帧数限制 (0=无限制)
```

### 4.3 PipelineConfig 新增字段

```cpp
struct PipelineConfig {
    // ... 原有字段 ...
    
    // ROS2 数据源配置 (新增)
    bool use_ros2_bag = false;
    std::string ros2_bag_file;
    bool use_ros2_topics = false;
    std::string lidar_ros2_topic;
    std::string camera_ros2_topic;
    std::string imu_ros2_topic;
    double ros2_max_wait_time = 30.0;
    double ros2_sample_interval = 0.0;
    size_t ros2_max_frames = 100;
};
```

---

## 5. 代码变更

### 5.1 新增文件

| 文件路径 | 说明 |
|---------|------|
| `include/unicalib/io/ros2_data_source.h` | ROS2 数据源头文件 |
| `src/io/ros2_data_source.cpp` | ROS2 数据源实现 |

### 5.2 修改文件

| 文件路径 | 修改内容 |
|---------|---------|
| `include/unicalib/pipeline/calib_pipeline.h` | 添加 ROS2 配置字段 |
| `src/pipeline/calib_pipeline.cpp` | 支持从 ROS2 加载数据 |
| `apps/lidar_camera_extrin/main.cpp` | 添加 ROS2 命令行参数 |
| `config/unicalib_example.yaml` | 添加 ROS2 配置示例 |

---

## 6. 编译 & 部署

### 6.1 编译

```bash
cd calib_unified/build

# 启用 ROS2 支持
cmake -DUNICALIB_WITH_ROS2=ON ..
make -j$(nproc)
```

### 6.2 依赖项

ROS2 模式需要以下依赖：

- **ros2bag2_cpp** (>= 0.6.0)
- **rosbag2_storage** (>= 0.6.0)
- **rclcpp** (ROS2 Humble)
- **sensor_msgs** (ROS2)
- **cv_bridge** (ROS2)
- **pcl_conversions** (ROS2)

安装命令：

```bash
sudo apt-get install -y \
    ros-humble-ros2bag2-cpp \
    ros-humble-rosbag2-storage \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions
```

### 6.3 Docker 环境使用

在 Docker 容器中使用：

```bash
# 宿主机上运行
./calib_unified_run.sh --run --task lidar-cam \
    --dataset nya_02_ros2 \
    --ros2-bag /root/calib_ws/data/nya_02.bag
```

容器内已安装 ROS2 环境，可直接使用。

---

## 7. 验证 & 测试

### 7.1 单元测试

```bash
# 测试 bag 文件加载
./unicalib_lidar_camera --ros2-bag test_data.bag --config test_config.yaml

# 测试实时话题订阅
ros2 bag play test_data.bag --topics /velodyne_points /cam_left/image_raw &
./unicalib_lidar_camera --ros2-topic --lidar-topic /velodyne_points --camera-topic /cam_left/image_raw
```

### 7.2 集成测试

```bash
# 1. 检查话题列表
ros2 bag info data.bag

# 2. 使用 bag 模式标定
./unicalib_lidar_camera \
    --config calib_unified/config/unicalib_example.yaml \
    --ros2-bag data.bag

# 3. 检查输出
ls calib_output/lidar_camera/
```

### 7.3 性能指标

| 指标 | 目标值 |
|------|--------|
| Bag 加载速度 | > 100 Hz (点云) |
| 话题延迟 | < 10 ms |
| 内存占用 | < 2 GB (100 帧) |
| 实时模式启动时间 | < 2 秒 |

---

## 8. 故障排查

### 8.1 常见问题

#### 问题1：未找到话题

**现象**：
```
ERROR [Ros2RealtimeDataSource] 话题订阅失败
```

**原因**：话题名称不匹配

**解决**：
```bash
# 查看所有话题
ros2 topic list

# 或查看 bag 中的话题
ros2 bag info data.bag
```

#### 问题2：时间戳对齐失败

**现象**：
```
WARN [Fine-Auto] LiDAR 和相机时间戳差异过大
```

**原因**：时间戳同步问题

**解决**：
- 检查 IMU 时间同步
- 使用 `--time-sync` 参数（如果支持）
- 调整 `ros2_max_wait_time` 参数

#### 问题3：数据不足

**现象**：
```
WARN [Ros2RealtimeDataSource] 收集数据不足
```

**原因**：bag 数据太短或实时模式超时

**解决**：
- 增加 `ros2_max_wait_time`
- 检查话题发布频率
- 使用 `--max-frames` 参数降低帧数要求

### 8.2 日志级别

```bash
--log-level trace    # 最详细，用于调试
--log-level debug    # 调试信息
--log-level info     # 正常运行信息
--log-level warn     # 仅警告
--log-level error    # 仅错误
```

---

## 9. 后续演进

### 9.1 MVP (当前版本)

- ✅ 支持 ROS2 bag 文件读取
- ✅ 支持实时话题订阅
- ✅ 支持 PointCloud2 消息
- ✅ 支持 Image 消息
- ✅ 支持基本数据过滤

### 9.2 V1 (下一步)

- [ ] 支持更多消息类型（CompressedImage, Imu）
- [ ] 支持多传感器同步
- [ ] 添加数据可视化工具
- [ ] 支持时间偏移标定
- [ ] 添加实时标定 GUI

### 9.3 V2 (长期)

- [ ] 支持 ROS1 rosbag（向后兼容）
- [ ] 支持 ROS2 录制到新 bag
- [ ] 集成 ROS2 launch 文件
- [ ] 支持分布式标定
- [ ] 添加性能监控和 Profiling

---

## 10. 参考资源

### 10.1 文档链接

- ROS2 2.0 文档: https://docs.ros.org/en/humble/
- rosbag2_cpp 文档: https://github.com/ros2/rosbag2
- UniCalib 主文档: `CALIB_UNIFIED_USAGE.md`

### 10.2 示例配置

完整的配置示例参见：
- `calib_unified/config/unicalib_example.yaml`
- `calib_unified/config/unicalib_ros2_example.yaml` (待创建)

---

## 11. 观测性 & 运维

### 11.1 日志记录

所有 ROS2 相关操作都有详细日志：

```
[Ros2BagDataSource] 打开 bag 文件: data.bag
[Ros2BagDataSource] 找到 5 个话题
[Ros2BagDataSource] 收集 LiDAR lidar_front 数据, 帧数: 50
[Ros2BagDataSource] 收集相机 cam_left 数据, 帧数: 50
```

### 11.2 性能监控

运行时自动记录：

- 数据加载时间
- 消息处理速度
- 内存使用情况

### 11.3 告警

- Bag 文件损坏
- 话题订阅失败
- 数据采集超时
- 内存溢出风险

---

## 12. 安全 & 最佳实践

### 12.1 安全考虑

- ✅ 不修改原始 bag 文件
- ✅ 线程安全的数据访问（mutex 保护）
- ✅ 优雅的资源清理（RAII）
- ✅ 超时保护（防止死锁）

### 12.2 最佳实践

1. **先使用小数据集测试**
   ```bash
   # 使用 bag 中的前 1000 条消息测试
   ros2 bag play data.bag --duration 10
   ```

2. **检查数据质量**
   - 点云无 NaN
   - 图像无模糊
   - 时间戳连续

3. **合理设置采样参数**
   - `max_frames`: 根据内存调整
   - `sample_interval`: 避免数据冗余

4. **使用合适的日志级别**
   - 开发时用 `--log-level debug`
   - 生产时用 `--log-level info`

---

## 附录 A: 示例配置文件

### A.1 ROS2 Bag 模式配置

```yaml
# unicalib_ros2_bag.yaml
output_dir: ./calib_output/lidar_camera_ros2

ros2:
  use_ros2_bag: true
  ros2_bag_file: /path/to/calibration.bag
  lidar_topic: /velodyne_points
  camera_topic: /cam_left/image_raw
  max_frames: 100
  sample_interval: 0.1  # 每 100ms 采样一帧

lidar_camera:
  method: edge
  board_cols: 9
  board_rows: 6
  square_size: 0.025
  edge_canny_low: 50
  edge_canny_high: 150
```

### A.2 实时话题模式配置

```yaml
# unicalib_ros2_realtime.yaml
output_dir: ./calib_output/lidar_camera_realtime

ros2:
  use_ros2_topics: true
  lidar_topic: /velodyne_points
  camera_topic: /cam_left/image_raw
  max_wait_time: 60.0  # 等待 60 秒采集数据
  max_frames: 50
  sample_interval: 0.0

lidar_camera:
  method: edge
  prefer_targetfree: true
```

---

## 附录 B: 故障排查清单

### B.1 预检查清单

- [ ] ROS2 环境已正确安装（source /opt/ros/humble/setup.bash）
- [ ] 依赖包已安装（rosbag2_cpp, cv_bridge, pcl_conversions）
- [ ] Bag 文件存在且可读
- [ ] 话题名称正确（ros2 topic list / ros2 bag info）
- [ ] 配置文件路径正确
- [ ] 有足够的磁盘空间存储结果

### B.2 运行时检查清单

- [ ] 节点成功创建（rclcpp::init）
- [ ] 订阅成功（没有错误日志）
- [ ] 数据正在接收（查看日志输出）
- [ ] 标定算法正常执行（无异常）
- [ ] 结果文件正确生成（YAML, PNG）

---

**文档版本**: 1.0  
**最后更新**: 2025-03-05  
**维护者**: UniCalib Team
