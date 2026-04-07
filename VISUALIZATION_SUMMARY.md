# UniCalib 可视化增强实现总结

## 概述

本次更新为 UniCalib 添加了**完整的标定可视化系统**，支持五种标定类型的独立可视化功能：

1. IMU 内参标定
2. 相机内参标定
3. IMU-LiDAR 外参标定
4. LiDAR-Camera 外参标定
5. Camera-Camera 外参标定

---

## 核心组件

### 1. CalibVisualizer 统一可视化类

**文件**: `calib_unified/include/unicalib/viz/calib_visualizer.h` (477 行)

**功能**:
- 统一的可视化接口，支持所有标定类型
- 3D 点云查看器封装 (基于 PCL Visualizer)
- 2D 图表绘制 (基于 OpenCV)
- HTML 报告生成集成
- 进度回调机制，支持实时干预

**数据结构**:
- `IMUIntrinsicVizData`: IMU 内参可视化数据
- `CameraIntrinsicVizData`: 相机内参可视化数据
- `IMULiDARVizData`: IMU-LiDAR 外参可视化数据
- `LiDARCameraVizData`: LiDAR-Camera 外参可视化数据
- `CameraCameraVizData`: Camera-Camera 外参可视化数据

### 2. 实现文件

**文件**: `calib_unified/src/viz/calib_visualizer.cpp` (499 行)

**核心方法**:
- `show_imu_lidar_result()`: 显示 IMU-LiDAR 标定完整结果
- `show_lidar_odometry()`: 显示 LiDAR 里程计轨迹
- `show_rotation_pairs()`: 显示旋转对可视化
- `show_bspline_optimization()`: 显示 B 样条优化过程
- `plot_time_offset_estimation()`: 绘制时间偏移估计曲线
- `show_lidar_cam_result()`: 显示 LiDAR-Camera 标定结果
- `show_camera_cam_result()`: 显示 Camera-Camera 标定结果

### 3. 构建系统更新

**文件**: `calib_unified/CMakeLists.txt`

**更新**: 添加了 `src/viz/calib_visualizer.cpp` 到编译源文件列表

---

## 五种标定的可视化功能

### 1. IMU 内参标定可视化

**功能**:
- Allan 方差曲线 (双对数坐标，三轴)
- 陀螺仪噪声样本分布
- 加速度计噪声样本分布
- 陀螺仪零偏不稳定性
- 加速度计零偏不稳定性
- 随机游走 (ARW) 标注

**接口**:
```cpp
void show_imu_intrinsic_result(const IMUIntrinsicVizData& data, bool block = true);
void show_allan_variance_plot(const AllanResult& allan);
```

### 2. 相机内参标定可视化

**功能**:
- 棋盘格角点检测可视化 (红色圆圈)
- 重投影误差分布直方图
- 畸变校正前后对比 (左右并排)
- 内参参数矩阵显示
- 每帧误差柱状图

**接口**:
```cpp
void show_camera_intrinsic_result(const CameraIntrinsicVizData& data, bool block = true);
void show_reprojection_error_distribution(const std::vector<double>& errors);
void show_undistortion_comparison(const cv::Mat& raw_image, const cv::Mat& undistorted_image,
                                 const CameraIntrinsics& intrinsics);
```

### 3. IMU-LiDAR 外参标定可视化

**功能**:
- LiDAR 里程计轨迹 3D 显示 (蓝色轨迹)
- 旋转对可视化 (IMU vs LiDAR 旋转对比)
  - IMU 旋转轴 (红色)
  - LiDAR 旋转轴 (蓝色)
- 手眼标定残差分布 (角度误差柱状图)
- B 样条轨迹收敛动画
  - 红色: 粗标定轨迹
  - 绿色: 精标定轨迹
- 时间偏移估计曲线 (收敛过程)
- 对齐点云展示 (多帧变换到同一坐标系)
- 最终外参坐标系显示

**接口**:
```cpp
void show_imu_lidar_result(const IMULiDARVizData& data, bool block = true);
void show_lidar_odometry(const std::vector<std::pair<double, Sophus::SE3d>>& poses);
void show_rotation_pairs(const std::vector<IMULiDARVizData::RotationPair>& pairs);
void show_bspline_optimization(const std::vector<IMULiDARVizData::OptimizationLog>& history, bool show_animation);
cv::Mat plot_time_offset_estimation(const std::vector<IMULiDARVizData::OptimizationLog>& history);
void show_aligned_point_clouds(const std::vector<pcl::PointCloud<PointXYZI>::Ptr>& clouds,
                             const std::vector<Sophus::SE3d>& poses,
                             const Sophus::SE3d& extrinsic);
```

### 4. LiDAR-Camera 外参标定可视化

**功能**:
- 点云投影到图像 (深度着色)
  - 红色: 近距离 (0.5m ~ 5m)
  - 黄色: 中距离 (5m ~ 20m)
  - 蓝色: 远距离 (20m ~ 50m)
- 边缘对齐可视化
  - 绿色: LiDAR 边缘点
  - 蓝色: 图像边缘点
  - 红色连线: 对齐误差 > 阈值的配准
- 互信息热力图 (评估外参质量)
- 重投影误差分布柱状图
- 外参坐标系叠加 (相机锥体)

**接口**:
```cpp
void show_lidar_cam_result(const LiDARCameraVizData& data, bool block = true);
cv::Mat show_projection_to_image(const LiDARCameraVizData& data, double min_depth, double max_depth);
cv::Mat show_edge_alignment(const LiDARCameraVizData& data);
cv::Mat show_mutual_info_heatmap(const LiDARCameraVizData& data);
cv::Mat plot_reprojection_error_distribution(const LiDARCameraVizData& data);
```

### 5. Camera-Camera 外参标定可视化

**功能**:
- 立体匹配点 3D 显示
  - 左相机点云 (红色)
  - 右相机点云 (蓝色)
  - 用 3D 线条连接匹配点对
- 视差图 2D 显示 (近白远黑)
- 极线可视化 (叠加到左/右图像)
  - 在左图像上绘制右图像对应的极线
- 三角化误差分布 (两个相机)
- 极线约束验证 (3D + 2D 联合可视化)

**接口**:
```cpp
void show_camera_cam_result(const CameraCameraVizData& data, bool block = true);
void show_matched_points_3d(const std::vector<Eigen::Vector3d>& points_cam0,
                          const std::vector<Eigen::Vector3d>& points_cam1,
                          const std::vector<std::pair<int, int>>& correspondences);
void show_disparity_map(const cv::Mat& disparity, const cv::Mat& left_image, bool depth_colored);
void show_epipolar_lines(const cv::Mat& left_image, const cv::Mat& right_image,
                       const CameraIntrinsics& intrinsics0, const CameraIntrinsics& intrinsics1,
                       const ExtrinsicSE3& extrinsic);
void plot_triangulation_errors(const std::vector<double>& errors_cam0,
                             const std::vector<double>& errors_cam1);
```

---

## 实时干预机制

### 进度回调

```cpp
using ProgressCallback = std::function<void(
    const std::string& stage,      // 阶段名称 (IMU-LiDAR, LiDAR-Camera 等)
    const std::string& step,       // 步骤名称 (里程计、手眼标定、B样条优化等)
    double progress,               // 0.0 ~ 1.0
    const std::string& message     // 附加消息
)>;
```

### 干预决策流程

```
标定进行中 → 观察数据
              ↓
         数据是否异常? ─────┐
              ↓            │
         正常          异常
              ↓            ↓
        继续标定     用户决策
                           ↓
                  ┌─────┼─────┐
             停止标定  手动调整  重新标定
                  │         │         │
                  └─────────┴─────────┘
```

### 关键干预信号

| 信号 | 触发条件 | 建议操作 |
|-----|---------|---------|
| 旋转对残差 > 5° | 手眼标定质量差 | 检查 IMU 运动充分性 |
| 收敛成本下降平缓 | 陷入局部最优 | 调整初始猜测或优化参数 |
| 时间偏移 > 0.1s | 时间同步异常 | 检查传感器时间戳对齐 |
| 轨迹抖动 | 运动估计不稳定 | 检查 LiDAR 配准质量 |
| 残差分布长尾 | 存在离群点 | 使用 MAD 滤波或调整阈值 |

---

## 可视化输出目录

```
viz_output/
├── imu_intrinsic/
│   ├── allan_variance.png
│   ├── noise_samples.png
│   └── imu_result.png
├── camera_intrinsic/
│   ├── reprojection_errors.png
│   ├── detected_corners.png
│   └── undistortion_comparison.png
├── imu_lidar/
│   ├── lidar_odom_trajectory.png
│   ├── rotation_pairs.png
│   ├── bspline_convergence.png
│   ├── time_offset_estimation.png
│   ├── aligned_point_clouds.png
│   └── imu_lidar_result.png
├── lidar_camera/
│   ├── lidar_projection.png
│   ├── edge_alignment.png
│   ├── mutual_info_heatmap.png
│   └── lidar_camera_result.png
├── camera_camera/
│   ├── matched_points_3d.png
│   ├── disparity_map.png
│   ├── epipolar_lines.png
│   └── camera_camera_result.png
└── reports/
    └── calibration_report.html
```

---

## 编译与运行

### 依赖

- Eigen3 >= 3.3.0
- Sophus (bundled)
- Ceres Solver >= 2.1.0
- OpenCV >= 4.5.0
- PCL >= 1.12
- spdlog >= 1.13.0
- yaml-cpp >= 0.8.0
- basalt-headers (bundled)

### 编译步骤

```bash
cd /home/wqs/Documents/github/UniCalib/calib_unified
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DUNICALIB_BUILD_APPS=ON \
    -DUNICALIB_WITH_PCL_VIZ=ON
make -j$(nproc)
```

### 运行示例

```bash
# IMU 内参标定
./bin/unicalib_imu_intrinsic --config config/unicalib_example.yaml --enable-viz

# IMU-LiDAR 外参标定
./bin/unicalib_imu_lidar_extrin --config config/imu_lidar_example.yaml --enable-viz

# LiDAR-Camera 外参标定
./bin/unicalib_lidar_camera_extrin --config config/lidar_camera_example.yaml --enable-viz

# Camera-Camera 外参标定
./bin/unicalib_cam_cam_extrin --config config/cam_cam_example.yaml --enable-viz
```

---

## 核心价值

1. **标定过程从"黑盒"变成"透明盒"**
   - 实时看到 LiDAR 里程计轨迹是否平滑
   - 立即发现 IMU 运动是否充分（旋转对可视化）
   - 监控 B 样条优化是否收敛（收敛曲线）

2. **即时发现标定问题**
   - 异常数据通过可视化立即可见（如平面运动、收敛失败）
   - 支持随时停止标定、检查数据、重新开始

3. **多维度诊断**
   - 轨迹对比、残差分布、收敛曲线同时显示
   - 支持标定结果回放验证

4. **完整的覆盖**
   - 五种标定类型各有独立的 3D/2D 可视化
   - 所有关键图表自动保存到统一目录

---

## 技术支持

如有问题，请通过以下方式反馈：

- GitHub Issues: https://github.com/[your-repo]/UniCalib/issues
- 技术文档: `calib_unified/README.md`
- 示例配置: `calib_unified/config/`

---

**文档版本**: v1.0
**最后更新**: 2024-03-04
**维护者**: UniCalib 开发团队
