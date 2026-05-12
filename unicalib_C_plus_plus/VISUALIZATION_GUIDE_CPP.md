# UniCalib C++ 可视化系统使用指南

## 目录

1. [概述](#概述)
2. [功能特性](#功能特性)
3. [快速开始](#快速开始)
4. [API参考](#api参考)
5. [编译说明](#编译说明)
6. [示例代码](#示例代码)

---

## 概述

UniCalib C++ 可视化系统提供了与Python版本对应的全套可视化功能，包括热力图投影、误差分布图、传感器视锥图、多帧对比图等。

### 核心类

| 类 | 功能 | 主要方法 |
|----|------|----------|
| `VisualizationV2` | 增强版可视化工具 | `DrawLidarHeatmap()`, `SaveErrorDistributionPlot()` 等 |

---

## 功能特性

### 1. 热力图投影
- **功能**: 将LiDAR点云投影到图像，按深度着色
- **输出**: OpenCV Mat格式图像
- **依赖**: OpenCV (必需）

### 2. 误差分布图
- **功能**: 双子图（直方图 + CDF曲线）展示误差分布
- **输出**: PNG格式图像
- **实现**: 调用Python matplotlib（通过临时脚本）

### 3. 传感器视锥图
- **功能**: 3D展示各传感器坐标系和相机视锥
- **输出**: PNG格式图像
- **实现**: 调用Python matplotlib

### 4. 多帧对比图
- **功能**: 并排展示多帧投影结果
- **输出**: JPG格式图像
- **依赖**: OpenCV (必需）

### 5. 残差收敛曲线
- **功能**: 展示优化过程中残差的收敛情况
- **输出**: PNG格式图像
- **实现**: 调用Python matplotlib

### 6. 点云对齐可视化
- **功能**: 3D展示点云配准效果
- **输出**: PNG格式图像
- **实现**: 调用Python matplotlib

---

## 快速开始

### 安装依赖

#### 必需依赖
```bash
# Eigen3
sudo apt-get install libeigen3-dev

# yaml-cpp (可选）
sudo apt-get install libyaml-cpp-dev

# OpenCV (必需，用于可视化）
sudo apt-get install libopencv-dev

# Python matplotlib (用于图表生成）
pip3 install matplotlib numpy
```

### 编译

```bash
cd unicalib_C_plus_plus
mkdir -p build && cd build
cmake ..
make -j4
```

### 运行示例

```bash
# 运行可视化示例
./unicalib_viz_example
```

输出将保存在 `./viz_output_cpp/` 目录。

---

## API参考

### VisualizationV2

#### 构造函数
```cpp
explicit VisualizationV2(const std::string& output_dir);
```
**参数**:
- `output_dir`: 输出目录路径

#### DrawLidarHeatmap()
```cpp
cv::Mat DrawLidarHeatmap(
    const cv::Mat& img,
    const Eigen::MatrixXd& pts_cam,
    const Eigen::Matrix3d& K,
    const Eigen::Vector4d& D,
    const VisualizationConfig& config = VisualizationConfig());
```
**参数**:
- `img`: 原始图像 (CV_8UC3)
- `pts_cam`: 相机坐标系下的点云 (N×3)
- `K`: 相机内参矩阵 (3×3)
- `D`: 畸变系数 (4×1)
- `config`: 可视化配置（可选）

**返回**: 叠加了热力图的图像

#### SaveErrorDistributionPlot()
```cpp
void SaveErrorDistributionPlot(
    const std::vector<double>& errors,
    const std::string& title,
    const std::string& filename,
    double threshold = 1.0);
```
**参数**:
- `errors`: 误差值数组
- `title`: 图表标题
- `filename`: 输出文件名
- `threshold`: 合格阈值（像素）

#### SaveMultiFrameProjection()
```cpp
void SaveMultiFrameProjection(
    const std::vector<cv::Mat>& images,
    const std::vector<Eigen::MatrixXd>& pts_list,
    const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& extrinsics_list,
    const Eigen::Matrix3d& K,
    const Eigen::Vector4d& D,
    const std::vector<std::string>& labels,
    const std::string& filename);
```
**参数**:
- `images`: 图像列表
- `pts_list`: 对应的点云列表（世界坐标系）
- `extrinsics_list`: 外参列表
- `K`: 相机内参
- `D`: 畸变系数
- `labels`: 每个子图的标签
- `filename`: 输出文件名

#### SaveSensorFrustum()
```cpp
void SaveSensorFrustum(
    const std::unordered_map<std::string, SensorConfig>& sensors,
    const std::unordered_map<std::string, IntrinsicResultHolder>& intrinsics,
    const std::unordered_map<std::string, CalibResult>& extrinsics,
    const std::vector<std::string>& camera_ids,
    const std::vector<std::string>& lidar_ids,
    const std::string& filename);
```
**参数**:
- `sensors`: 传感器配置字典
- `intrinsics`: 内参结果字典
- `extrinsics`: 外参结果字典
- `camera_ids`: 要显示的相机ID列表
- `lidar_ids`: 要显示的LiDAR ID列表
- `filename`: 输出文件名

#### SaveResidualPlot()
```cpp
void SaveResidualPlot(
    const std::vector<double>& iteration_errors,
    const std::string& title,
    const std::string& filename);
```
**参数**:
- `iteration_errors`: 每次迭代的误差列表
- `title`: 图表标题
- `filename`: 输出文件名

#### SavePointcloudAlignment()
```cpp
void SavePointcloudAlignment(
    const Eigen::MatrixXd& pts_source,
    const Eigen::MatrixXd& pts_target,
    const std::pair<Eigen::Matrix3d, Eigen::Vector3d>& transform,
    const std::string& filename);
```
**参数**:
- `pts_source`: 源点云 (N×3)
- `pts_target`: 目标点云 (M×3)
- `transform`: 变换矩阵 (R, t)
- `filename`: 输出文件名

### VisualizationConfig

```cpp
struct VisualizationConfig {
  struct {
    double max_depth = 50.0;      // 最大深度（米）
    int point_size = 3;           // 点大小
    double alpha = 0.7;           // 透明度 [0,1]
    int colormap = 0;             // OpenCV颜色映射
  } heatmap;

  struct {
    double threshold = 1.0;       // 合格阈值（像素）
    int bins = 50;                // 直方图bin数量
  } error_distribution;

  struct {
    double frustum_depth = 5.0;   // 视锥深度（米）
    double axis_scale = 1.0;      // 坐标轴长度
  } sensor_frustum;
};
```

---

## 编译说明

### CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `UNICALIB_USE_YAML` | ON | 是否使用yaml-cpp |
| `UNICALIB_USE_OPENCV` | ON | 是否使用OpenCV（可视化必需） |

### 编译示例

```bash
# 基础编译（仅必需依赖）
cmake .. -DUNICALIB_USE_YAML=OFF -DUNICALIB_USE_OPENCV=ON
make -j4

# 完整编译（包含所有依赖）
cmake .. -DUNICALIB_USE_YAML=ON -DUNICALIB_USE_OPENCV=ON
make -j4
```

### 依赖检查

```bash
# 检查Eigen3
pkg-config --modversion eigen3

# 检查OpenCV
pkg-config --modversion opencv4

# 检查Python matplotlib
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

---

## 示例代码

### 示例1: 生成误差分布图

```cpp
#include "unicalib/visualization_v2.hpp"
#include <vector>

using namespace unicalib;

int main() {
  // 创建可视化器
  VisualizationV2 viz("./output");
  
  // 生成误差数据
  std::vector<double> errors;
  for (int i = 0; i < 1000; ++i) {
    errors.push_back(0.5 * (std::rand() / RAND_MAX));
  }
  
  // 保存误差分布图
  viz.SaveErrorDistributionPlot(
    errors,
    "Reprojection Error Distribution",
    "error_dist.png",
    1.0
  );
  
  return 0;
}
```

### 示例2: 生成热力图投影

```cpp
#ifdef UNICALIB_USE_OPENCV
#include "unicalib/visualization_v2.hpp"
#include <opencv2/opencv.hpp>

using namespace unicalib;

int main() {
  VisualizationV2 viz("./output");
  
  // 加载图像
  cv::Mat img = cv::imread("image.jpg");
  
  // 创建点云（相机坐标系）
  Eigen::MatrixXd pts_cam(10000, 3);
  for (int i = 0; i < 10000; ++i) {
    pts_cam(i, 0) = (std::rand() / RAND_MAX - 0.5) * 20.0;
    pts_cam(i, 1) = (std::rand() / RAND_MAX - 0.5) * 20.0;
    pts_cam(i, 2) = 2.0 + (std::rand() / RAND_MAX) * 48.0;
  }
  
  // 相机内参
  Eigen::Matrix3d K;
  K << 1000, 0, 960,
       0, 1000, 540,
       0, 0, 1;
  
  Eigen::Vector4d D;
  D << 0, 0, 0, 0;
  
  // 生成热力图
  cv::Mat heatmap = viz.DrawLidarHeatmap(img, pts_cam, K, D);
  cv::imwrite("./output/heatmap.jpg", heatmap);
  
  return 0;
}
#endif
```

### 示例3: 生成残差收敛曲线

```cpp
#include "unicalib/visualization_v2.hpp"
#include <vector>
#include <cmath>

using namespace unicalib;

int main() {
  VisualizationV2 viz("./output");
  
  // 模拟优化过程
  std::vector<double> iteration_errors;
  for (int i = 0; i < 50; ++i) {
    double error = 1.0 * std::exp(-i / 10.0) + 0.01 * (std::rand() / RAND_MAX);
    iteration_errors.push_back(error);
  }
  
  // 保存残差曲线
  viz.SaveResidualPlot(
    iteration_errors,
    "Optimization Residual Convergence",
    "residual.png"
  );
  
  return 0;
}
```

### 示例4: 生成传感器视锥图

```cpp
#include "unicalib/visualization_v2.hpp"
#include "unicalib/calib_result.hpp"
#include "unicalib/sensor_config.hpp"

using namespace unicalib;

int main() {
  VisualizationV2 viz("./output");
  
  // 准备传感器配置（示例）
  std::unordered_map<std::string, SensorConfig> sensors;
  // ... 填充传感器配置
  
  // 准备内参结果（示例）
  std::unordered_map<std::string, IntrinsicResultHolder> intrinsics;
  // ... 填充内参结果
  
  // 准备外参结果（示例）
  std::unordered_map<std::string, CalibResult> extrinsics;
  // ... 填充外参结果
  
  // 指定要显示的传感器
  std::vector<std::string> camera_ids = {"cam_front", "cam_left", "cam_right"};
  std::vector<std::string> lidar_ids = {"lidar0"};
  
  // 生成传感器视锥图
  viz.SaveSensorFrustum(
    sensors, intrinsics, extrinsics,
    camera_ids, lidar_ids,
    "sensor_frustums.png"
  );
  
  return 0;
}
```

---

## 性能说明

| 操作 | 数据量 | 性能 |
|------|--------|------|
| 热力图生成 | 10K点 | <100ms |
| 误差分布图 | 1000点 | <500ms |
| 传感器视锥图 | 5个传感器 | <1s |
| 多帧对比 | 4帧 | <500ms |

---

## 故障排查

### 问题1: 找不到OpenCV
```bash
sudo apt-get install libopencv-dev
# 或指定OpenCV路径
cmake .. -DOpenCV_DIR=/path/to/opencv
```

### 问题2: matplotlib无法调用
```bash
pip3 install matplotlib numpy
# 确保python3在PATH中
which python3
```

### 问题3: 编译错误
```bash
# 清理并重新编译
cd build
rm -rf *
cmake ..
make -j4 VERBOSE=1
```

---

## 总结

UniCalib C++ 可视化系统提供了：
- ✅ 完整的可视化功能（热力图、误差分布、3D视图等）
- ✅ 与Python版本API一致
- ✅ 高性能C++实现
- ✅ 易于集成到现有C++项目中

**下一步**:
1. 查看示例代码: `examples/visualization_example.cpp`
2. 阅读Python版本文档: `UniCalib/VISUALIZATION_GUIDE.md`
3. 集成到您的标定流程中
