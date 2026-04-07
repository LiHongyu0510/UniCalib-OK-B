# UniCalib C++（本项目主框架）

**本项目的编译与运行均在此 C++ 工程完成，不使用 [UniCalib](../UniCalib) Python 作为主运行框架。**  
本仓库为 UniCalib 多传感器标定系统的 C++ 实现，由 UniCalib Python 设计/代码转换而来；Python 版本仅作参考与对照，不参与主流程构建与运行。

## 目录结构

```
unicalib_C_plus_plus/
├── CMakeLists.txt
├── README.md
├── config/
│   └── sensors.yaml          # 示例传感器配置
├── include/unicalib/
│   ├── sensor_config.hpp     # 传感器配置与标定对
│   ├── calib_result.hpp      # 内参/外参/验证结果
│   ├── transforms.hpp        # 几何变换 (欧拉角、四元数、SE3)
│   ├── allan_variance.hpp    # IMU Allan 方差分析
│   ├── data_manager.hpp      # 数据管理 (目录/CSV)
│   ├── imu_intrinsic.hpp     # IMU 内参标定 (Allan + 六面法)
│   ├── reprojection.hpp      # 重投影验证 (占位接口)
│   ├── report_generator.hpp  # 报告生成 (终端 + HTML)
│   ├── system.hpp            # 系统主控
│   ├── exceptions.hpp        # 统一异常与错误码
│   ├── logger.hpp            # 日志 (无额外依赖)
│   ├── data_validator.hpp    # 数据质量验证
│   └── calibration_quality.hpp # 标定质量评估
├── src/
│   ├── sensor_config.cpp
│   ├── calib_result.cpp
│   ├── transforms.cpp
│   ├── allan_variance.cpp
│   ├── data_manager.cpp
│   ├── imu_intrinsic.cpp
│   ├── reprojection.cpp
│   ├── report_generator.cpp
│   ├── system.cpp
│   ├── exceptions.cpp
│   ├── logger.cpp
│   ├── data_validator.cpp
│   └── calibration_quality.cpp
└── examples/
    └── example_main.cpp      # 示例入口
```

## 依赖

- **CMake** (≥3.14)
- **Eigen3** (≥3.3)：必需，矩阵/向量运算（如 `sudo apt install libeigen3-dev`）
- **yaml-cpp**：可选，用于解析 YAML 配置（如 `sudo apt install libyaml-cpp-dev`）；未安装时使用默认配置仍可编译

## 编译

```bash
cd unicalib_C_plus_plus
mkdir build && cd build
cmake ..
make
```
若未安装 CMake：`sudo apt install cmake`（或 `sudo apt install build-essential cmake`）。

若系统未安装 yaml-cpp，可关闭 YAML 支持后编译：

```bash
cmake -DUNICALIB_USE_YAML=OFF ..
make
```

## 运行

```bash
# 仅打印传感器与标定对（不跑流水线）
./unicalib_example

# 指定配置与数据目录
./unicalib_example /path/to/sensors.yaml /path/to/data

# 运行完整流水线（需提供数据目录）
RUN_PIPELINE=1 ./unicalib_example config/sensors.yaml /path/to/data
```

数据目录需包含（目录模式）：

- 图像：`<topic 对应子目录>/*.jpg` 等
- 点云：`<topic 对应子目录>/*.pcd` 等
- IMU：`imu.csv` 或 `<topic 名>.csv`，列：`timestamp,gx,gy,gz,ax,ay,az`

## 与 Python 的对应关系

| Python (UniCalib)           | C++ (本仓库)        |
|----------------------------|---------------------|
| `core/sensor_config.py`    | `sensor_config.hpp/cpp` |
| `core/calib_result.py`     | `calib_result.hpp/cpp`  |
| `utils/transforms.py`     | `transforms.hpp/cpp`   |
| `intrinsic/allan_variance.py` | `allan_variance.hpp/cpp` |
| `intrinsic/imu_intrinsic.py`  | `imu_intrinsic.hpp/cpp` (Allan+六面法，无 iKalibr) |
| `validation/reprojection.py` | `reprojection.hpp/cpp` (占位) |
| `validation/report_generator.py` | `report_generator.hpp/cpp` |
| `core/data_manager.py`    | `data_manager.hpp/cpp` (目录+CSV) |
| `core/system.py`          | `system.hpp/cpp` (含保存 YAML + 报告) |

## 工程化与产品化（异常处理、精度与可维护性）

- **统一异常与错误码**：`exceptions.hpp/cpp` 提供 `UniCalibException`、`ConfigException`、`DataException`、`ValidationException`、`CalibrationException`、`IOException` 及 `ErrorCode` 枚举，便于上层捕获与日志。
- **日志**：`logger.hpp/cpp` 基于 iostream，支持 TRACE/DEBUG/INFO/WARNING/ERROR/CRITICAL、可选文件输出、`PERF_TIMER` 计时、异常日志。
- **数据质量验证**：`DataValidator` 在 IMU 标定前检查样本数、时长、采样率、时间戳单调性、陀螺/加速度范围；可通过 `DataValidationConfig` 配置阈值（默认不强制时长）。
- **标定质量评估**：`CalibrationQualityChecker` 对 IMU 内参结果做合理性范围检查，不合格时抛出 `CalibrationException`。
- **流水线行为**：系统主控在 `run_full_pipeline` / `run_stage` 中统一 try/catch，失败时记录日志并重新抛出；`DataManager::open()` 在数据路径不存在时抛 `DataException`；IMU CSV 解析跳过异常行并打日志。
- **标定精度与稳定性**：IMU 阶段在传入 `DataValidator` 与 `CalibrationQualityChecker` 时，数据不足或质量不通过会抛异常而非静默回退，便于定位与复现。

## 当前实现状态

- **已实现**：传感器/标定对配置、内参/外参/验证数据结构、几何变换、Allan 方差、**IMU 内参标定（Allan+六面法）**、目录模式数据与 CSV IMU、**报告生成（终端汇总+HTML+YAML 保存）**、系统配置解析与保存流程；**统一异常、日志、数据验证与标定质量检查**。
- **占位**：重投影验证（接口已有，无图像时返回占位指标）、Stage2/3/4 外参。
- **已实现**：鱼眼相机内参标定（系统必备能力）：`camera_fisheye.hpp/cpp`，可选 OpenCV 时使用 `cv::fisheye::calibrate`（equidistant 模型）；Python 端支持 EUCM/DS/Kannala-Brandt/Equidistant 多模型选优。
- **深度融合（子进程调用）**：**DM-Calib**（Stage1 针孔内参）、**learn-to-calibrate**（Stage2 IMU-LiDAR 粗外参）已接入；配置 `third_party` 或环境变量 `UNICALIB_DM_CALIB` / `UNICALIB_LEARN_TO_CALIB`。详见 `DEEP_INTEGRATION.md`。
- **未实现**：rosbag2、iKalibr/MIAS-LCEC/click_calib 子进程调用、针孔 OpenCV 精化、着色/边缘验证。后续可按需扩展。

## 验证

构建成功后运行示例应能列出配置中的传感器与标定对，并打印一段几何变换示例，无需数据目录。
