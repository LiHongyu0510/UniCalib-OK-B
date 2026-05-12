# UniCalib C++ 编译/部署/运行说明

**本项目以本 C++ 工程为主框架进行编译和运行，不使用 UniCalib Python。**

## 环境要求

- **OS**：Linux（推荐 Ubuntu 20.04+）
- **编译器**：支持 C++17 的 g++/clang++
- **CMake**：≥ 3.14
- **Eigen3**：≥ 3.3（必需）
- **yaml-cpp**：可选，用于 YAML 配置解析

## 依赖安装

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install build-essential cmake libeigen3-dev
sudo apt install libyaml-cpp-dev   # 可选，无则编译时加 -DUNICALIB_USE_YAML=OFF
```

## 构建

```bash
cd unicalib_C_plus_plus
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

无 yaml-cpp 时：

```bash
cmake -DUNICALIB_USE_YAML=OFF ..
make -j$(nproc)
```

## 安装（可选）

```bash
cd build
sudo make install
# 默认安装到 /usr/local：lib、include/unicalib、bin
```

## 运行

```bash
cd build

# 仅打印配置中的传感器与标定对（不需数据目录）
./unicalib_example

# 指定配置与数据目录
./unicalib_example /path/to/sensors.yaml /path/to/data

# 运行完整标定流水线
RUN_PIPELINE=1 ./unicalib_example config/sensors.yaml /path/to/data
```

## 配置说明

- **config/sensors.yaml**：传感器列表与 system.output_dir 等；格式见仓库内示例。
- **数据目录（目录模式）**：
  - IMU：根目录下 `imu.csv` 或 `<topic名>.csv`，列：`timestamp,gx,gy,gz,ax,ay,az`（单位：秒、rad/s、m/s²）。
  - 图像/点云：按 topic 对应子目录放置（见 README）。

## 验证步骤

1. **仅配置**：`./unicalib_example` 应输出传感器列表、标定对与一段几何变换示例，退出码 0。
2. **有数据目录**：`RUN_PIPELINE=1 ./unicalib_example config/sensors.yaml <data_dir>` 应完成流水线并在 `output_dir`（默认 `./calib_results`）下生成 `intrinsics.yaml`、`extrinsics.yaml`、`validation_report.yaml` 及 HTML 报告。
3. **异常路径**：数据目录不存在时应收到 `DataException` 及错误信息；数据不足或质量不通过时应收到 `ValidationException` / `CalibrationException`，并在日志中看到对应说明。

## 常见故障排查（Runbook）

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 编译报错找不到 Eigen3 | 未安装或 CMake 未找到 | `sudo apt install libeigen3-dev`，或设置 `Eigen3_DIR` |
| 编译报错 yaml-cpp | 未安装 yaml-cpp | 安装 `libyaml-cpp-dev` 或 `cmake -DUNICALIB_USE_YAML=OFF ..` |
| 运行报错「Data path does not exist」 | 数据目录不存在或路径错误 | 检查传入的 data_path 存在且为目录 |
| 运行报错「IMU validation failed」 | 样本数/采样率/范围等不满足 | 增加数据量、检查 CSV 格式与单位，或放宽 `DataValidationConfig` |
| 运行报错「calibration quality check failed」 | 内参结果超出合理范围 | 检查 IMU 数据质量与采集方式，或调整 `CalibrationQualityConfig` |
| 无 YAML/报告输出 | 无写权限或磁盘满 | 检查 output_dir 可写、磁盘空间，查看日志中的 IO 错误 |

## 日志与观测

- 日志默认输出到 stdout/stderr（ERROR 及以上到 stderr）。
- 可在代码中调用 `unicalib::Logger::instance().init("unicalib", level, log_file_path)` 将日志同时写入文件。
- 使用 `PERF_TIMER("name")` 可对代码块打点计时并输出到日志。
