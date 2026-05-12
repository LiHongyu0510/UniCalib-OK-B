# 深度融合改造说明

本文档描述 unicalib_C_plus_plus 与六方项目（DM-Calib、learn-to-calibrate 等）的**子进程调用式**深度融合：通过配置路径、子进程执行外部脚本/可执行文件并解析输出，实现自动化标定。

## 一、已实现的融合

| 阶段 | 项目 | 实现方式 | 配置/环境变量 |
|------|------|----------|-------------------------------|
| **Stage1 内参** | **DM-Calib** | `dm_calib_wrapper`：子进程调用 `python DMCalib/tools/infer.py`，解析 JSON 得到针孔内参 | `third_party.dm_calib` 或 `UNICALIB_DM_CALIB` |
| **Stage2 粗外参** | **learn-to-calibrate** | `learn_to_calib_wrapper`：imu.csv + config.yaml，子进程调用 `calib_rl.py`，解析 YAML 得到 IMU-LiDAR 外参 | `third_party.learn_to_calibrate` 或 `UNICALIB_LEARN_TO_CALIB` |
| **Stage2 粗外参** | **MIAS-LCEC** | `mias_lcec_wrapper`：同步帧 symlink + config，子进程调用 `mias_lcec --config`，解析 result.yaml 得到 LiDAR-Camera 粗外参 | `third_party.mias_lcec` 或 `UNICALIB_MIAS_LCEC` |
| **Stage2 粗外参** | **特征匹配** | `feature_matching`：OpenCV SIFT + findEssentialMat + recoverPose，Camera-Camera 外参 (R,t 无尺度) | 需 `UNICALIB_USE_OPENCV` |
| **Stage3 精外参** | **MIAS-LCEC** | `run_mias_lcec_fine`：同上，mode=fine，带 initial_extrinsic | 同上 |
| **Stage4 验证** | **重投影** | `ReprojectionValidator::validate`：读 .bin 点云，针孔投影，统计图像内内点比例，写 report | 无额外配置 |

## 二、基础设施

- **ProcessRunner**（`process_runner.hpp/cpp`）：跨进程执行命令，支持参数、环境变量、超时、捕获 stdout/stderr；当前实现为 Linux/macOS（fork/exec + pipe）。
- **ExternalToolsConfig**（`external_tools_config.hpp/cpp`）：六方根目录配置，从 YAML `third_party` 或环境变量 `UNICALIB_*` 读取并合并。

## 三、配置示例

在 `config/sensors.yaml` 中增加（或使用环境变量）：

```yaml
third_party:
  dm_calib: "/path/to/DM-Calib"
  learn_to_calibrate: "/path/to/learn-to-calibrate"
  # mias_lcec: "/path/to/MIAS-LCEC"
  # ikalibr: "/path/to/iKalibr"
  # click_calib: "/path/to/click_calib"
  # transformer_imu: "/path/to/Transformer-IMU-Calibrator"
```

或：

```bash
export UNICALIB_DM_CALIB=/path/to/DM-Calib
export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate
```

## 四、数据与依赖要求

- **DM-Calib**：数据目录下需有针孔相机图像（topic 对应子目录中的 .jpg/.png）；DM-Calib 需已安装依赖并具备 `model` 目录。
- **learn-to-calibrate**：需 IMU CSV（`imu.csv` 或 `<topic>.csv`）及 LiDAR 点云目录（.bin/.pcd/.ply）；Python 环境需能运行 `rl_solver/calib_rl.py`。

## 五、未实现（后续可扩展）

- **Stage1**：Transformer-IMU-Calibrator 备选 IMU 内参；针孔 OpenCV 精化。
- **Stage3**：iKalibr 联合优化（ROS2，配置复杂）；click_calib BA（需关键点文件与 optimize.py）。
- **Stage4**：着色/边缘验证、click_calib 人工验证 GUI。

参见 `INTEGRATION_ANALYSIS.md` 中的“深度融合可选路径”与阶段性目标。
