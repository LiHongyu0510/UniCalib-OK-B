# calib_unified 一键脚本使用说明

## 概述

`calib_unified_run.sh` 用于 **calib_unified** 模块的编译、自动化验证与标定运行。全工程仅使用**一份配置文件**：`calib_unified/config/unicalib_example.yaml`，支持六种标定任务与数据路径参数。

## 前置条件

- Docker 已安装并运行，镜像 `calib_env:humble` 已构建（见 `docker/docker_build.sh`）
- 可选：GPU（nvidia-docker）、X11（用于可视化）

## 常用命令

| 模式 | 命令 |
|------|------|
| 编译 + 验证 | `./calib_unified_run.sh` |
| 仅编译 | `./calib_unified_run.sh --build-only` |
| 仅验证 | `./calib_unified_run.sh --test-only` |
| 运行标定 | `./calib_unified_run.sh --run --task <任务>` |
| 任务详细说明 | `./calib_unified_run.sh --task-help` |
| 帮助 | `./calib_unified_run.sh --help` |
| 进入容器 | `./calib_unified_run.sh --shell` |

## 标定任务 (--task)

所有任务均使用同一配置 `calib_unified/config/unicalib_example.yaml`：

| 任务 | 说明 |
|------|------|
| `imu-intrin` | IMU 内参标定 |
| `cam-intrin` | 相机内参标定 |
| `imu-lidar` | IMU 与雷达外参标定 |
| `lidar-cam` | 雷达与相机外参标定 |
| `cam-cam` | 相机与相机外参标定 |
| `joint` / `all` | 联合标定 |

## 数据路径参数

| 参数 | 说明 |
|------|------|
| `--config <file>` | 配置文件，默认 `calib_unified/config/unicalib_example.yaml` |
| `--data-dir <dir>` | 宿主机数据根目录，挂载为容器内 `CALIB_DATA_DIR` |
| `--dataset <名>` | 数据集子目录，实际数据目录 = `<data-dir>/<名>` |

示例：使用 `nya_02_ros2` 数据集进行 LiDAR-Camera 标定：

```bash
./calib_unified_run.sh --run --task lidar-cam --config calib_unified/config/unicalib_example.yaml --dataset nya_02_ros2
./calib_unified_run.sh --run --task lidar-cam --data-dir /path/to/data --dataset nya_02_ros2 --coarse --manual
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CALIB_DATA_DIR` | `$PROJECT_ROOT/data` | 宿主机数据目录 |
| `CALIB_RESULTS_DIR` | `$PROJECT_ROOT/results` | 宿主机结果目录 |
| `CALIB_LOGS_DIR` | `$PROJECT_ROOT/log` | 宿主机日志目录 |
| `DOCKER_GPU_FLAGS` | `--gpus all` | Docker GPU 参数 |

## 更多说明

- 配置文件字段与注释：见 `calib_unified/config/unicalib_example.yaml`
- 模块架构与编译：见 [calib_unified/README.md](calib_unified/README.md)
