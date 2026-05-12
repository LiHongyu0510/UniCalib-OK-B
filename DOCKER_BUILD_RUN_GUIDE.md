# Docker 环境编译运行指南

## 一、概述

本工程使用 Docker 容器进行编译和运行，确保环境一致性和依赖完整性。

### 1.1 容器环境

| 组件 | 版本 |
|------|------|
| Ubuntu | 22.04 |
| ROS2 | Humble |
| CUDA | 11.8 |
| PyTorch | 2.1.0 |
| OpenCV | 4.8.0 (with CUDA) |
| Ceres | 2.1.0 |
| GTSAM | 4.2 |

### 1.2 目录映射

| 宿主机路径 | 容器内路径 | 用途 |
|------------|------------|------|
| `{项目根目录}` | `/root/calib_ws` | 所有项目代码 |
| `{项目根目录}/Transformer-IMU-Calibrator` | `/root/calib_ws/Transformer-IMU-Calibrator` | IMU内参备选 |
| `{项目根目录}/DM-Calib` | `/root/calib_ws/DM-Calib` | 针孔内参标定 |
| `{项目根目录}/learn-to-calibrate` | `/root/calib_ws/learn-to-calibrate` | IMU-LiDAR粗外参 |
| `{项目根目录}/MIAS-LCEC` | `/root/calib_ws/MIAS-LCEC` | LiDAR-Camera外参 |
| `{项目根目录}/iKalibr` | `/root/calib_ws/iKalibr` | 时空联合优化 |
| `{项目根目录}/click_calib` | `/root/calib_ws/click_calib` | Camera-Camera BA |
| `${CALIB_DATA_DIR}` | `/root/calib_ws/data` | 标定数据 |
| `${CALIB_RESULTS_DIR}` | `/root/calib_ws/results` | 标定结果 |

---

## 二、一键编译运行

### 2.1 基础使用

```bash
# 进入项目根目录
cd /path/to/calibration

# 赋予执行权限
chmod +x build_and_run.sh

# 完整流程：构建镜像 → 编译C++ → 运行标定
./build_and_run.sh
```

### 2.2 运行模式

```bash
# 仅构建（构建镜像 + 编译C++，不运行标定）
./build_and_run.sh --build-only

# 仅运行（假设已构建完成，仅运行标定）
./build_and_run.sh --run-only

# 交互式 Shell（用于调试）
./build_and_run.sh --shell
```

### 2.3 自定义数据目录

```bash
# 自定义数据目录
CALIB_DATA_DIR=/path/to/my/data ./build_and_run.sh

# 自定义结果目录
CALIB_RESULTS_DIR=/path/to/my/results ./build_and_run.sh

# 组合使用
CALIB_DATA_DIR=/data \
CALIB_RESULTS_DIR=/output \
./build_and_run.sh
```

---

## 三、容器内操作

### 3.1 进入容器

```bash
./build_and_run.sh --shell
```

### 3.2 容器内常用命令

```bash
# 验证环境
/root/scripts/test_env.sh
# 或
/opt/scripts/verify_env.sh

# 手动编译 C++ 项目
cd /root/calib_ws/unicalib_C_plus_plus
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 运行标定
cd /root/calib_ws/unicalib_C_plus_plus/build
RUN_PIPELINE=1 ./unicalib_example ../config/sensors_docker.yaml /root/calib_ws/data
```

### 3.3 验证模型加载

```bash
# 在容器内运行
cd /root/calib_ws
./setup_models.sh
./verify_models.sh
```

---

## 四、配置文件说明

### 4.1 使用 Docker 专用配置

容器内使用 `sensors_docker.yaml` 配置文件，其中第三方工具路径已配置为容器内绝对路径。

```bash
# 在容器内运行时使用
RUN_PIPELINE=1 ./unicalib_example ../config/sensors_docker.yaml /root/calib_ws/data
```

### 4.2 配置第三方工具

在 `sensors_docker.yaml` 中已预配置：

```yaml
third_party:
  dm_calib: "/root/calib_ws/DM-Calib"
  learn_to_calibrate: "/root/calib_ws/learn-to-calibrate"
  mias_lcec: "/root/calib_ws/MIAS-LCEC"
  ikalibr: "/root/calib_ws/iKalibr"
  click_calib: "/root/calib_ws/click_calib"
  transformer_imu: "/root/calib_ws/Transformer-IMU-Calibrator"
```

---

## 五、模型文件检查

### 5.1 模型文件位置

| 工程 | 模型文件 | 容器内路径 |
|------|----------|------------|
| Transformer-IMU | `TIC_13.pth` | `/root/calib_ws/Transformer-IMU-Calibrator/model/TIC_13.pth` |
| MIAS-LCEC | `pretrained_overlap_transformer.pth.tar` | `/root/calib_ws/MIAS-LCEC/model/pretrained_overlap_transformer.pth.tar` |
| learn-to-calibrate | `*.model` | `/root/calib_ws/learn-to-calibrate/rl_solver/bingham/precomputed/` |
| DM-Calib | `diffusion_pytorch_model.safetensors` | `/root/calib_ws/DM-Calib/model/calib/` |

### 5.2 模型路径修复

首次进入容器后运行：

```bash
cd /root/calib_ws
./setup_models.sh
```

这会：
1. 为 Transformer-IMU 创建 `checkpoint/TIC_13.pth` 软链接
2. 检查所有模型文件是否存在
3. 输出检查结果

---

## 六、完整运行流程

### 6.1 准备数据

数据目录结构：

```
${CALIB_DATA_DIR}/
├── camera/
│   └── cam_front/
│       └── images/
│           ├── 000001.jpg
│           ├── 000002.jpg
│           └── ...
├── imu/
│   └── data.csv          # timestamp,gx,gy,gz,ax,ay,az
└── lidar/
    └── points/
        ├── scan001.bin   # 或 .pcd
        └── ...
```

### 6.2 运行标定

```bash
# 方式一：使用脚本
./build_and_run.sh

# 方式二：手动步骤
./build_and_run.sh --build-only  # 编译
./build_and_run.sh --shell       # 进入容器

# 在容器内
cd /root/calib_ws/unicalib_C_plus_plus/build
RUN_PIPELINE=1 ./unicalib_example ../config/sensors_docker.yaml /root/calib_ws/data
```

### 6.3 查看结果

```bash
# 结果保存在
ls /root/calib_ws/results/

# 输出文件
# - intrinsics.yaml      # 内参结果
# - extrinsics.yaml      # 外参结果
# - validation_report.yaml
# - report.html          # HTML 报告
```

---

## 七、故障排除

### 7.1 Docker 镜像构建失败

```bash
# 检查 Docker 是否运行
docker info

# 手动构建镜像
cd docker
./docker_build.sh --build-only
```

### 7.2 C++ 编译失败

```bash
# 进入容器调试
./build_and_run.sh --shell

# 手动编译查看详细错误
cd /root/calib_ws/unicalib_C_plus_plus
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j1 VERBOSE=1
```

### 7.3 模型加载失败

```bash
# 在容器内检查模型
cd /root/calib_ws

# 运行模型检查
./setup_models.sh
./verify_models.sh
```

### 7.4 GPU 不可用

```bash
# 检查 nvidia-docker
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# 如果失败，检查 NVIDIA 驱动和 nvidia-container-toolkit
```

---

## 八、开发调试

### 8.1 修改代码后重新编译

```bash
# 方式一：在容器内
./build_and_run.sh --shell
cd /root/calib_ws/unicalib_C_plus_plus/build
make -j$(nproc)

# 方式二：使用脚本
./build_and_run.sh --build-only
```

### 8.2 调试模式

```bash
./build_and_run.sh --shell
cd /root/calib_ws/unicalib_C_plus_plus/build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)

# 使用 gdb
gdb --args ./unicalib_example ../config/sensors_docker.yaml /root/calib_ws/data
```

---

## 九、附录

### 9.1 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CALIB_DATA_DIR` | `/tmp/calib_data` | 宿主机数据目录 |
| `CALIB_RESULTS_DIR` | `/tmp/calib_results` | 宿主机结果目录 |
| `AUTO_BUILD_LIVOX` | `0` | 是否自动编译 Livox 驱动 |

### 9.2 Docker 镜像信息

```bash
# 查看镜像
docker images | grep calib_env

# 镜像大小约 10-12 GB（包含所有依赖）

# 查看容器日志
docker logs <container_id>
```

### 9.3 清理

```bash
# 清理 Docker 资源
docker system prune -a

# 清理编译缓存
cd unicalib_C_plus_plus/build
make clean

# 清理结果
rm -rf /tmp/calib_results/*
```
