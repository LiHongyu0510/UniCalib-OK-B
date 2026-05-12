# 多传感器标定工程 (calibration)

本项目整体框架使用 **unicalib_C_plus_plus**（C++）进行**编译和运行**，**不使用** UniCalib（Python）作为主运行框架。

## 编译与运行

### 快速开始（推荐）

```bash
# 1. 自动检测并配置所有工具路径
./auto_config_env.sh

# 2. 验证配置（可选）
./verify_config.sh

# 3. 一键编译和运行
./build_and_run.sh
```

### 详细配置与运行

使用项目提供的一键脚本进行自动化编译和运行：

```bash
# 1. 验证环境
./verify_environment.sh

# 2. 一键编译和运行（包含Docker镜像构建、C++编译、标定流程）
./build_and_run.sh

# 可选：自定义数据目录
CALIB_DATA_DIR=/path/to/data ./build_and_run.sh
```

**更多选项**：
- `./build_and_run.sh --build-only` - 仅构建镜像、外部工具和C++代码，不运行标定
- `./build_and_run.sh --build-external-only` - 仅编译外部依赖工具（DM-Calib, iKalibr等）
- `./build_and_run.sh --run-only` - 仅运行标定（需已有构建）
- `./build_and_run.sh --shell` - 进入容器交互式Shell

**详细文档**：
- [calib_unified 一键脚本使用说明](CALIB_UNIFIED_USAGE.md) - calib_unified_run.sh、唯一配置与数据路径
- [外部工具编译指南](BUILD_EXTERNAL_TOOLS.md) - 编译DM-Calib, iKalibr等外部工具
- [快速开始](QUICK_START.md) - 三步快速配置和运行
- [LiDAR-Camera和Camera-Camera外参指南](LIDAR_CAMERA_AND_CAMERA_CAMERA.md) - 外参标定详细指南
- [外参标定完善总结](EXTRINSIC_CALIBRATION_SUMMARY.md) - 外参标定完善说明
- [快速参考](QUICK_REFERENCE.md) - 常用命令快速查询
- [详细指南](BUILD_AND_RUN_GUIDE.md) - 完整使用说明和故障排查
- [改进说明](IMPROVEMENTS.md) - 最新改进和优化

### 配置工具

```bash
# 验证配置状态
./verify_config.sh

# 自动修复（检测路径并更新配置）
./verify_config.sh --fix-auto

# 自动检测并配置环境变量
source ./auto_config_env.sh
```

### 手动编译和运行

- **主入口**：`unicalib_C_plus_plus/`
- **编译**：见 [unicalib_C_plus_plus/README.md](unicalib_C_plus_plus/README.md) 与 [unicalib_C_plus_plus/BUILD_AND_RUN.md](unicalib_C_plus_plus/BUILD_AND_RUN.md)
- **简要步骤**：
  ```bash
  cd unicalib_C_plus_plus
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  RUN_PIPELINE=1 ./unicalib_example config/sensors.yaml /path/to/data
  ```

## 目录说明

| 目录 | 说明 |
|------|------|
| **unicalib_C_plus_plus/** | **主框架**：C++ 实现，本项目的编译与运行均在此完成 |
| **calib_unified/** | 多传感器统一标定 C++ 模块（IMU/相机/LiDAR 内参与外参、联合标定） |
| **calib_unified_run.sh** | calib_unified 一键脚本：编译 + 验证 + 标定运行，全工程唯一配置见 [calib_unified/README.md](calib_unified/README.md) |
| **UniCalib/** | 参考实现：Python 版标定逻辑与设计参考，不参与本项目的主流程编译与运行 |
| **docker/** | Docker 构建与依赖（用于一键脚本） |
| **build_and_run.sh** | 一键编译和运行脚本（推荐使用） |
| **verify_models.sh** | 模型文件验证脚本 |
| **verify_config.sh** | 配置验证工具（新增） |
| **auto_config_env.sh** | 自动配置环境变量（新增） |
| **QUICK_START.md** | 快速开始指南 |
| **QUICK_REFERENCE.md** | 快速参考文档 |
| **BUILD_AND_RUN_GUIDE.md** | 详细使用指南 |
| **IMPROVEMENTS.md** | 改进说明文档 |
| **Readme.mk** | 多传感器标定架构与设计文档（含方案对比、模块说明等） |

## 配置说明

### 第三方工具配置

UniCalib集成了多个第三方AI标定工具，需要配置路径才能使用：

| 工具 | 作用 | 必需 | 环境变量 | 配置键 |
|------|------|------|-----------|--------|
| **DM-Calib** | 针孔相机内参标定 | ✅ | `UNICALIB_DM_CALIB` | `third_party.dm_calib` |
| **learn-to-calibrate** | IMU-LiDAR粗外参标定 | ✅ | `UNICALIB_LEARN_TO_CALIB` | `third_party.learn_to_calibrate` |
| **MIAS-LCEC** | LiDAR-Camera外参标定 | ⚠️ | `UNICALIB_MIAS_LCEC` | `third_party.mias_lcec` |
| **iKalibr** | 多传感器联合优化 | ⚠️ | `UNICALIB_IKALIBR` | `third_party.ikalibr` |
| **click_calib** | Camera-Camera束调整 | ⚠️ | `UNICALIB_CLICK_CALIB` | `third_party.click_calib` |

### 配置方式

#### 方式1：自动配置（推荐）
```bash
./auto_config_env.sh
```

#### 方式2：环境变量
```bash
export UNICALIB_DM_CALIB=/path/to/DM-Calib
export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate
export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC
```

#### 方式3：配置文件
编辑 `unicalib_C_plus_plus/config/sensors.yaml`：
```yaml
third_party:
  dm_calib: "/path/to/DM-Calib"
  learn_to_calibrate: "/path/to/learn-to-calibrate"
  mias_lcec: "/path/to/MIAS-LCEC"
```

## 依赖与配置

- 编译依赖：CMake ≥3.14、Eigen3、可选 yaml-cpp、可选 OpenCV（鱼眼内参）
- 配置文件示例：`unicalib_C_plus_plus/config/sensors.yaml`
- 数据目录格式：见 `unicalib_C_plus_plus/README.md` 中「数据目录」一节
