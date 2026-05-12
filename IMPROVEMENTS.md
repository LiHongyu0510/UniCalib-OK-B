# UniCalib 改进说明

## 改进概览

本次改进包括三个方面：

1. **改进错误提示** - 在各个wrapper中添加详细的配置建议
2. **增强验证脚本** - 添加配置路径检测和配置建议
3. **自动配置脚本** - 创建 `auto_config_env.sh` 自动设置环境变量

---

## 改进1: 错误提示增强

### 修改的文件

- `unicalib_C_plus_plus/src/dm_calib_wrapper.cpp`
- `unicalib_C_plus_plus/src/learn_to_calib_wrapper.cpp`
- `unicalib_C_plus_plus/src/mias_lcec_wrapper.cpp`
- `unicalib_C_plus_plus/src/ikalibr_wrapper.cpp`
- `unicalib_C_plus_plus/src/click_calib_wrapper.cpp`

### 改进内容

每个wrapper在检测到工具路径未配置时，现在会输出：

1. 配置方式说明（YAML配置文件 或 环境变量）
2. 配置示例
3. 默认路径建议

#### 示例输出

**DM-Calib未配置时**:
```
[WARN] DM-Calib not configured. Set 'third_party.dm_calib' in config.yaml or set UNICALIB_DM_CALIB environment variable.
[WARN] Example config:
[WARN]   third_party:
[WARN]     dm_calib: "/path/to/DM-Calib"
[WARN] Example environment variable:
[WARN]   export UNICALIB_DM_CALIB=/path/to/DM-Calib
[WARN] Default location: /path/to/calibration/DM-Calib
```

**learn-to-calibrate未配置时**:
```
[WARN] learn-to-calibrate not configured. Set 'third_party.learn_to_calibrate' in config.yaml or set UNICALIB_LEARN_TO_CALIB environment variable.
[WARN] Example config:
[WARN]   third_party:
[WARN]     learn_to_calibrate: "/path/to/learn-to-calibrate"
[WARN] Example environment variable:
[WARN]   export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate
[WARN] Default location: /path/to/calibration/learn-to-calibrate
```

**MIAS-LCEC未配置时**:
```
[WARN] MIAS-LCEC not configured for coarse extrinsic calibration.
[WARN] Set 'third_party.mias_lcec' in config.yaml or set UNICALIB_MIAS_LCEC environment variable.
[WARN] Example config:
[WARN]   third_party:
[WARN]     mias_lcec: "/path/to/MIAS-LCEC"
[WARN] Example environment variable:
[WARN]   export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC
[WARN] Default location: /path/to/calibration/MIAS-LCEC
[WARN] Note: Run './auto_config_env.sh' to auto-detect and set environment variables.
```

---

## 改进2: 验证脚本增强

### 新增文件

- `verify_config.sh` - 配置验证工具

### 功能特性

#### 1. 配置文件检查
- 检查 `unicalib_C_plus_plus/config/sensors.yaml` 是否存在
- 自动创建示例配置（如果不存在）

#### 2. 环境变量检查
- 检查所有 `UNICALIB_*` 环境变量
- 验证路径是否存在
- 区分必需工具和可选工具

#### 3. 配置文件路径检查
- 解析YAML配置文件中的 `third_party` 部分
- 验证每个工具路径是否存在

#### 4. 自动检测工具
- 在多个标准位置自动搜索工具
- 支持的搜索路径：
  - 项目根目录（`./DM-Calib`）
  - src目录（`./src/DM-Calib`）
  - 系统安装目录（`/opt/DM-Calib`）
  - 用户主目录（`~/DM-Calib`）

#### 5. 配置建议生成
- 根据检测结果生成配置示例
- 提供环境变量和配置文件两种配置方式
- 标注必需工具和可选工具

### 使用方法

```bash
# 基本验证
./verify_config.sh

# 自动修复（检测路径并更新配置）
./verify_config.sh --fix-auto
```

### 输出示例

```
==========================================
  UniCalib Configuration Verification Tool
==========================================

>>> Check Configuration File...
[PASS] Configuration file exists: /path/to/unicalib_C_plus_plus/config/sensors.yaml

>>> Check Environment Variables...
[PASS] UNICALIB_DM_CALIB: /path/to/DM-Calib
[PASS] UNICALIB_LEARN_TO_CALIB: /path/to/learn-to-calibrate
[WARN] UNICALIB_MIAS_LCEC: not set (optional)

>>> Check Configuration File Paths...
[PASS] third_party.dm_calib: /path/to/DM-Calib
[PASS] third_party.learn_to_calibrate: /path/to/learn-to-calibrate

>>> Auto-detect Tools...
[PASS] DM-Calib: /path/to/DM-Calib
[PASS] learn-to-calibrate: /path/to/learn-to-calibrate
[PASS] MIAS-LCEC: /path/to/MIAS-LCEC

>>> Configuration Suggestions...
    └─ ℹ️  MIAS-LCEC not configured (optional)
        MIAS-LCEC is used for LiDAR-Camera extrinsic calibration.
        Suggestions:
          Method 1 (environment variable):
            export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC
          Method 2 (config file):
            In /path/to/config/sensors.yaml add:
              third_party:
                mias_lcec: "/path/to/MIAS-LCEC"

    └─ 🚀 Quick Configuration (Recommended):
        Run auto-configuration script:
          ./auto_config_env.sh
        
        This will:
          • Auto-detect all tool paths
          • Generate environment variable script: env_setup.sh
          • Update configuration file: config/sensors.yaml
          • Export environment variables to current shell

==========================================
  Verification Summary
==========================================

  Passed: 8
  Warnings: 2
  Failed: 0

==========================================
  Configuration Example
==========================================

third_party:
  dm_calib: "/path/to/DM-Calib"  # auto-detected
  learn_to_calibrate: "/path/to/learn-to-calibrate"  # auto-detected
  mias_lcec: "/path/to/MIAS-LCEC"  # auto-detected
  # ikalibr: "/path/to/iKalibr"  # needs to be set
  # click_calib: "/path/to/click_calib"  # needs to be set

==========================================
  Next Steps
==========================================

✓ All configuration checks passed!

You can now run the calibration pipeline:
  ./build_and_run.sh

Or run manually:
  cd unicalib_C_plus_plus/build
  RUN_PIPELINE=1 ./unicalib_example ../config/sensors.yaml /path/to/data
```

---

## 改进3: 自动配置脚本

### 新增文件

- `auto_config_env.sh` - 自动检测并配置环境变量

### 功能特性

#### 1. 自动检测工具
在多个标准位置搜索工具目录，包括：
- 项目根目录
- src子目录
- 系统安装目录（`/opt/`）
- 用户主目录（`~/`）

#### 2. 验证工具完整性
检查关键文件是否存在：
- **DM-Calib**: `DMCalib/tools/infer.py`
- **learn-to-calibrate**: `rl_solver/calib_rl.py`
- **MIAS-LCEC**: `model/pretrained_overlap_transformer.pth.tar`
- **Transformer-IMU**: `checkpoint/TIC_13.pth`
- **iKalibr**: `package.xml`
- **click_calib**: `source/optimize.py`

#### 3. 环境变量导出
生成环境变量配置：
```bash
export UNICALIB_DM_CALIB=/path/to/DM-Calib
export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate
export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC
export UNICALIB_IKALIBR=/path/to/iKalibr
export UNICALIB_CLICK_CALIB=/path/to/click_calib
export UNICALIB_TRANSFORMER_IMU=/path/to/Transformer-IMU-Calibrator
```

#### 4. 配置文件更新
自动更新 `unicalib_C_plus_plus/config/sensors.yaml` 中的 `third_party` 部分

#### 5. 脚本生成
生成 `env_setup.sh` 文件，可被source以持久化环境变量

### 使用方法

```bash
# 方式1: 直接运行并临时导出环境变量
source ./auto_config_env.sh

# 方式2: 生成配置脚本后手动source
./auto_config_env.sh > env_setup.sh
source env_setup.sh

# 方式3: 永久添加到bashrc
./auto_config_env.sh >> ~/.bashrc
source ~/.bashrc
```

### 输出示例

```
==========================================
  UniCalib Auto Configuration
==========================================

>>> Detect Third-party Tools...
[PASS] DM-Calib: /home/user/calibration/DM-Calib
[PASS] learn-to-calibrate: /home/user/calibration/learn-to-calibrate
[PASS] MIAS-LCEC: /home/user/calibration/MIAS-LCEC
[PASS] Transformer-IMU-Calibrator: /home/user/calibration/Transformer-IMU-Calibrator
[WARN] iKalibr: not found
[WARN] click_calib: not found

>>> Detection Result Summary
[PASS] dm_calib: /home/user/calibration/DM-Calib
[PASS] learn_to_calibrate: /home/user/calibration/learn-to-calibrate
[PASS] mias_lcec: /home/user/calibration/MIAS-LCEC
[PASS] transformer_imu: /home/user/calibration/Transformer-IMU-Calibrator
[WARN] ikalibr: not found
[WARN] click_calib: not found

>>> Generate Environment Variables...

# UniCalib Environment Variables Configuration
# Generated by auto_config_env.sh
# Usage: source auto_config_env.sh or source env_setup.sh
# Generated time: Sat Feb 28 10:30:00 UTC 2026

export UNICALIB_DM_CALIB="/home/user/calibration/DM-Calib"
export UNICALIB_LEARN_TO_CALIB="/home/user/calibration/learn-to-calibrate"
export UNICALIB_MIAS_LCEC="/home/user/calibration/MIAS-LCEC"
export UNICALIB_TRANSFORMER_IMU="/home/user/calibration/Transformer-IMU-Calibrator"
# export UNICALIB_IKALIBR=""  # not found, please set manually
# export UNICALIB_CLICK_CALIB=""  # not found, please set manually

>>> Update Configuration File: /home/user/calibration/unicalib_C_plus_plus/config/sensors.yaml
[PASS] Configuration file updated: /home/user/calibration/unicalib_C_plus_plus/config/sensors.yaml
[INFO] Detected paths written to third_party section

[PASS] Environment variable script generated: /home/user/calibration/env_setup.sh
[INFO] Usage: source /home/user/calibration/env_setup.sh

==========================================
  Configuration Complete
==========================================

[INFO] Detected tools: 4
[INFO] Next steps:
  1. Use directly (environment variables active):
     ./build_and_run.sh
  2. Or persist environment variables:
     source /home/user/calibration/env_setup.sh  # temporary
     echo 'source /home/user/calibration/env_setup.sh' >> ~/.bashrc  # permanent
  3. Or use config file (auto-updated):
     Third-party paths written to: /home/user/calibration/unicalib_C_plus_plus/config/sensors.yaml
[INFO] To re-detect, run again: source auto_config_env.sh

[INFO] Exporting environment variables...
[PASS] export UNICALIB_DM_CALIB="/home/user/calibration/DM-Calib"
[PASS] export UNICALIB_LEARN_TO_CALIB="/home/user/calibration/learn-to-calibrate"
[PASS] export UNICALIB_MIAS_LCEC="/home/user/calibration/MIAS-LCEC"
[PASS] export UNICALIB_TRANSFORMER_IMU="/home/user/calibration/Transformer-IMU-Calibrator"
```

---

## 快速开始

### 一键配置和运行

```bash
# Step 1: 自动检测并配置所有工具路径
./auto_config_env.sh

# Step 2: 一键编译和运行
./build_and_run.sh
```

### 验证配置

```bash
# 检查配置状态
./verify_config.sh

# 自动修复（检测路径并更新配置）
./verify_config.sh --fix-auto
```

---

## 工具优先级

### 必需工具（Must Have）

1. **DM-Calib** - 针孔相机内参标定
   - 路径: `DM-Calib/DMCalib/tools/infer.py`
   - 环境变量: `UNICALIB_DM_CALIB`
   - 配置键: `third_party.dm_calib`

2. **learn-to-calibrate** - IMU-LiDAR粗外参标定
   - 路径: `learn-to-calibrate/rl_solver/calib_rl.py`
   - 环境变量: `UNICALIB_LEARN_TO_CALIB`
   - 配置键: `third_party.learn_to_calibrate`

### 可选工具（Nice to Have）

3. **MIAS-LCEC** - LiDAR-Camera外参标定
   - 路径: `MIAS-LCEC/model/pretrained_overlap_transformer.pth.tar`
   - 环境变量: `UNICALIB_MIAS_LCEC`
   - 配置键: `third_party.mias_lcec`
   - 用途: 粗/精两阶段外参标定

4. **iKalibr** - 多传感器联合优化
   - 路径: `iKalibr/package.xml`
   - 环境变量: `UNICALIB_IKALIBR`
   - 配置键: `third_party.ikalibr`
   - 要求: ROS2环境

5. **click_calib** - Camera-Camera束调整
   - 路径: `click_calib/source/optimize.py`
   - 环境变量: `UNICALIB_CLICK_CALIB`
   - 配置键: `third_party.click_calib`

6. **Transformer-IMU-Calibrator** - IMU内参备选
   - 路径: `Transformer-IMU-Calibrator/checkpoint/TIC_13.pth`
   - 环境变量: `UNICALIB_TRANSFORMER_IMU`
   - 配置键: `third_party.transformer_imu`
   - 用途: 运动捕捉场景

---

## 环境变量优先级

系统按以下优先级读取配置：

1. **环境变量** (UNICALIB_*)
2. **配置文件** (third_party.*)
3. **自动检测** (脚本运行时）

推荐使用方式：
- 首次配置: 运行 `./auto_config_env.sh`
- 持久化: `source env_setup.sh` 添加到 `~/.bashrc`
- 团队共享: 使用配置文件 `config/sensors.yaml`

---

## 故障排查

### 问题1: DM-Calib not configured

**症状**:
```
[WARN] DM-Calib not configured. Set 'third_party.dm_calib' in config.yaml...
```

**解决方案**:
```bash
# 运行自动配置
./auto_config_env.sh

# 或手动设置
export UNICALIB_DM_CALIB=/path/to/DM-Calib
```

### 问题2: learn-to-calibrate not configured

**症状**:
```
[WARN] learn-to-calibrate not configured. Set 'third_party.learn_to_calibrate'...
```

**解决方案**:
```bash
# 运行自动配置
./auto_config_env.sh

# 或手动设置
export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate
```

### 问题3: 工具路径检测失败

**症状**: 自动检测找不到工具

**解决方案**:
```bash
# 检查工具是否存在
ls -la DM-Calib/DMCalib/tools/infer.py
ls -la learn-to-calibrate/rl_solver/calib_rl.py

# 手动设置环境变量
export UNICALIB_DM_CALIB=$(pwd)/DM-Calib
export UNICALIB_LEARN_TO_CALIB=$(pwd)/learn-to-calibrate
```

### 问题4: 配置文件无法解析

**症状**: `verify_config.sh` 报告YAML解析错误

**解决方案**:
```bash
# 检查YAML语法
python3 -c "import yaml; yaml.safe_load(open('unicalib_C_plus_plus/config/sensors.yaml'))"

# 重新生成配置文件
./auto_config_env.sh
```

---

## 文件清单

### 修改的文件

- `unicalib_C_plus_plus/src/dm_calib_wrapper.cpp` - 增强错误提示
- `unicalib_C_plus_plus/src/learn_to_calib_wrapper.cpp` - 增强错误提示
- `unicalib_C_plus_plus/src/mias_lcec_wrapper.cpp` - 增强错误提示
- `unicalib_C_plus_plus/src/ikalibr_wrapper.cpp` - 增强错误提示
- `unicalib_C_plus_plus/src/click_calib_wrapper.cpp` - 增强错误提示

### 新增的文件

- `verify_config.sh` - 配置验证工具
- `auto_config_env.sh` - 自动配置工具
- `env_setup.sh` (自动生成) - 环境变量脚本
- `QUICK_START.md` - 快速开始指南
- `IMPROVEMENTS.md` - 本改进说明文档

---

## 总结

通过这三项改进，UniCalib系统的配置和使用体验得到显著提升：

1. **更友好的错误提示** - 用户可以快速理解问题并得到解决建议
2. **自动化的配置验证** - 一键检查所有配置，自动检测工具路径
3. **智能的自动配置** - 无需手动查找路径，脚本自动检测并设置

用户现在可以通过以下三步快速开始：
```bash
./auto_config_env.sh    # 自动配置
./verify_config.sh       # 验证配置
./build_and_run.sh       # 运行标定
```

大幅降低了配置门槛和出错概率！
