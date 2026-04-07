# 🚀 iKalibr ROS2 Humble 迁移 - 快速指南

## 📋 问题总结

**问题**：iKalibr 使用 ROS1 (catkin) 构建系统，但 Docker 镜像是 ROS2 Humble (ament/colcon)

**错误**：
```
CMake Error at CMakeLists.txt:63 (find_package):
  By not providing "Findcatkin.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "catkin", but
  CMake did not find one.
```

**解决方案**：
1. ✅ 检测到 iKalibr 使用 catkin
2. ✅ 提示用户选择方案
3. ✅ 推荐跳过 iKalibr（其他 5 个工具已足够）

---

## 🎯 立即执行（3 个方案）

### 方案 1：跳过 iKalibr（推荐，快速）

**适合场景**：只需要其他 5 个标定工具

```bash
cd /home/wqs/Documents/github/UniCalib

# 应用补丁（添加 SKIP_IKALIBR 选项）
bash build_ikalibr_patch.sh

# 执行编译（默认会询问是否跳过）
bash build_and_run.sh
```

**交互提示**：
```
==========================================
应用 iKalibr 补丁
==========================================

✅ 补丁已应用

使用方法：
  bash build_and_run.sh  # 默认会询问是否跳过
  SKIP_IKALIBR=1 bash build_and_run.sh  # 直接跳过

恢复原文件：
  mv build_external_tools.sh.backup build_external_tools.sh
```

**预期输出**：
```
==========================================
iKalibr 需要从 ROS1 迁移到 ROS2 Humble
==========================================================================
这是一个大工程，需要修改：
  1. CMakeLists.txt (~830 行)
  2. package.xml
  3. Python 代码 (~50+ 文件)
  4. launch 文件 (~10+ 文件)

详细迁移步骤请参考:
  /home/wqs/Documents/github/UniCalib/MIGRATE_IKALIBR_ROS2.md
==========================================================================
选择方案:
  1) 跳过 iKalibr 编译（其他 5 个工具已足够）[推荐]
  2) 尝试编译 iKalibr（会失败，预期错误）
  3) 先完成 ROS2 迁移，再使用 --only-ikalibr 参数

是否跳过 iKalibr? [Y/n]: 
```

**输入 `Y` 或直接回车** → 跳过 iKalibr

### 方案 2：先跳过，后续迁移

**适合场景**：现在想快速编译其他工具，后续有时间再迁移 iKalibr

```bash
cd /home/wqs/Documents/github/UniCalib

# 方法 A：直接跳过
SKIP_IKALIBR=1 bash build_and_run.sh

# 方法 B：跳过特定工具
bash build_external_tools.sh --skip ikalibr
```

### 方案 3：完整 ROS2 迁移（长期方案）

**适合场景**：需要 iKalibr 功能，愿意花时间迁移

**详细步骤**：参考 `MIGRATE_IKALIBR_ROS2.md`

**预计工作量**：7-11 天

---

## 📊 当前状态

### 已修复的工具

| 工具 | 状态 | 说明 |
|-----|------|------|
| DM-Calib | ✅ 已修复 | GLM 本地依赖 |
| learn-to-calibrate | ✅ 无需修复 | 纯 Python |
| MIAS-LCEC | ✅ 已修复 | GLM + GLFW 本地依赖 |
| iKalibr | ⚠️ 需跳过 | ROS1 → ROS2 不兼容 |
| click_calib | ✅ 无需修复 | 纯 Python |
| Transformer-IMU-Calibrator | ✅ 无需修复 | 纯 Python |

### 依赖目录状态

```
docker/deps/
├── cmake/              ✅ CMake 3.24
├── glm/               ✅ GLM 0.9.9.8 (Header-only)
├── glfw/              ✅ GLFW 3.3.8
├── gtsam/             ✅ GTSAM 4.2
├── Sophus/            ✅ Sophus
├── magic_enum/         ✅ magic_enum
├── ceres-solver/       ✅ Ceres
├── opencv/             ✅ OpenCV 4.8.0
└── ...
```

---

## 🔍 编译流程（跳过 iKalibr）

### 预期输出

```
========================================
UniCalib 外部工具编译
========================================

>>> 检测外部工具...
[PASS] DM-Calib: /root/calib_ws/DM-Calib
[PASS] learn-to-calibrate: /root/calib_ws/learn-to-calibrate
[PASS] MIAS-LCEC: /root/calib_ws/MIAS-LCEC
[WARN] iKalibr: /root/calib_ws/iKalibr  (使用 ROS1 catkin)
[SKIP] 跳过 iKalibr 编译
[PASS] click_calib: /root/calib_ws/click_calib
[PASS] Transformer-IMU-Calibrator: /root/calib_ws/Transformer-IMU-Calibrator

[INFO] 检测到 5 个工具

>>> 开始编译...

>>> 编译 DM-Calib (Python + PyTorch)
[PASS] DM-Calib 准备完成

>>> 编译 learn-to-calibrate (Python + PyTorch)
[PASS] learn-to-calibrate 准备完成

>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 检查 GLFW 是否已安装...
[INFO] GLFW 未安装，开始编译安装...
[SUCCESS] GLFW 编译安装完成
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[PASS] MIAS-LCEC 编译完成

>>> 编译 click_calib (Python)
[PASS] click_calib 准备完成

>>> 编译 Transformer-IMU-Calibrator (Python + PyTorch)
[PASS] Transformer-IMU-Calibrator 准备完成

========================================
编译完成
========================================

[INFO] 成功编译 5 个工具
```

---

## 📚 详细文档

| 文档 | 用途 |
|-----|------|
| `MIGRATE_IKALIBR_ROS2.md` | ROS1 → ROS2 完整迁移指南 |
| `build_ikalibr_patch.sh` | 快速补丁脚本 |
| `build_external_tools.sh` | 已修改的编译脚本 |

---

## 🐛 故障排查

### 问题 1：补丁脚本执行失败

**现象**：`sed: unknown option`

**解决方案**：
```bash
# 检查 sed 版本
sed --version

# 如果是旧版本，手动修改
# 参考 build_ikalibr_patch.sh 中的修改
```

### 问题 2：仍然询问跳过

**现象**：即使设置 `SKIP_IKALIBR=1`，仍然询问

**解决方案**：
```bash
# 检查环境变量
echo $SKIP_IKALIBR

# 确保导出
export SKIP_IKALIBR=1
bash build_and_run.sh
```

### 问题 3：其他工具编译失败

**现象**：DM-Calib 或 MIAS-LCEC 编译失败

**排查**：
```bash
# 检查 GLM 文件
ls -la docker/deps/glm/glm/glm.hpp

# 检查 GLFW 文件
ls -la docker/deps/glfw/include/GLFW/glfw3.h

# 清理旧的编译产物
rm -rf MIAS-LCEC/bin/iridescence/build

# 重新编译
bash build_and_run.sh
```

---

## 🎯 推荐行动路径

### 短期（今天）

1. ✅ 应用补丁：`bash build_ikalibr_patch.sh`
2. ✅ 跳过 iKalibr：输入 `Y`
3. ✅ 验证其他 5 个工具编译成功

### 中期（本周）

1. ⏳ 评估是否需要 iKalibr 功能
2. ⏳ 如果需要，阅读 `MIGRATE_IKALIBR_ROS2.md`
3. ⏳ 开始 ROS2 迁移（7-11 天）

### 长期（本月）

1. ⏳ 完成 iKalibr ROS2 迁移
2. ⏳ 测试所有功能
3. ⏳ 更新文档

---

## 📊 编译工具对比

| 工具 | 编译时间 | 复杂度 | 优先级 |
|-----|---------|--------|--------|
| DM-Calib | 1 分钟 | 低 | 高 |
| learn-to-calibrate | 30 秒 | 低 | 高 |
| MIAS-LCEC | 5-10 分钟 | 中 | 高 |
| iKalibr | 跳过 | 高 | 低 |
| click_calib | 2 分钟 | 低 | 中 |
| Transformer-IMU-Calibrator | 1 分钟 | 低 | 中 |

**总计**：约 10-15 分钟（跳过 iKalibr）

---

## 🎉 立即执行

**一键执行**：
```bash
cd /home/wqs/Documents/github/UniCalib

# 1. 应用补丁
bash build_ikalibr_patch.sh

# 2. 执行编译（输入 Y 跳过 iKalibr）
bash build_and_run.sh
```

**直接跳过**：
```bash
cd /home/wqs/Documents/github/UniCalib
SKIP_IKALIBR=1 bash build_and_run.sh
```

**仅编译外部工具**：
```bash
cd /home/wqs/Documents/github/UniCalib
SKIP_IKALIBR=1 bash build_and_run.sh --build-external-only
```

---

## 📞 总结

### 核心要点

1. ✅ **已修复**：GLM、GLFW、编译路径问题
2. ✅ **新问题**：iKalibr ROS1 vs ROS2 不兼容
3. ✅ **解决方案**：提供跳过选项 + 迁移指南
4. ✅ **灵活方案**：3 种执行路径（跳过/尝试/迁移）

### 立即执行

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_ikalibr_patch.sh
bash build_and_run.sh
# 输入 Y 跳过 iKalibr
```

**预期结果**：其他 5 个工具编译成功！🎉
