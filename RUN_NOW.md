# 🚀 立即执行编译

## ✅ 准备状态

### 已完成
- ✅ GLM 已下载：`docker/deps/glm/`
- ✅ GLFW 已下载：`docker/deps/glfw/`
- ✅ 编译路径已修复：`build_external_tools.sh`
- ✅ 动态 GLFW 编译已添加：`build_mias_lcec()`

### 验证

```bash
# 验证 GLM
ls -la docker/deps/glm/glm/glm.hpp

# 验证 GLFW
ls -la docker/deps/glfw/include/GLFW/glfw3.h
```

---

## 🎯 立即执行

### 方法 1：完整编译（推荐）

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

### 方法 2：仅编译外部工具

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh --build-external-only
```

### 方法 3：仅编译 MIAS-LCEC

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_external_tools.sh --tools mias_lcec
```

---

## 📊 预期输出

### MIAS-LCEC 编译成功

```
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 检查 GLFW 是否已安装...
[INFO] GLFW 未安装，开始编译安装...
[INFO] 使用本地 GLFW 源码: /root/calib_ws/docker/deps/glfw
-- Configuring done
-- Generating done
[100%] Built target glfw
[SUCCESS] GLFW 编译安装完成
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[INFO] 编译 C++ 扩展...
-- Found GLM: /root/calib_ws/docker/deps/glm
-- Found OpenGL: /usr/lib/x86_64-linux-gnu/libOpenGL.so
-- Found GLFW: /usr/local/lib/libglfw.so  ← 找到了！
[PASS] MIAS-LCEC 编译完成
```

### 所有工具编译成功

```
>>> 编译 DM-Calib (Python + PyTorch)
[PASS] DM-Calib 准备完成

>>> 编译 learn-to-calibrate (Python + PyTorch)
[PASS] learn-to-calibrate 准备完成

>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[INFO] 检查 GLFW 是否已安装...
[SUCCESS] GLFW 编译安装完成
[PASS] MIAS-LCEC 编译完成  ← 成功！

>>> 编译 iKalibr (ROS2 + C++)
[PASS] iKalibr 编译完成

>>> 编译 click_calib (Python)
[PASS] click_calib 准备完成

>>> 编译 Transformer-IMU-Calibrator (Python + PyTorch)
[PASS] Transformer-IMU-Calibrator 准备完成

[INFO] 成功编译 6 个工具
```

---

## 🐛 故障排查

### GLFW 编译失败

**现象**：GLFW 编译报错

**解决方案**：
```bash
# 检查依赖（在容器内）
docker run --rm calib_env:humble apt-cache depends libglfw3-dev

# 应显示需要的依赖：
# libx11-dev, libxrandr-dev, libxinerama-dev, 等
```

### CMake 仍然找不到 GLFW

**现象**：`Could NOT find GLFW`

**排查**：
```bash
# 检查 pkg-config
docker run --rm calib_env:humble pkg-config --modversion glfw3

# 检查库文件
docker run --rm calib_env:humble find /usr -name "libglfw.so*"
```

---

## 📚 相关文档

| 文档 | 用途 |
|-----|------|
| `FIX_GLFW_ISSUE.md` | GLFW 完整修复文档 |
| `GLM_FIX_SUMMARY.md` | GLM 修复总结 |
| `docker/download_glfw.sh` | GLFW 下载脚本 |
| `docker/download_glm.sh` | GLM 下载脚本 |

---

## 🎉 立即执行

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

**预期结果**：所有 6 个外部工具编译成功！🎉
