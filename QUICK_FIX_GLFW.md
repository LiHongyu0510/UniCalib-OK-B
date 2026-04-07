# 🚀 快速执行 - GLFW 依赖修复

## ✅ 问题状态

### 已解决 ✅
- **GLM 问题**：已使用本地 `docker/deps/glm` 解决

### 新问题 ⚠️
- **GLFW 错误**：`fatal error: GLFW/glfw3.h: No such file or directory`

---

## 🎯 立即执行修复（2 步骤）

### 步骤 1：下载 GLFW 到本地

```bash
cd /home/wqs/Documents/github/UniCalib/docker/deps
bash download_glfw.sh
```

**预期输出**：
```
[SUCCESS] GLFW 下载完成
目录: /home/wqs/Documents/github/UniCalib/docker/deps/glfw
```

### 步骤 2：重新执行编译

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

### 预期成功输出（MIAS-LCEC 部分）

```
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 检查 GLFW 是否已安装...
[INFO] GLFW 未安装，开始编译安装...
[INFO] 使用本地 GLFW 源码: /root/calib_ws/docker/deps/glfw
[SUCCESS] GLFW 编译安装完成
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[INFO] 编译 C++ 扩展...
-- Found GLM: /root/calib_ws/docker/deps/glm
-- Found OpenGL: /usr/lib/x86_64-linux-gnu/libOpenGL.so
-- Found GLFW: /usr/local/lib/libglfw.so  ← GLFW 找到了！
[PASS] MIAS-LCEC 编译完成
```

---

## 📁 核心修改

### 1. 新增 GLFW 下载脚本

**文件**：`docker/download_glfw.sh`
- 从 GitHub 下载 GLFW 3.3.8
- 支持 wget/curl/git 多种下载方式

### 2. 修改编译脚本支持 GLFW

**文件**：`build_external_tools.sh` - `build_mias_lcec()` 函数

**新增逻辑**：
```bash
# 检查 GLFW 是否已安装
if ! pkg-config --exists glfw3; then
    # 从本地源码编译安装 GLFW
    cd /tmp/glfw_build
    cmake ${CALIB_ROOT}/docker/deps/glfw
    make -j$(nproc)
    make install
fi
```

---

## 📊 依赖目录

```
docker/deps/
├── glm/    ← GLM 0.9.9.8 (Header-only，已下载)
├── glfw/    ← GLFW 3.3.8 (需编译，新下载)
├── cmake/   ← CMake 3.24
├── gtsam/    ← GTSAM 4.2
└── ...
```

---

## 🐛 故障排查

### GLFW 下载失败

```bash
# 手动下载
cd docker/deps
wget https://github.com/glfw/glfw/releases/download/3.3.8/glfw-3.3.8.zip
unzip glfw-3.3.8.zip
mv glfw-3.3.8 glfw
rm -f glfw-3.3.8.zip
```

### GLFW 编译失败

```bash
# 检查依赖（在容器内）
docker run --rm calib_env:humble \
    apt-cache depends libglfw3-dev

# 应显示：
# - libx11-dev
# - libxrandr-dev
# - libxinerama-dev
# ...
```

---

## 📚 详细文档

- `FIX_GLFW_ISSUE.md` - GLFW 完整修复文档
- `GLM_FIX_SUMMARY.md` - GLM 修复总结
- `docker/download_glfw.sh` - GLFW 下载脚本

---

**现在执行上述 2 个步骤即可修复 GLFW 问题！** 🎉
