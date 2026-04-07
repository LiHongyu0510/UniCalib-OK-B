# 🔧 MIAS-LCEC GLFW 依赖修复 - 完整解决方案

## 📋 问题说明

**错误信息**：
```
/root/calib_ws/MIAS-LCEC/bin/iridescence/thirdparty/imgui/backends/imgui_impl_glfw.cpp:77:10: fatal error: GLFW/glfw3.h: No such file or directory
   77 | #include <GLFW/glfw3.h>
      |          ^~~~~~~~~~~~~~
```

**问题链条**：
1. ❌ MIAS-LCEC 的 CMake 需要 GLFW 库（窗口管理）
2. ❌ 当前 Docker 镜像未安装 GLFW
3. ❌ 编译时找不到 `GLFW/glfw3.h` 头文件

---

## ✅ 解决方案：动态编译安装 GLFW

### 核心理念

**不修改 Dockerfile**，而是：
1. 下载 GLFW 源码到 `docker/deps/glfw/`
2. 在编译 MIAS-LCEC 时，动态编译并安装 GLFW
3. 编译完成后 GLFW 已安装到系统中

---

## 📝 已完成的修改

### 1. 创建 GLFW 下载脚本 ✅

**文件**：`docker/download_glfw.sh`

功能：
- 从 GitHub Releases 下载 GLFW 3.3.8（推荐）
- 支持从 GitHub 克隆（备选）
- 自动验证下载完整性

### 2. 修改编译脚本支持 GLFW ✅

**文件**：`build_external_tools.sh` - `build_mias_lcec()` 函数

**新增功能**：
```bash
# 检查 GLFW 是否已安装
if ! pkg-config --exists glfw3 2>/dev/null; then
    # 从本地源码编译安装 GLFW
    local glfw_root="${CALIB_ROOT}/docker/deps/glfw"
    cd /tmp/glfw_build
    cmake "${glfw_root}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON
    make -j$(nproc)
    make install
fi
```

**工作原理**：
1. 使用 `pkg-config` 检查 GLFW 是否已安装
2. 如果未安装，从 `docker/deps/glfw/` 编译
3. 编译后安装到系统（`/usr/local/lib`、`/usr/local/include`）

---

## 🚀 立即执行修复

### 步骤 1：下载 GLFW 到本地

```bash
cd /home/wqs/Documents/github/UniCalib/docker/deps
bash download_glfw.sh
```

**预期输出**：
```
==========================================
GLFW 下载脚本
==========================================

[INFO] 方法 1: 从 GitHub Releases 下载...
使用 wget 下载...
[SUCCESS] 从 GitHub Releases 下载成功

[SUCCESS] GLFW 下载完成
目录: /home/wqs/Documents/github/UniCalib/docker/deps/glfw

目录内容:
total 56
drwxrwxr-x  7 wqs wqs  4096 ... glfw
drwxrwxr-x 17 wqs wqs  4096 ... .
...
```

**验证**：
```bash
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glfw/include/GLFW/glfw3.h
```

**预期输出**：
```
-rw-r--r-- 1 wqs wqs 189K ... glfw3.h
```

### 步骤 2：重新执行编译

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

### 步骤 3：查看编译输出

**预期成功输出**（MIAS-LCEC 部分）：
```
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 检查 GLFW 是否已安装...
[INFO] GLFW 未安装，开始编译安装...
[INFO] 使用本地 GLFW 源码: /root/calib_ws/docker/deps/glfw
-- The C compiler identification is GNU 11.4.0
-- The CXX compiler identification is GNU 11.4.0
-- Configuring done
-- Generating done
-- Build files have been written to: /tmp/glfw_build
[  1%] Building C object ...
[ 50%] Building C object ...
[100%] Built target glfw
[SUCCESS] GLFW 编译安装完成
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[INFO] 编译 C++ 扩展...
-- Found GLM: /root/calib_ws/docker/deps/glm
-- Found OpenGL: /usr/lib/x86_64-linux-gnu/libOpenGL.so
-- Found GLFW: /usr/local/lib/libglfw.so  ← GLFW 找到了！
[PASS] MIAS-LCEC 编译完成
```

---

## 📊 依赖目录结构

```
docker/deps/
├── glm/              ← GLM 0.9.9.8 (Header-only)
├── glfw/             ← GLFW 3.3.8 (需要编译)
├── cmake/            ← CMake 3.24
├── gtsam/            ← GTSAM 4.2
├── Sophus/           ← Sophus
├── magic_enum/       ← magic_enum
├── ceres-solver/     ← Ceres
├── opencv/           ← OpenCV 4.8.0
└── ...
```

---

## 🔍 编译流程

### GLFW 编译流程

```
build_mias_lcec()
  ↓ (检查 GLFW 是否安装)
  ↓ (pkg-config --exists glfw3)
  ↓ (如果未安装)
  ↓ (使用本地 deps/glfw)
  ↓ (cmake 配置)
  ↓ (make 编译)
  ↓ (make install 安装)
  ↓ (ldconfig 更新库缓存)
  ↓ (GLFW 安装到 /usr/local)
  ↓ (继续编译 MIAS-LCEC)
  ↓ (CMake 找到 GLFW)
  ↓ (编译成功 ✅)
```

### 环境变量

| 变量 | 作用 | 设置位置 |
|-----|------|---------|
| `GLM_ROOT_DIR` | GLM 根目录 | `build_mias_lcec()` |
| `PKG_CONFIG_PATH` | pkg-config 搜索路径 | `build_mias_lcec()` |

---

## ✅ 验证步骤

### 1. 验证 GLFW 已下载

```bash
# 主机上检查
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glfw/include/GLFW/glfw3.h

# 或
ls -la docker/deps/glfw/include/GLFW/glfw3.h
```

### 2. 验证 GLFW 编译安装

编译 MIAS-LCEC 后，验证 GLFW 已安装：

```bash
docker run --rm \
    -v $(pwd):/root/calib_ws \
    calib_env:humble \
    pkg-config --modversion glfw3
```

**预期输出**：
```
3.3.8
```

### 3. 验证 CMake 找到 GLFW

查看编译日志，应包含：
```
-- Found GLFW: /usr/local/lib/libglfw.so
```

---

## 🐛 故障排查

### 问题 1：GLFW 下载失败

**现象**：下载脚本失败

**解决方案**：
```bash
# 手动下载
cd /home/wqs/Documents/github/UniCalib/docker/deps
wget https://github.com/glfw/glfw/releases/download/3.3.8/glfw-3.3.8.zip
unzip glfw-3.3.8.zip
mv glfw-3.3.8 glfw
rm -f glfw-3.3.8.zip
```

### 问题 2：GLFW 编译失败

**现象**：`make install` 失败

**排查**：
```bash
# 检查依赖是否完整
docker run --rm calib_env:humble \
    apt-cache depends libglfw3-dev

# 或手动检查依赖
apt list --installed | grep -E "x11|wayland"
```

**依赖列表**：
- `libx11-dev` (X11)
- `libxrandr-dev` (XRandR)
- `libxinerama-dev` (Xinerama)
- `libxcursor-dev` (XCursor)
- `libxi-dev` (XI)
- `libwayland-dev` (Wayland，可选)

### 问题 3：GLFW 安装后仍然找不到

**现象**：`Could NOT find GLFW`

**排查**：
```bash
# 检查 pkg-config
docker run --rm calib_env:humble \
    pkg-config --cflags glfw3

# 检查库文件
docker run --rm calib_env:humble \
    find /usr -name "libglfw.so*"

# 检查头文件
docker run --rm calib_env:humble \
    find /usr -name "glfw3.h"
```

**解决方案**：
```bash
# 更新库缓存
ldconfig

# 重新编译
bash build_and_run.sh
```

---

## 📚 相关文档

| 文档 | 用途 |
|-----|------|
| `GLM_FIX_SUMMARY.md` | GLM 修复总结 |
| `docker/download_glfw.sh` | GLFW 下载脚本 |
| `docker/download_glm.sh` | GLM 下载脚本 |
| `docker/DEPS_README.md` | 依赖管理文档 |

---

## 🎯 快速执行

```bash
# 1. 下载 GLFW
cd /home/wqs/Documents/github/UniCalib/docker/deps
bash download_glfw.sh

# 2. 回到项目根目录
cd /home/wqs/Documents/github/UniCalib

# 3. 重新编译
bash build_and_run.sh
```

**预期结果**：GLFW 编译安装成功，MIAS-LCEC 编译通过！🎉

---

## 🔄 后续优化

### 短期（V1）
1. 预编译 GLFW 到镜像中（减少重复编译）
2. 缓存 GLFW 编译产物
3. 支持多版本 GLFW 切换

### 中期（V2）
1. 集成到自动构建流程
2. 支持并行编译多个依赖
3. 添加编译产物完整性检查

### 长期（V3）
1. 建立依赖预编译仓库
2. 支持 CI/CD 自动化
3. 实现依赖版本管理

---

## 📞 技术支持

### 快速命令参考

```bash
# 查看所有依赖
ls -la docker/deps/

# 下载 GLFW
cd docker/deps && bash download_glfw.sh

# 验证 GLFW 文件
ls -la docker/deps/glfw/include/GLFW/glfw3.h

# 检查 GLFW 是否已安装（在容器内）
docker run --rm calib_env:humble pkg-config --modversion glfw3

# 仅编译外部工具
bash build_and_run.sh --build-external-only

# 跳过某个工具
bash build_external_tools.sh --skip mias_lcec

# 仅编译 MIAS-LCEC
bash build_external_tools.sh --tools mias_lcec
```

---

## 🎉 总结

### 核心修复点

1. ✅ **下载 GLFW**：下载 GLFW 3.3.8 到 `docker/deps/glfw/`
2. ✅ **动态安装**：在编译 MIAS-LCEC 时编译安装 GLFW
3. ✅ **不修改 Dockerfile**：保持 Dockerfile 原始状态
4. ✅ **本地优先**：使用本地 deps 目录，避免网络问题

### 优势

- 🚀 **灵活性高**：无需重建镜像
- 🔒 **版本可控**：锁定 GLFW 3.3.8
- 🛡️ **可靠性高**：本地源码 + 动态编译
- 📦 **一致性**：与其他 deps 管理方式一致

### 立即执行

```bash
cd /home/wqs/Documents/github/UniCalib/docker/deps
bash download_glfw.sh
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

**预期结果**：所有 6 个外部工具编译成功！🎉
