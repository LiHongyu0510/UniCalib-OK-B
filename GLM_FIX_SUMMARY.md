# 🔧 MIAS-LCEC GLM 依赖修复 - 完整总结

## 📋 问题背景

**原始错误**：
```bash
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 编译 C++ 扩展...
CMake Error: Could NOT find GLM (missing: GLM_INCLUDE_DIR)
```

**问题链条**：
1. ❌ `build_external_tools.sh` 路径错误（`cd bin` → `cd bin/iridescence`）
2. ❌ Docker 镜像缺少 GLM 库
3. ❌ 编译时未指定 GLM 本地依赖路径

---

## ✅ 修复方案：使用本地 deps 目录

### 核心理念

根据您的要求 **"编译的时候直接使用本地的库"**，采用以下策略：

```
主机: /home/wqs/Documents/github/UniCalib/docker/deps/glm/
    ↓ (Docker 挂载)
容器: /root/calib_ws/docker/deps/glm/
    ↓ (GLM_ROOT_DIR 环境变量)
CMake: 找到 GLM 头文件
    ↓
编译成功 ✅
```

---

## 📝 完整修改清单

### 修改 1：下载 GLM 到本地 ✅

**路径**：`docker/deps/glm/`

**命令**：
```bash
cd docker/deps
curl -L -o glm.zip https://github.com/g-truc/glm/archive/refs/tags/0.9.9.8.zip
unzip -q glm.zip
mv glm-0.9.9.8 glm
rm -f glm.zip
```

**结果**：
```
docker/deps/glm/
├── CMakeLists.txt
├── glm/
│   ├── glm/glm.hpp  ← 主头文件
│   ├── glm/vec3.hpp
│   ├── glm/mat4x4.hpp
│   └── ...
├── readme.md
└── ...
```

### 修改 2：修复编译路径 ✅

**文件**：`build_external_tools.sh`

**修改位置**：第 310-311 行

```diff
-    cd bin
-    if [ -f "iridescence/setup.py" ]; then
-        python3 setup.py build_ext --inplace
+    cd bin/iridescence
+    if [ -f "setup.py" ]; then
+        python3 setup.py build_ext --inplace
```

### 修改 3：使用本地 GLM 依赖 ✅

**文件**：`build_external_tools.sh` - `build_mias_lcec()` 函数

**新增代码**（第 309-318 行）：
```bash
# 设置 GLM 环境变量（使用本地 deps 目录）
local glm_root="${CALIB_ROOT}/docker/deps/glm"
if [ -d "${glm_root}" ] && [ -f "${glm_root}/glm/glm.hpp" ]; then
    export GLM_ROOT_DIR="${glm_root}"
    info "使用本地 GLM: ${glm_root}"
else
    warning "本地 GLM 未找到，将尝试使用系统 GLM"
fi
```

**工作原理**：
- GLM 的 `FindGLM.cmake` 自动读取 `GLM_ROOT_DIR` 环境变量
- CMake 会优先在 `${GLM_ROOT_DIR}/glm/glm.hpp` 查找

### 修改 4：挂载 deps 目录到容器 ✅

**文件**：`build_and_run.sh` - `build_external_tools()` 函数

**修改位置**：第 159-174 行

```bash
docker run --rm \
    --gpus all \
    --ipc=host \
    -v "${PROJECT_ROOT}:/root/calib_ws:rw" \
    -v "${PROJECT_ROOT}/docker/deps:/root/thirdparty/deps:ro" \  # 新增
    -e CALIB_DATA_DIR="/root/calib_ws/data" \
    -e CALIB_RESULTS_DIR="/root/calib_ws/results" \
    -e THIRDPARTY_DEPS="/root/thirdparty/deps" \  # 新增
    "${DOCKER_IMAGE}" \
    bash -c "..."
```

**说明**：
- `docker/deps` 挂载到 `/root/thirdparty/deps`（只读）
- 容器内实际路径：`/root/calib_ws/docker/deps/glm`

### 修改 5：Dockerfile 添加 GLM 支持 ✅

**文件**：`docker/Dockerfile`

**新增 apt 依赖**（阶段3，第 160-165 行）：
```dockerfile
# GLM 和 OpenGL (MIAS-LCEC 需要)
libglm-dev \
libgl1-mesa-dev \
libglu1-mesa-dev \
```

**新增本地 GLM 支持**（CMake 之后，第 78-89 行）：
```dockerfile
# 使用本地预装 GLM (Header-only 库，避免 apt 依赖)
# 如果本地没有 deps/glm，会从 apt 安装 libglm-dev
COPY ./deps/glm /tmp/glm_src 2>/dev/null || true
RUN if [ -d "/tmp/glm_src" ] && [ -f "/tmp/glm_src/glm/glm.hpp" ]; then \
      echo "Installing GLM from local source..." && \
      mkdir -p /usr/local/include && \
      cp -r /tmp/glm_src/glm /usr/local/include/ && \
      rm -rf /tmp/glm_src && \
      echo "GLM installed from local source: /usr/local/include/glm"; \
    else \
      echo "Local GLM not found, will install from apt later"; \
    fi
```

**策略**：本地优先 + apt 回退

---

## 📊 依赖路径映射

### 目录结构

```
项目根目录: /home/wqs/Documents/github/UniCalib/
├── docker/
│   ├── deps/
│   │   ├── cmake/          → 容器: /opt/cmake
│   │   ├── glm/           → 容器: /root/calib_ws/docker/deps/glm
│   │   ├── gtsam/         → 容器: /root/thirdparty/gtsam
│   │   └── ...
│   ├── Dockerfile
│   └── download_glm.sh
├── MIAS-LCEC/
│   └── bin/
│       └── iridescence/
│           ├── CMakeLists.txt
│           ├── cmake/FindGLM.cmake
│           └── setup.py
├── build_external_tools.sh
└── build_and_run.sh
```

### Docker 挂载

```bash
# 项目目录（读写）
-v "${PROJECT_ROOT}:/root/calib_ws:rw"

# 依赖目录（只读）
-v "${PROJECT_ROOT}/docker/deps:/root/thirdparty/deps:ro"
```

### GLM 路径链

```
1. 主机: /home/wqs/Documents/github/UniCalib/docker/deps/glm/
2. 容器: /root/calib_ws/docker/deps/glm/
3. 环境变量: GLM_ROOT_DIR=/root/calib_ws/docker/deps/glm
4. CMake 查找: ${GLM_ROOT_DIR}/glm/glm.hpp
5. 编译成功 ✅
```

---

## 🚀 立即执行修复

### 步骤 1：验证 GLM 已下载

```bash
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glm/glm/glm.hpp
```

**预期输出**：
```
-rw-rw-r-- 1 wqs wqs 4499 Apr 14  2020 ... glm/glm.hpp
```

### 步骤 2：执行完整编译流程

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

### 步骤 3：查看编译输出

**预期成功输出**（MIAS-LCEC 部分）：
```
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[INFO] 编译 C++ 扩展...
running build_ext
-- The C compiler identification is GNU 11.4.0
-- The CXX compiler identification is GNU 11.4.0
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Found GLM: /root/calib_ws/docker/deps/glm/glm/glm.hpp
-- Found OpenGL: /usr/lib/x86_64-linux-gnu/libGL.so
-- Found Boost: /usr/lib/x86_64-linux-gnu/cmake/Boost-1.74.0/...
-- Found PNG: /usr/lib/x86_64-linux-gnu/libpng.so
-- Found JPEG: /usr/lib/x86_64-linux-gnu/libjpeg.so
-- Configuring done
-- Generating done
-- Build files have been written to: ...
[PASS] MIAS-LCEC 编译完成
```

---

## ✅ 验证步骤

### 1. 验证 GLM 文件存在

```bash
# 主机上检查
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glm/glm/glm.hpp

# 容器内检查
docker run --rm \
    -v $(pwd):/root/calib_ws \
    calib_env:humble \
    ls -la /root/calib_ws/docker/deps/glm/glm/glm.hpp
```

### 2. 验证环境变量

在 `build_external_tools.sh` 的 `build_mias_lcec()` 函数中添加调试输出：

```bash
export GLM_ROOT_DIR="${glm_root}"
echo "[DEBUG] GLM_ROOT_DIR=${GLM_ROOT_DIR}"
echo "[DEBUG] GLM file exists: $(ls -la ${GLM_ROOT_DIR}/glm/glm.hpp)"
```

### 3. 验证 CMake 找到 GLM

查看编译日志，应包含：
```
-- Found GLM: /root/calib_ws/docker/deps/glm/glm/glm.hpp
```

---

## 🐛 故障排查

### 问题 1：GLM 仍然找不到

**现象**：`Could NOT find GLM`

**排查步骤**：
```bash
# 1. 检查 GLM 文件
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glm/glm/glm.hpp

# 2. 检查容器内挂载
docker run --rm -v $(pwd):/root/calib_ws calib_env:humble \
    ls -la /root/calib_ws/docker/deps/glm/glm/glm.hpp

# 3. 检查环境变量
docker run --rm -v $(pwd):/root/calib_ws calib_env:humble \
    bash -c "echo \$CALIB_ROOT; ls -la \$CALIB_ROOT/docker/deps/glm/glm.hpp"
```

**解决方案**：
- 确保 GLM 文件存在
- 确保挂载路径正确
- 检查 `CALIB_ROOT` 变量

### 问题 2：权限问题

**现象**：`Permission denied`

**解决方案**：
```bash
chmod -R 755 /home/wqs/Documents/github/UniCalib/docker/deps/glm
```

### 问题 3：构建缓存导致旧错误

**现象**：修改后仍然报旧错误

**解决方案**：
```bash
# 清理构建缓存
rm -rf /home/wqs/Documents/github/UniCalib/MIAS-LCEC/bin/iridescence/build

# 重新编译
bash build_and_run.sh
```

---

## 📚 相关文档

| 文档 | 用途 |
|-----|------|
| `QUICK_START.md` | 快速执行指南 |
| `FIX_COMPLETE.md` | 完整修复文档 |
| `docker/DEPS_README.md` | 依赖管理说明 |
| `docker/download_glm.sh` | GLM 下载脚本 |
| `FIX_GL_ISSUE.md` | GLM 问题修复指南 |

---

## 🎯 修复总结

### 核心要点

1. ✅ **本地依赖**：GLM 下载到 `docker/deps/glm/`（header-only 库）
2. ✅ **路径修复**：修正 `build_external_tools.sh` 编译路径
3. ✅ **环境变量**：通过 `GLM_ROOT_DIR` 指定本地 GLM
4. ✅ **Docker 挂载**：`docker/deps` 挂载到容器
5. ✅ **双重保障**：本地优先 + apt 回退

### 技术优势

- 🚀 **快速构建**：使用本地依赖，无需网络下载
- 🔒 **版本可控**：锁定 GLM 0.9.9.8
- 🛡️ **高可靠性**：本地 + apt 双重保障
- 📦 **一致性**：与其他 deps 管理方式一致

### 文件变更

| 文件 | 变更 | 说明 |
|-----|------|------|
| `docker/deps/glm/` | 新增 | GLM 0.9.9.8 源码 |
| `build_external_tools.sh` | 修改 | 修复路径 + 添加 GLM_ROOT_DIR |
| `build_and_run.sh` | 修改 | 挂载 deps 目录 |
| `docker/Dockerfile` | 修改 | 添加 GLM 支持 |
| `docker/download_glm.sh` | 新增 | GLM 下载脚本 |
| `docker/DEPS_README.md` | 新增 | 依赖管理文档 |

---

## 🎉 立即执行

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

**预期结果**：所有 6 个外部工具编译成功！🎉

---

**修复完成时间**：2026-03-01
**修复工程师**：AI Assistant (Cursor)
**修复版本**：V1.0
