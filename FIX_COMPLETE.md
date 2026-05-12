# 🎯 MIAS-LCEC GLM 依赖修复 - 完整解决方案

## 📋 问题总结

**原始错误**：
```
Could NOT find GLM (missing: GLM_INCLUDE_DIR)
```

**根本原因**：
1. `build_external_tools.sh` 中 MIAS-LCEC 编译路径错误
2. Docker 镜像缺少 GLM 库
3. 编译时未指定 GLM 本地依赖路径

---

## ✅ 已完成的修复

### 1. 下载 GLM 到本地 deps 目录 ✅

```bash
cd docker/deps
curl -L -o glm.zip https://github.com/g-truc/glm/archive/refs/tags/0.9.9.8.zip
unzip glm.zip
mv glm-0.9.9.8 glm
rm -f glm.zip
```

**验证**：
```bash
ls -la docker/deps/glm/glm/glm.hpp
# 输出: -rw-rw-r-- 1 wqs wqs 4499 ... glm/glm.hpp
```

### 2. 修复编译路径 ✅

**文件**：`build_external_tools.sh`

**修改内容**：
```bash
# 修改前
cd bin
python3 setup.py build_ext --inplace

# 修改后
cd bin/iridescence
python3 setup.py build_ext --inplace
```

### 3. 使用本地 GLM 依赖 ✅

**文件**：`build_external_tools.sh` - `build_mias_lcec()` 函数

**新增代码**：
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
- GLM 的 `FindGLM.cmake` 会读取 `GLM_ROOT_DIR` 环境变量
- 指定后 CMake 会优先在 `${GLM_ROOT_DIR}/glm/glm.hpp` 查找

### 4. 挂载 deps 目录到容器 ✅

**文件**：`build_and_run.sh` - `build_external_tools()` 函数

**新增挂载**：
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
- `docker/deps` 挂载到容器 `/root/thirdparty/deps`（只读）
- 容器内路径为 `/root/calib_ws/docker/deps`

### 5. Dockerfile 添加 GLM 支持 ✅

**文件**：`docker/Dockerfile`

**新增 apt 依赖**：
```dockerfile
# GLM 和 OpenGL (MIAS-LCEC 需要)
libglm-dev \
libgl1-mesa-dev \
libglu1-mesa-dev \
```

**新增本地 GLM 支持**：
```dockerfile
COPY ./deps/glm /tmp/glm_src 2>/dev/null || true
RUN if [ -d "/tmp/glm_src" ]; then \
      cp -r /tmp/glm_src/glm /usr/local/include/; \
    fi
```

---

## 📊 文件变更清单

| 文件路径 | 变更类型 | 行号变更 | 说明 |
|---------|---------|---------|------|
| `docker/deps/glm/` | 新增目录 | - | GLM 0.9.9.8 源码 |
| `build_external_tools.sh` | 修改 | 310-318 | 修复路径 + 添加 GLM_ROOT_DIR |
| `build_and_run.sh` | 修改 | 159-174 | 挂载 deps 目录 |
| `docker/Dockerfile` | 修改 | 78-89, 160-165 | 添加 GLM 支持 |
| `docker/download_glm.sh` | 新增 | - | GLM 下载脚本 |
| `docker/DEPS_README.md` | 新增 | - | 依赖管理文档 |

---

## 🚀 立即执行修复

### 步骤 1：验证 GLM 已下载

```bash
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glm/glm/glm.hpp
```

**预期输出**：
```
-rw-rw-r-- 1 wqs wqs 4499 ... glm/glm.hpp
```

### 步骤 2：执行完整编译流程

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

**预期输出**（MIAS-LCEC 部分）：
```
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm
[INFO] 编译 C++ 扩展...
running build_ext
-- The C compiler identification is GNU 11.4.0
-- The CXX compiler identification is GNU 11.4.0
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

## 🔍 工作原理

### 依赖路径映射

```
主机:
/home/wqs/Documents/github/UniCalib/docker/deps/glm/
    └── glm/
        └── glm.hpp

容器内 (挂载):
/root/calib_ws/docker/deps/glm/
    └── glm/
        └── glm.hpp

GLM_ROOT_DIR 环境变量:
/root/calib_ws/docker/deps/glm
```

### CMake 查找 GLM 的流程

1. 读取 `GLM_ROOT_DIR` 环境变量
2. 在 `${GLM_ROOT_DIR}/glm/glm.hpp` 查找
3. 如果找到，设置 `GLM_INCLUDE_DIR`
4. 编译时使用该头文件路径

### 编译命令链

```
build_and_run.sh
  ↓ (挂载 docker/deps → /root/thirdparty/deps)
  ↓ (执行 build_external_tools.sh)
  ↓ (调用 build_mias_lcec)
  ↓ (设置 GLM_ROOT_DIR)
  ↓ (cd bin/iridescence)
  ↓ (python3 setup.py build_ext --inplace)
  ↓ (调用 CMake)
  ↓ (FindGLM.cmake 找到 GLM)
  ↓ (编译成功 ✅)
```

---

## ✅ 验证步骤

### 1. 验证 GLM 挂载

```bash
docker run --rm \
    -v $(pwd):/root/calib_ws \
    calib_env:humble \
    ls -la /root/calib_ws/docker/deps/glm/glm/glm.hpp
```

### 2. 验证 GLM 环境变量

修改 `build_external_tools.sh` 临时添加调试输出：

```bash
export GLM_ROOT_DIR="${glm_root}"
echo "[DEBUG] GLM_ROOT_DIR=${GLM_ROOT_DIR}"
ls -la "${GLM_ROOT_DIR}/glm/glm.hpp"
```

### 3. 验证 CMake 找到 GLM

查看编译日志中的输出：

```
-- Found GLM: /root/calib_ws/docker/deps/glm/glm/glm.hpp
```

---

## 🐛 故障排查

### 问题 1：GLM 仍然找不到

**现象**：`Could NOT find GLM`

**排查**：
```bash
# 检查 GLM 文件是否存在
ls -la /home/wqs/Documents/github/UniCalib/docker/deps/glm/glm/glm.hpp

# 检查容器内挂载
docker run --rm -v $(pwd):/root/calib_ws calib_env:humble \
    ls -la /root/calib_ws/docker/deps/glm/glm/glm.hpp
```

**解决方案**：
- 确保 GLM 文件存在
- 确保挂载路径正确
- 检查 `GLM_ROOT_DIR` 环境变量

### 问题 2：路径映射错误

**现象**：`[INFO] 使用本地 GLM: /root/calib_ws/docker/deps/glm` 但 GLM 未找到

**排查**：
```bash
# 检查工作目录
echo $CALIB_ROOT  # 应为 /root/calib_ws

# 检查 GLM 路径
ls -la ${CALIB_ROOT}/docker/deps/glm/glm/glm.hpp
```

**解决方案**：
- 确保 `CALIB_ROOT` 设置正确
- 检查 `docker/deps` 是否在项目根目录

### 问题 3：权限问题

**现象**：`Permission denied` 或 `No such file or directory`

**解决方案**：
```bash
# 检查文件权限
chmod -R 755 docker/deps/glm

# 或使用 sudo
sudo chmod -R 755 docker/deps/glm
```

---

## 📚 相关文档

- `docker/DEPS_README.md` - 依赖管理说明
- `docker/download_glm.sh` - GLM 下载脚本
- `MIAS-LCEC/bin/iridescence/cmake/FindGLM.cmake` - GLM 查找逻辑

---

## 🔄 后续优化

### 短期（V1）

1. **预编译 MIAS-LCEC**：将 C++ 扩展编译到镜像中
2. **依赖缓存**：优化 Dockerfile 层级，减少重建时间
3. **编译检查**：添加编译产物完整性验证

### 中期（V2）

1. **多工具支持**：扩展 GLM 本地化策略到其他依赖
2. **版本管理**：支持多个 GLM 版本切换
3. **CI/CD 集成**：自动化依赖下载和验证

### 长期（V3）

1. **依赖仓库**：建立内部依赖镜像仓库
2. **依赖管理工具**：开发统一的依赖管理 CLI
3. **智能缓存**：基于依赖哈希的增量构建

---

## 📞 技术支持

### 快速命令参考

```bash
# 查看所有依赖
ls -la docker/deps/

# 重新下载 GLM
cd docker/deps && bash download_glm.sh

# 验证 GLM 文件
ls -la docker/deps/glm/glm/glm.hpp

# 检查容器内挂载
docker run --rm -v $(pwd):/root/calib_ws calib_env:humble \
    bash -c "ls -la /root/calib_ws/docker/deps/glm/glm/glm.hpp"

# 仅编译外部工具
bash build_and_run.sh --build-external-only

# 跳过某个工具
bash build_external_tools.sh --skip mias_lcec

# 仅编译指定工具
bash build_external_tools.sh --tools mias_lcec
```

### 关键环境变量

| 变量 | 作用 | 默认值 |
|-----|------|--------|
| `GLM_ROOT_DIR` | GLM 根目录 | `${CALIB_ROOT}/docker/deps/glm` |
| `CALIB_ROOT` | 项目根目录 | 脚本所在目录 |
| `THIRDPARTY_DEPS` | 第三方依赖目录 | `/root/thirdparty/deps` |

---

## 🎉 总结

### 核心修复点

1. ✅ **下载 GLM**：将 GLM 0.9.9.8 下载到 `docker/deps/glm`
2. ✅ **修复路径**：修正 `build_external_tools.sh` 中的 MIAS-LCEC 编译路径
3. ✅ **本地依赖**：通过 `GLM_ROOT_DIR` 指定本地 GLM 路径
4. ✅ **挂载目录**：在 `build_and_run.sh` 中挂载 `docker/deps` 到容器
5. ✅ **多重保障**：Dockerfile 添加 apt 安装作为回退

### 优势

- 🚀 **构建速度快**：使用本地依赖，无需网络下载
- 🔒 **版本可控**：锁定 GLM 版本为 0.9.9.8
- 🛡️ **可靠性高**：本地依赖 + apt 回退双重保障
- 📦 **一致性**：与项目其他 deps 管理方式一致

### 立即执行

```bash
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh
```

**预期结果**：所有 6 个外部工具编译成功！🎉
