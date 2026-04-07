# MIAS-LCEC GLM 依赖修复指南

## 📋 问题说明

编译 MIAS-LCEC 时报错：
```
Could NOT find GLM (missing: GLM_INCLUDE_DIR)
```

## ✅ 已完成的修复

### 1. 修复编译路径错误
**文件**：`build_external_tools.sh`
- 修正工作目录：`cd bin` → `cd bin/iridescence`
- 修正文件检查：`iridescence/setup.py` → `setup.py`

### 2. 添加 GLM 依赖支持
**文件**：`docker/Dockerfile`

**策略**：混合方案（本地优先 + apt 回退）

```dockerfile
# 本地 GLM（优先）
COPY ./deps/glm /tmp/glm_src 2>/dev/null || true
RUN if [ -d "/tmp/glm_src" ]; then \
      cp -r /tmp/glm_src/glm /usr/local/include/; \
      rm -rf /tmp/glm_src; \
    fi

# apt GLM（回退）
RUN apt-get install -y libglm-dev libgl1-mesa-dev libglu1-mesa-dev
```

### 3. 创建依赖下载工具
**文件**：`docker/download_glm.sh`
- 支持从 GitHub Releases 下载（推荐）
- 支持从 GitHub 克隆（备选）
- 自动验证下载完整性

### 4. 依赖管理文档
**文件**：`docker/DEPS_README.md`
- 完整的依赖目录说明
- 下载和使用指南
- 故障排查指南

---

## 🚀 立即执行修复

### 方案 A：重建镜像（使用 apt 安装 GLM）

**推荐使用此方案**，最快且无需下载额外依赖：

```bash
cd /home/wqs/Documents/github/UniCalib/docker

# 重建 Docker 镜像
bash docker_build.sh

# 如果遇到构建缓存问题，使用 --no-cache
# bash docker_build.sh --no-cache
```

**预期时间**：15-30 分钟（取决于机器性能和网络）

### 方案 B：下载 GLM 到本地（可选优化）

如果 GitHub 网络良好，可以将 GLM 下载到本地（避免 apt 依赖）：

```bash
cd /home/wqs/Documents/github/UniCalib/docker/deps

# 下载 GLM
bash download_glm.sh

# 返回并重建镜像
cd ../..
bash build_and_run.sh
```

**优势**：
- ✅ 避免依赖 apt 仓库
- ✅ 版本可控（锁定 0.9.9.8）
- ✅ 与其他 deps 管理方式一致

---

## 📊 修改详情

### 1. build_external_tools.sh

**修改前**：
```bash
cd bin
if [ -f "iridescence/setup.py" ]; then
    python3 setup.py build_ext --inplace
```

**修改后**：
```bash
cd bin/iridescence
if [ -f "setup.py" ]; then
    python3 setup.py build_ext --inplace
```

### 2. Dockerfile

**新增依赖**（阶段3：核心数学库）：
```dockerfile
# GLM 和 OpenGL (MIAS-LCEC 需要)
libglm-dev \
libgl1-mesa-dev \
libglu1-mesa-dev \
```

**新增本地 GLM 支持**（CMake 之后）：
```dockerfile
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

---

## ✅ 验证步骤

重建镜像后，验证编译：

```bash
# 完整流程
cd /home/wqs/Documents/github/UniCalib
bash build_and_run.sh

# 或仅编译外部工具
bash build_and_run.sh --build-external-only
```

**预期输出**：
```
>>> 编译 MIAS-LCEC (C++ + Python)
[INFO] 编译 C++ 扩展...
running build_ext
-- Found GLM: /usr/local/include/glm (或 /usr/include/glm)
-- Found OpenGL: /usr/lib/x86_64-linux-gnu/libGL.so
[PASS] MIAS-LCEC 编译完成
```

---

## 🐛 故障排查

### 问题 1：GLM 仍然找不到

**现象**：`Could NOT find GLM`

**解决方案**：
```bash
# 方案 1：重建镜像（无缓存）
docker build --no-cache -t calib_env:humble docker/

# 方案 2：下载 GLM 到本地
cd docker/deps && bash download_glm.sh
cd ../.. && bash build_and_run.sh
```

### 问题 2：构建超时

**现象**：apt 安装过程超时

**解决方案**：
```bash
# 使用本地 GLM（避免网络问题）
cd docker/deps && bash download_glm.sh
cd ../.. && bash build_and_run.sh
```

### 问题 3：OpenGL 相关错误

**现象**：`Could NOT find OpenGL`

**解决方案**：
```bash
# 确保镜像中安装了 OpenGL 相关包
# Dockerfile 已包含：
# - libgl1-mesa-dev
# - libglu1-mesa-dev

# 重建镜像即可
bash docker/docker_build.sh
```

---

## 📚 相关文件

- `build_external_tools.sh` - 外部工具编译脚本
- `docker/Dockerfile` - Docker 镜像定义
- `docker/download_glm.sh` - GLM 下载脚本
- `docker/DEPS_README.md` - 依赖管理文档

---

## 🔄 后续优化

1. **预编译 MIAS-LCEC**：将 MIAS-LCEC 的 C++ 扩展预编译到镜像中
2. **缓存优化**：将 GLM 等常用依赖打包到基础镜像层
3. **CI/CD 集成**：在 CI 管道中自动构建和测试

---

## 📞 支持

如有问题，请检查：
1. Docker 镜像是否成功重建
2. GLM 是否正确安装（`docker run --rm calib_env:humble ls /usr/include/glm`）
3. 构建日志中的详细错误信息
