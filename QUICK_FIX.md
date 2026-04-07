# 快速修复：OpenCV IPPICV 下载卡住问题

## 问题描述

构建 Docker 镜像时卡在 IPPICV 下载：
```
#15 1841.3 -- IPPICV: Downloading ippicv_2021.8_lnx_intel64_20230330_general.tgz
```

## 解决方案（3 步骤）

### 步骤 1：下载依赖文件

```bash
cd /home/wqs/Documents/github/calibration/docker
./download_opencv_deps.sh
```

**预期输出**：
```
===== OpenCV 依赖下载脚本 =====
✅ IPPICV 下载完成 (~50MB)
```

### 步骤 2：验证文件已下载

```bash
ls -lh ../deps/opencv_deps/
```

**预期输出**：
```
ippicv_2021.8_lnx_intel64_20230330_general.tgz
```

### 步骤 3：重新构建镜像

```bash
# 方式 A：使用构建脚本（推荐，自动检查依赖）
./docker_build.sh

# 方式 B：直接构建
docker build -t calib_env:humble ..
```

## 关键修改

### Dockerfile（已自动修改）

```dockerfile
# 新增：复制预下载的依赖
COPY ./deps/opencv_deps ${THIRDPARTY_WS}/opencv_deps

# CMake 新增参数（指向本地文件）
cmake .. \
    -DOPENCV_IPPICV_DOWNLOADED_FILE=1 \
    -DOPENCV_IPPICV_DOWNLOAD_PATH=${THIRDPARTY_WS}/opencv_deps \
    ...
```

## 验证修复

构建日志中**不再出现**以下内容：
```
-- IPPICV: Downloading ippicv_2021.8_lnx_intel64_20230330_general.tgz
from https://raw.githubusercontent.com/...
```

取而代之，应该看到：
```
-- Using cached IPPICV: /root/thirdparty/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz
```

## 常见问题

**Q: 手动下载失败怎么办？**
```bash
# 使用代理（如果需要）
export https_proxy=http://your-proxy:port
./download_opencv_deps.sh
```

**Q: 文件已存在但仍提示下载？**
```bash
# 检查文件路径是否正确
ls -lh deps/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz

# 检查文件权限
chmod 644 deps/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz
```

**Q: 需要在 CI/CD 中使用？**
在 CI 脚本中添加下载步骤：
```yaml
# GitLab CI 示例
before_script:
  - cd docker && ./download_opencv_deps.sh
```

## 文件清单

| 文件 | 状态 | 说明 |
|------|------|------|
| `docker/download_opencv_deps.sh` | ✅ 已创建 | 自动下载脚本 |
| `deps/opencv_deps/` | ✅ 已创建 | 依赖文件目录 |
| `docker/Dockerfile` | ✅ 已修改 | 添加 COPY + CMake 参数 |
| `docker/docker_build.sh` | ✅ 已修改 | 添加依赖检查 |
| `.gitignore` | ✅ 已创建 | 排除大文件 |
| `docs/opencv-build-flow.md` | ✅ 已创建 | 详细文档 |

---

**立即执行命令**：
```bash
cd /home/wqs/Documents/github/calibration/docker && ./download_opencv_deps.sh
```
