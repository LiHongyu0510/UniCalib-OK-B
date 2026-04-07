# OpenCV 依赖下载优化指南

## 问题背景
原始下载脚本从 GitHub raw 下载速度仅 37.3KB/s，预计需要 15 分钟以上完成 31MB 文件下载。

## 优化方案

### 方案对比

| 方案 | 预期速度 | 实现难度 | 推荐度 |
|------|---------|---------|--------|
| 原始脚本 | 37KB/s | - | ⭐ |
| 国内镜像源 | 1-5MB/s | ⭐ | ⭐⭐⭐⭐⭐ |
| aria2c 多线程 | 500KB-2MB/s | ⭐⭐ | ⭐⭐⭐⭐ |
| **镜像源+aria2c** | **2-10MB/s** | ⭐⭐ | **⭐⭐⭐⭐⭐** |

## 快速使用

### 1. 安装 aria2c（推荐）
```bash
sudo apt-get update
sudo apt-get install -y aria2
```

### 2. 使用优化脚本
```bash
cd /home/wqs/Documents/github/calibration/docker
bash download_opencv_deps_fast.sh
```

### 3. 预期效果
- **安装 aria2c 后**：2-10MB/s，10-30 秒完成
- **未安装 aria2c（自动降级到 curl）**：1-5MB/s（使用镜像源），30-60 秒完成

## 优化详情

### 优化点 1：国内镜像源
优先使用以下镜像源（自动按顺序尝试）：
1. `https://ghproxy.com/` - 高速镜像
2. `https://mirror.ghproxy.com/` - 备用镜像
3. `https://ghproxy.net/` - 备用镜像

### 优化点 2：aria2c 多线程下载
- 并发连接数：16（`-x 16 -s 16`）
- 分块大小：1MB（`-k 1M`）
- 自动重试机制
- 断点续传支持

### 优化点 3：智能降级
工具优先级：`aria2c` → `curl` → `wget`

自动检测并使用最佳可用工具。

## 文件对比

| 特性 | 原始脚本 | 优化脚本 |
|------|---------|---------|
| 镜像源 | ❌ | ✅ 多镜像源自动切换 |
| 多线程 | ❌ | ✅ aria2c 支持 |
| 重试机制 | 1 次 | ✅ 3 次可配置 |
| 速度 | 37KB/s | 2-10MB/s (提速 50-270x) |
| 工具检测 | 简单 | ✅ 智能降级 |

## 故障排查

### 问题：下载仍然很慢
**可能原因**：镜像源服务器限流或网络问题
**解决方案**：
```bash
# 检查网络连通性
ping ghproxy.com

# 手动测试镜像源速度
curl -I https://ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/1224f78da6684df04397ac0f40c961ed37f79ccb/ippicv/ippicv_2021.8_lnx_intel64_20230330_general.tgz
```

### 问题：aria2c 下载失败
**可能原因**：aria2c 版本过旧或网络配置问题
**解决方案**：脚本会自动降级到 curl，无需干预

### 问题：文件校验失败
**可能原因**：下载文件损坏
**解决方案**：
```bash
# 删除损坏的文件重新下载
rm -f ./deps/opencv_deps/ippicv_*.tgz
bash download_opencv_deps_fast.sh
```

## 验证下载成功

```bash
# 检查文件大小（31MB）
ls -lh ./deps/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz

# 检查文件完整性（SHA256）
sha256sum ./deps/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz
# 预期值：a58d8af05a3d9748e5b4834f2d876b468094042a0f6f8320d9b9a27829849874
```

## 下一步

下载完成后，继续 Docker 构建：

```bash
bash docker_build.sh
```

## 附录：手动下载（备选方案）

如果自动脚本仍然失败，可以手动下载：

### 方法 1：浏览器下载
1. 访问：`https://ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/1224f78da6684df04397ac0f40c961ed37f79ccb/ippicv/ippicv_2021.8_lnx_intel64_20230330_general.tgz`
2. 下载后放到：`./deps/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz`

### 方法 2：使用其他镜像源
```bash
# 使用 jsDelivr 镜像
wget -O ./deps/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz \
  "https://cdn.jsdelivr.net/gh/opencv/opencv_3rdparty@1224f78da6684df04397ac0f40c961ed37f79ccb/ippicv/ippicv_2021.8_lnx_intel64_20230330_general.tgz"
```

## 性能对比

| 场景 | 原始脚本 | 优化脚本（curl） | 优化脚本（aria2c） |
|------|---------|----------------|-------------------|
| 31MB 文件 | ~15 分钟 | ~60 秒 | ~10-20 秒 |
| 单文件速度 | 37KB/s | 500KB-2MB/s | 2-10MB/s |
| 10MB 文件 | ~5 分钟 | ~20 秒 | ~3-5 秒 |
