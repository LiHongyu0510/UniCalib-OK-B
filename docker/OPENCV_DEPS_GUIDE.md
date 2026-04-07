# OpenCV 依赖缓存机制

## 目标

- 加速 Docker 镜像构建（减少网络下载时间）
- 支持离线构建
- 自动缓存构建时下载的文件，供下次使用

## 依赖文件清单

OpenCV 4.8.0 构建时需要以下依赖文件：

| 文件名 | 大小 | 来源 | 用途 | 是否必需 |
|--------|------|------|------|----------|
| `ippicv_2021.8_lnx_intel64_20230330_general.tgz` | ~31MB | Intel IPP | 性能优化 | ✅ 必需 |
| `res10_300x300_ssd_iter_140000_fp16.caffemodel` | ~1.8KB | OpenCV DNN | 人脸检测模型 | ⭕ 可选 |
| `detect.caffemodel` | ~943KB | WeChat CV | QR Code 检测模型 | ⭕ 可选 |
| `detect.prototxt` | ~几KB | WeChat CV | QR Code 检测配置 | ⭕ 可选 |
| `sr.caffemodel` | ~943KB | WeChat CV | QR Code 超分辨率模型 | ⭕ 可选 |
| `sr.prototxt` | ~几KB | WeChat CV | QR Code 超分辨率配置 | ⭕ 可选 |

## 工作流程

### 首次构建（无缓存）

```bash
cd docker

# 1. 下载必需的 IPPICV 文件（其他文件可选）
./download_opencv_deps_ultra.sh

# 2. 构建镜像
./docker_build.sh

# 3. 构建完成后，自动提取 OpenCV 下载的文件
./extract_opencv_deps.sh

# 4. 查看提取的文件
ls -lh ../deps/opencv_deps/
```

### 后续构建（有缓存）

```bash
cd docker

# 构建时自动使用本地缓存文件
./docker_build.sh
```

## 脚本说明

### 1. `download_opencv_deps_ultra.sh`

**用途**：预下载 OpenCV 依赖文件到本地缓存目录

**功能**：
- 多镜像源支持（GitHub + 国内镜像）
- 自动重试机制（最多 5 次）
- 文件完整性检查
- 跳过已存在文件

**使用**：
```bash
# 完整模式（检查文件完整性）
./download_opencv_deps_ultra.sh

# 跳过已存在文件（不检查完整性）
./download_opencv_deps_ultra.sh --skip-existing
```

### 2. `docker_build.sh`

**用途**：构建 Docker 镜像并自动提取依赖

**功能**：
- 检查依赖文件
- 构建 Docker 镜像
- 构建完成后自动提取 OpenCV 下载的文件
- 导出镜像归档

**使用**：
```bash
# 完整流程（检查依赖 -> 构建 -> 提取 -> 运行）
./docker_build.sh

# 仅构建，不运行
./docker_build.sh --build-only

# 仅运行（已有镜像时）
./docker_build.sh --run-only
```

### 3. `extract_opencv_deps.sh`

**用途**：从已构建的 Docker 镜像中提取 OpenCV 依赖

**功能**：
- 从镜像的 `/root/thirdparty/opencv_deps` 目录提取文件
- 保存到 `../deps/opencv_deps/`
- 跳过已存在且大小相同的文件
- 显示提取进度和文件列表

**使用**：
```bash
# 从默认镜像提取
./extract_opencv_deps.sh

# 从指定镜像提取
./extract_opencv_deps.sh calib_env:humble
```

## Dockerfile 配置

### 关键配置

```dockerfile
# 拷贝本地依赖到镜像
COPY ./deps/opencv_deps ${THIRDPARTY_WS}/opencv_deps

# CMake 配置使用本地文件
-DOPENCV_IPPICV_URL="file://${THIRDPARTY_WS}/opencv_deps/${IPPICV_FILENAME}" \
-DOPENCV_DOWNLOAD_PATH="${THIRDPARTY_WS}/opencv_deps" \
```

### 依赖保留

Dockerfile 在构建后**保留** `opencv_deps` 目录，这样可以：
- 构建完成后使用 `extract_opencv_deps.sh` 提取文件
- 下次构建时直接使用缓存文件

**注意**：保留该目录会增加镜像体积（约 32MB）。如果需要减小镜像体积：
1. 使用 `extract_opencv_deps.sh` 提取文件到本地
2. 修改 Dockerfile 添加 `RUN rm -rf ${THIRDPARTY_WS}/opencv_deps`
3. 重新构建镜像

## 手动下载失败时的解决方案

### 方案 1：使用代理

```bash
# 设置代理
export http_proxy=http://proxy.example.com:8080
export https_proxy=http://proxy.example.com:8080

# 运行下载脚本
./download_opencv_deps_ultra.sh
```

### 方案 2：手动下载并拷贝

如果自动下载失败，可以手动下载文件：

1. **IPPICV**:
   - URL: `https://raw.githubusercontent.com/opencv/opencv_3rdparty/1224f78da6684df04397ac0f40c961ed37f79ccb/ippicv/ippicv_2021.8_lnx_intel64_20230330_general.tgz`
   - 拷贝到: `deps/opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz`

2. **WeChat QR Code 模型**:
   - 访问: `https://github.com/WeChatCV/opencv_3rdparty/tree/a8b69ccc738421293254aec5ddb38bd523503252`
   - 下载: `detect.caffemodel`, `detect.prototxt`, `sr.caffemodel`, `sr.prototxt`
   - 拷贝到: `deps/opencv_deps/`

3. **验证文件大小**:
   ```bash
   ls -lh deps/opencv_deps/
   # IPPICV 应该约 31MB
   ```

### 方案 3：让 Docker 构建时下载

如果本地没有缓存，OpenCV 构建时会自动下载文件：
- 构建完成后使用 `extract_opencv_deps.sh` 提取
- 下次构建时自动使用缓存

## 故障排查

### 问题 1：文件下载失败

**症状**：`download_opencv_deps_ultra.sh` 下载失败

**解决**：
1. 检查网络连接
2. 使用 `--skip-existing` 跳过已下载的文件
3. 手动下载失败文件
4. 让 Docker 构建时下载

### 问题 2：构建时仍然下载文件

**症状**：`docker build` 日志显示正在下载依赖文件

**解决**：
1. 确认 `deps/opencv_deps/` 目录中有正确的文件
2. 检查文件名是否匹配（注意大小写）
3. 验证文件完整性（IPPICV 应该约 31MB）

### 问题 3：提取脚本找不到文件

**症状**：`extract_opencv_deps.sh` 报告 "未找到 opencv_deps 目录"

**解决**：
1. 确认 Docker 构建成功完成
2. 检查 Dockerfile 是否删除了 `opencv_deps` 目录
3. 如果已删除，修改 Dockerfile 保留该目录，重新构建

### 问题 4：镜像体积过大

**症状**：镜像体积比预期大

**原因**：Dockerfile 保留了 `opencv_deps` 目录

**解决**：
```dockerfile
# 在 Dockerfile 最后添加
RUN rm -rf ${THIRDPARTY_WS}/opencv_deps
```

然后重新构建镜像。

## 最佳实践

1. **首次构建**：
   - 运行 `download_opencv_deps_ultra.sh` 下载必需文件
   - 运行 `docker_build.sh` 构建
   - 运行 `extract_opencv_deps.sh` 提取所有文件

2. **CI/CD 环境**：
   - 将 `deps/opencv_deps/` 目录纳入版本控制（或使用 Git LFS）
   - 构建时直接使用缓存，避免网络下载

3. **团队协作**：
   - 团队成员共享 `deps/opencv_deps/` 目录
   - 确保所有文件大小正确

4. **定期更新**：
   - 检查是否有新的依赖文件
   - 定期重新提取缓存

## 总结

该依赖缓存机制实现了：
- ✅ 构建加速（减少网络下载时间）
- ✅ 离线构建支持
- ✅ 自动缓存管理
- ✅ 多下载源支持
- ✅ 完整性验证

通过合理使用这些脚本，可以显著提升 Docker 镜像构建效率和稳定性。
