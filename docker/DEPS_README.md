# Docker 依赖管理说明

本目录包含 UniCalib 项目所需的外部依赖库，用于构建 Docker 镜像。

## 📁 依赖目录结构

```
deps/
├── cmake/                    # CMake 3.24 (Sophus 等依赖要求)
├── gtsam/                    # GTSAM 4.2 (iKalibr 需要)
├── Sophus/                   # Sophus 李群/李代数库
├── magic_enum/               # magic_enum (iKalibr 需要)
├── ceres-solver/            # Ceres 非线性优化
├── opencv/                   # OpenCV 4.8.0 (带 CUDA 支持)
├── opencv_deps/              # OpenCV 预下载依赖 (IPPICV 等)
├── glm/                      # GLM 0.9.9.8 (MIAS-LCEC 需要，可选)
├── ikd-Tree/                 # ikd-Tree (iKalibr 需要)
├── IKFoM/                    # IKFoM (iKalibr 需要)
├── Open3D/                   # Open3D
├── pmc/                      # PMC
├── TEASER-plusplus/          # TEASER++
├── Livox-SDK2/              # Livox SDK
└── livox_ros_driver2/       # Livox ROS2 驱动
```

## 🔄 依赖策略

### 优先级原则
1. **本地依赖优先**：优先使用 `deps/` 目录中的库
2. **apt 安装回退**：本地缺失时从 apt 仓库安装
3. **避免网络请求**：所有构建时依赖预下载到本地

### 具体实现

#### GLM (OpenGL Mathematics)

**状态**：Header-only 库，推荐本地化

**下载方法**：
```bash
cd docker/deps
bash download_glm.sh
```

**Dockerfile 行为**：
- ✅ 检测到 `deps/glm` → 复制到 `/usr/local/include/glm`
- ❌ 未检测到 → 安装 `libglm-dev` (从 apt)

**为什么需要 GLM**：
- MIAS-LCEC 依赖 CMake 的 `find_package(GLM REQUIRED)`
- GLM 提供 OpenGL 数学函数（向量、矩阵运算）

#### OpenCV 依赖

**预下载文件**：
- `opencv_deps/ippicv_2021.8_lnx_intel64_20230330_general.tgz`
- `opencv_deps/wechat_qrcode_*` 模型文件

**用途**：避免构建时网络请求，提高构建成功率

## 📥 下载依赖

### 完整下载（推荐首次构建前）

```bash
cd docker

# 下载 GLM（可选，但推荐）
cd deps
bash download_glm.sh

# 返回并构建镜像
cd ..
bash docker_build.sh
```

### 单独下载 GLM

```bash
cd docker/deps
bash download_glm.sh
```

## 🔧 手动添加新依赖

如果需要添加新的 header-only 库：

1. 下载到 `deps/` 目录
2. 在 `Dockerfile` 中添加 COPY 指令
3. 复制到 `/usr/local/include/`

示例：
```dockerfile
COPY ./deps/newlib /tmp/newlib_src 2>/dev/null || true
RUN if [ -d "/tmp/newlib_src" ]; then \
      cp -r /tmp/newlib_src/include /usr/local/include/newlib && \
      rm -rf /tmp/newlib_src; \
    fi
```

## ⚠️ 注意事项

1. **网络问题**：如果 GitHub 访问慢，建议提前下载所有依赖
2. **版本锁定**：所有依赖都有固定版本，避免不兼容
3. **清理缓存**：更新依赖后使用 `--no-cache` 重建镜像
   ```bash
   bash docker_build.sh --no-cache
   ```

## 📊 依赖清单

| 依赖 | 版本 | 类型 | 用途 | 本地化 |
|-----|------|------|------|--------|
| CMake | 3.24 | 构建 | 构建工具 | ✅ 必须 |
| GLM | 0.9.9.8 | 数学 | MIAS-LCEC | ⭐ 推荐 |
| GTSAM | 4.2 | 数学 | iKalibr | ✅ 必须 |
| Sophus | latest | 数学 | iKalibr | ✅ 必须 |
| magic_enum | latest | C++ | iKalibr | ✅ 必须 |
| OpenCV | 4.8.0 | 图像 | 多项目 | ✅ 必须 |
| ceres-solver | latest | 优化 | 多项目 | ✅ 必须 |
| ikd-Tree | latest | 数据结构 | iKalibr | ✅ 必须 |
| IKFoM | latest | 优化 | iKalibr | ✅ 必须 |

## 🐛 故障排查

### GLM 相关错误

**错误**：`Could NOT find GLM (missing: GLM_INCLUDE_DIR)`

**解决方案**：
```bash
# 方案 1：下载到本地（推荐）
cd docker/deps && bash download_glm.sh

# 方案 2：重建镜像（会从 apt 安装）
bash docker_build.sh --no-cache
```

### 构建超时

**问题**：网络下载依赖导致构建超时

**解决方案**：
```bash
# 1. 提前下载所有依赖
cd docker/deps
bash download_glm.sh

# 2. 使用本地源构建镜像
cd ..
bash docker_build.sh
```

## 📚 参考资料

- GLM 官方仓库：https://github.com/g-truc/glm
- GLM 文档：https://github.com/g-truc/glm/tree/master/manual
