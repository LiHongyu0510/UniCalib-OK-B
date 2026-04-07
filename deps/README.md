# Docker 依赖源码目录

此目录存放构建 Docker 镜像所需的第三方库源码，避免构建时从网络拉取。

## 目录结构

```
deps/
├── opencv/                  # OpenCV 主源码（版本 4.8.0）
│   ├── opencv/              # 主仓库
│   └── opencv_contrib/      # 扩展模块
├── opencv_deps/             # OpenCV 预下载依赖（大文件）
│   └── ippicv_*.tgz         # IPPICV 性能优化库
├── ceres-solver/           # 非线性优化库（版本 2.1.0）
├── gtsam/                  # 因子图优化库（版本 4.2）
├── Sophus/                 # 李群/李代数库
├── magic_enum/             # 枚举反射库
└── README.md              # 本文件
```

## 各目录说明

### opencv/
**作用**：OpenCV 计算机视觉库源码

**版本**：4.8.0

**使用方式**：
- `COPY ./deps/opencv/opencv ${THIRDPARTY_WS}/opencv`
- `COPY ./deps/opencv/opencv_contrib ${THIRDPARTY_WS}/opencv_contrib`

**为什么本地源码**：
- 网络下载不稳定（GitHub 可能限速）
- 支持离线构建
- 可控版本（避免上游变更）

### opencv_deps/
**作用**：OpenCV 构建时预下载的二进制依赖

**内容**：
- `ippicv_2021.8_lnx_intel64_20230330_general.tgz` (~50MB)
  - Intel Performance Primitives
  - 用于加速 OpenCV 函数
  - 否则每次构建都要从 GitHub 下载

**下载方式**：
```bash
cd docker
./download_opencv_deps.sh
```

**注意**：
- 此目录已添加到 `.gitignore`，不纳入版本控制
- 包含大文件，首次使用需下载

### ceres-solver/
**作用**：非线性最小二乘优化库

**版本**：2.1.0

**用途**：
- 相机标定优化
- SLAM 后端优化
- 传感器标定

### gtsam/
**作用**：Georgia Tech Smoothing and Mapping 库

**版本**：4.2

**用途**：
- iKalibr 项目核心依赖
- 因子图优化
- 传感器融合

### Sophus/
**作用**：李群/李代数库

**用途**：
- 旋转/位姿表示
- 流形优化
- 标定误差最小化

### magic_enum/
**作用**：C++ 枚举反射库

**用途**：
- 运行时枚举转字符串
- 调试和日志输出
- iKalibr 项目依赖

## 添加新依赖

当需要添加新的第三方库时，按以下步骤操作：

### 1. 克隆/下载源码到 deps/ 目录

```bash
cd deps
git clone <repo_url> <library_name>
cd <library_name>
git checkout <version>  # 固定版本
git submodule update --init --recursive  # 如果有子模块
```

### 2. 更新 Dockerfile

```dockerfile
COPY ./deps/<library_name> ${THIRDPARTY_WS}/<library_name>

RUN mkdir -p ${THIRDPARTY_WS}/<library_name>/build && \
    cd ${THIRDPARTY_WS}/<library_name>/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ... && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd ${THIRDPARTY_WS} && rm -rf <library_name>
```

### 3. 更新本 README

记录库的名称、版本、用途。

## 维护原则

### 版本固定
- 每个依赖库必须指定固定版本（git tag 或 commit hash）
- 避免使用 `master`、`main` 等不稳定分支

### 离线优先
- 优先使用本地源码，而非网络下载
- 对于必须下载的文件，提供下载脚本

### 清理构建产物
- Dockerfile 构建后删除源码（减小镜像体积）
- 仅保留编译后的库和头文件

### Git 管理
- 源码目录需要 `.gitignore` 忽略大文件和构建产物
- 仅提交 `README.md` 和空目录标记（`.gitkeep`）

## 故障排查

### 问题：Dockerfile COPY 失败

**原因**：源码目录不存在或路径错误

**解决**：
```bash
# 检查目录结构
ls -la deps/

# 确保 Dockerfile 中的路径匹配
# 示例：COPY ./deps/opencv/opencv ...
#  对应：deps/opencv/opencv/
```

### 问题：构建时仍尝试网络下载

**原因**：CMake 配置未正确指向本地文件

**解决**：
- 检查 CMake 参数是否指定本地路径
- 参考文档：`docs/opencv-build-flow.md`

### 问题：依赖文件缺失

**原因**：预下载脚本未执行或下载失败

**解决**：
```bash
cd docker
./download_opencv_deps.sh
# 检查输出文件
ls -lh ../deps/opencv_deps/
```

## 相关文档

- [OpenCV 构建流程优化](../docs/opencv-build-flow.md)
- [快速修复指南](../QUICK_FIX.md)
- [主 README](../README.md)

## 更新日志

| 日期 | 变更内容 | 作者 |
|------|---------|------|
| 2026-02-28 | 添加 opencv_deps 目录，预下载 IPPICV | Assistant |
| 2026-02-28 | 创建本 README | Assistant |
