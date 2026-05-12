# iKalibr 快速开始指南

## 3 步骤完成 iKalibr ROS2 编译

### 步骤 1：准备环境（Docker）

```bash
# 确保在项目根目录
cd /home/wqs/Documents/github/UniCalib

# 检查 Docker 镜像
docker images | grep calib_env

# 如果镜像不存在，先构建
cd docker && bash docker_build.sh --build-only && cd ..
```

### 步骤 2：编译 iKalibr 及其依赖

```bash
# 一键编译（推荐）
./build_and_run.sh --build-external-only

# 或者仅编译 iKalibr
./build_external_tools.sh --tools ikalibr
```

**编译内容**：
1. 自动下载 5 个第三方依赖库源码
2. 编译所有第三方库（约 30-60 分钟）
3. 编译 iKalibr 主程序

### 步骤 3：验证安装

```bash
# 进入 iKalibr 目录
cd iKalibr

# 运行验证脚本
./verify_ikalibr_build.sh

# 查看可执行文件
ls -lh ../install/lib/ikalibr/
```

**预期输出**：
```
[PASS] tiny-viewer: 已安装
[PASS] ctraj: 已安装
[PASS] ufomap: 已安装
[PASS] veta: 已安装
[PASS] opengv: 已安装
[PASS] ikalibr_prog: 可执行
[PASS] ikalibr_learn: 可执行
[PASS] ikalibr_imu_intri_calib: 可执行
```

---

## 使用 iKalibr

### 加载环境

```bash
# 在容器内
source install/setup.bash

# 在宿主机（需要先进入容器）
docker run -it --rm \
    --gpus all \
    -v $(pwd):/root/calib_ws \
    calib_env:humble \
    bash -c "source install/setup.bash && bash"
```

### 运行示例

```bash
# 查看帮助
ikalibr_prog --help

# 运行标定（需要准备数据）
ikalibr_prog --config config/example.yaml --data /path/to/data
```

---

## 故障排查

### 问题 1：编译失败

**解决方案**：
```bash
# 查看详细日志
./build_thirdparty_ros2.sh 2>&1 | tee build.log

# 清理后重新编译
cd iKalibr
rm -rf thirdparty-install/
./build_thirdparty_ros2.sh
```

### 问题 2：找不到库

**解决方案**：
```bash
# 检查环境变量
echo $ctraj_DIR
echo $ufomap_DIR

# 重新设置并编译
cd iKalibr
source set_env_vars.sh  # 需要创建此脚本
cd ..
colcon build --symlink-install
```

### 问题 3：GitHub 访问失败

**解决方案**：
```bash
# 手动下载源码到 thirdparty/ 目录
cd iKalibr/thirdparty
git clone https://github.com/Unsigned-Long/tiny-viewer.git
git clone https://github.com/Unsigned-Long/ctraj.git
git clone --branch devel_surfel https://github.com/Unsigned-Long/ufomap.git
git clone https://github.com/Unsigned-Long/veta.git
git clone https://github.com/laurentkneip/opengv.git

# 然后运行编译脚本（会跳过下载步骤）
cd ..
./build_thirdparty_ros2.sh
```

---

## 下一步

- 📖 阅读完整文档：`IKALIBR_BUILD_GUIDE.md`
- 🔧 准备标定数据：参考 iKalibr 原始文档
- 🧪 测试功能：使用示例数据验证安装

---

**快速参考**：
- 编译脚本：`iKalibr/build_thirdparty_ros2.sh`
- 验证脚本：`iKalibr/verify_ikalibr_build.sh`
- 可执行文件：`install/lib/ikalibr/ikalibr_prog`
- 配置文件：`iKalibr/config/*.yaml`
