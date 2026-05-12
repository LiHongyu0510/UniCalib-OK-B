#!/bin/bash
# ==========================================================================
# ROS2 驱动编译脚本 - 多标定项目环境
# 用于编译 livox_ros_driver2 等需在工作空间中编译的 ROS2 包
#
# 使用方法: /opt/scripts/build_ros2_drivers.sh
# 前置: ROS2 Humble 已安装，livox_ros_driver2 源码挂载到 ${WORKSPACE}/src/livox_ros_driver2
# ==========================================================================

set -e

WORKSPACE="${WORKSPACE:-/root/calib_ws}"

echo "========================================="
echo "ROS2 驱动编译脚本"
echo "========================================="
echo "[INFO] 工作空间: $WORKSPACE"

if [ -f "$WORKSPACE/install/livox_ros_driver2/share/livox_ros_driver2/package.xml" ]; then
    echo "[INFO] livox_ros_driver2 已编译，跳过"
    echo "[INFO] 重新编译请: rm -rf $WORKSPACE/build $WORKSPACE/install"
    exit 0
fi

source /opt/ros/humble/setup.bash

LIVOX_SRC="$WORKSPACE/src/livox_ros_driver2"
if [ ! -d "$LIVOX_SRC" ]; then
    echo "[ERROR] 未找到 livox_ros_driver2 源码: $LIVOX_SRC"
    echo "[建议] 挂载: -v /path/to/livox_ros_driver2:$LIVOX_SRC"
    exit 1
fi

echo "[INFO] 编译 livox_ros_driver2..."
cd "$WORKSPACE"
colcon build --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[SUCCESS] 完成。使用: source $WORKSPACE/install/setup.bash"
exit 0
