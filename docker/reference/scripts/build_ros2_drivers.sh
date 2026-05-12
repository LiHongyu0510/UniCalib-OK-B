#!/bin/bash
# ==========================================================================
# ROS2 驱动编译脚本
# 用于编译 livox_ros_driver2 等必须在工作空间中编译的ROS2包
#
# 使用方法:
#   /opt/scripts/build_ros2_drivers.sh
#
# 前置条件:
#   - ROS2 Humble 已安装并 source
#   - livox_ros_driver2 源码已挂载到 /root/automap_ws/src/livox_ros_driver2
#   - Livox-SDK2 已安装到 /usr/local
# ==========================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="/root/automap_ws"

echo "========================================="
echo "ROS2 驱动编译脚本"
echo "========================================="
echo "[INFO] 工作空间: $WORKSPACE"
echo "[INFO] 脚本目录: $SCRIPT_DIR"
echo ""

# 检查是否已编译
if [ -f "$WORKSPACE/install/livox_ros_driver2/share/livox_ros_driver2/package.xml" ]; then
    echo "[INFO] livox_ros_driver2 已编译，跳过"
    echo "[INFO] 如需重新编译，请删除: rm -rf $WORKSPACE/build $WORKSPACE/install"
    exit 0
fi

# Source ROS2 环境
echo "[INFO] Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# 检查源码是否已挂载到工作空间
LIVOX_SRC="$WORKSPACE/src/livox_ros_driver2"
if [ ! -d "$LIVOX_SRC" ]; then
    echo "[ERROR] livox_ros_driver2 源码未挂载到工作空间"
    echo "[ERROR] 期望路径: $LIVOX_SRC"
    echo ""
    echo "[建议] 请使用 -v 将源码挂载到工作空间:"
    echo "[示例] docker run -v /path/to/livox_ros_driver2:$LIVOX_SRC ..."
    exit 1
fi

echo "[INFO] 检测到 livox_ros_driver2 源码: $LIVOX_SRC"

# 检查 Livox-SDK2 是否已安装
LIVOX_SDK_LIB="/usr/local/lib/liblivox_lidar_sdk_shared.so"
if [ ! -f "$LIVOX_SDK_LIB" ]; then
    echo "[WARNING] Livox-SDK2 库未找到: $LIVOX_SDK_LIB"
    echo "[WARNING] 编译可能会失败，请确保 Livox-SDK2 已正确安装"
fi

# 编译
echo ""
echo "[INFO] 开始编译 livox_ros_driver2..."
echo "[INFO] 编译命令: colcon build --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo ""

cd "$WORKSPACE"
colcon build --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "[SUCCESS] 编译完成！"
    echo "[INFO] 安装路径: $WORKSPACE/install/livox_ros_driver2"
    echo ""
    echo "[下一步] 运行以下命令使用驱动:"
    echo "  source $WORKSPACE/install/setup.bash"
    echo "  ros2 launch livox_ros_driver2 rviz_HAP_launch.py"
else
    echo ""
    echo "[ERROR] 编译失败！"
    exit 1
fi
