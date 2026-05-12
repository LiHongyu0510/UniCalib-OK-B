#!/bin/bash
# ==========================================================================
# 容器启动脚本 (Entrypoint)
# 自动检测并编译必要的 ROS2 驱动
#
# 使用方法:
#   docker run -it ... automap-env:humble [命令]
#
# 环境变量:
#   AUTO_BUILD_LIVOX=0  # 禁用自动编译 livox_ros_driver2
# ==========================================================================

set -e

WORKSPACE="/root/automap_ws"

echo "========================================="
echo "AutoMap-Pro Runtime Environment"
echo "ROS2 Humble + CUDA 11.8 + 全部依赖库"
echo "========================================="
echo "[INFO] 工作空间: $WORKSPACE"
echo "[INFO] 用户: $(whoami)"
echo "[INFO] 主机: $(hostname)"
echo ""

# 自动编译 ROS2 驱动（如果需要且未禁用）
LIVOX_SRC="$WORKSPACE/src/livox_ros_driver2"
if [ -d "$LIVOX_SRC" ]; then
    if [ "${AUTO_BUILD_LIVOX:-1}" != "0" ]; then
        if [ ! -f "$WORKSPACE/install/livox_ros_driver2/share/livox_ros_driver2/package.xml" ]; then
            echo "[INFO] 检测到 livox_ros_driver2 源码，开始自动编译..."
            echo "[INFO] 如需跳过自动编译，设置环境变量: AUTO_BUILD_LIVOX=0"
            echo ""
            /opt/scripts/build_ros2_drivers.sh
        else
            echo "[INFO] livox_ros_driver2 已编译完成"
        fi
    else
        echo "[INFO] 自动编译已禁用 (AUTO_BUILD_LIVOX=0)"
    fi
else
    echo "[INFO] 未检测到 livox_ros_driver2 源码"
    echo "[INFO] 如需使用，请挂载源码到: $LIVOX_SRC"
fi

echo ""
echo "========================================="
echo "环境已就绪"
echo "========================================="

# 如果工作空间已编译，自动 source
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    echo "[INFO] 自动 source 工作空间: $WORKSPACE/install/setup.bash"
    source "$WORKSPACE/install/setup.bash"
fi

echo ""
echo "[可用命令]"
echo "  /opt/scripts/build_ros2_drivers.sh   # 手动编译 ROS2 驱动"
echo "  source /opt/ros/humble/setup.bash     # Source ROS2 环境"
echo "  source /root/automap_ws/install/setup.bash  # Source 工作空间"
echo ""
echo "[使用示例]"
echo "  # 编译并运行 Fast-LIVO2"
echo "  cd /root/automap_ws"
echo "  colcon build --packages-select fast_livo2"
echo "  source install/setup.bash"
echo "  ros2 launch fast_livo2 ..."
echo ""

# 执行传入的命令或启动 bash
if [ $# -gt 0 ]; then
    echo "[INFO] 执行命令: $@"
    exec "$@"
else
    echo "[INFO] 启动交互式 bash..."
    exec "/bin/bash"
fi
