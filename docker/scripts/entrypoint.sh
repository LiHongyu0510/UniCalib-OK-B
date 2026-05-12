#!/bin/bash
# ==========================================================================
# 容器启动脚本 (Entrypoint) - 多标定项目环境
# 自动 source ROS2、可选自动编译 ROS2 驱动、支持传入命令
#
# 使用方法:
#   docker run -it ... calib_env:humble [命令]
#
# 环境变量:
#   AUTO_BUILD_LIVOX=0  # 禁用自动编译 livox_ros_driver2
# ==========================================================================

set -e

WORKSPACE="${WORKSPACE:-/root/calib_ws}"

echo "========================================="
echo "Multi-Calibration Dev Environment"
echo "ROS2 Humble + CUDA 11.8 + PyTorch 2.1"
echo "========================================="
echo "[INFO] 工作空间: $WORKSPACE"
echo "[INFO] 用户: $(whoami)"
echo "[INFO] 主机: $(hostname)"
echo ""

# 自动编译 ROS2 驱动（若存在源码且未禁用）
LIVOX_SRC="$WORKSPACE/src/livox_ros_driver2"
if [ -d "$LIVOX_SRC" ]; then
    if [ "${AUTO_BUILD_LIVOX:-1}" != "0" ]; then
        if [ ! -f "$WORKSPACE/install/livox_ros_driver2/share/livox_ros_driver2/package.xml" ]; then
            echo "[INFO] 检测到 livox_ros_driver2 源码，开始自动编译..."
            echo "[INFO] 如需跳过: AUTO_BUILD_LIVOX=0"
            [ -x /opt/scripts/build_ros2_drivers.sh ] && /opt/scripts/build_ros2_drivers.sh || true
        else
            echo "[INFO] livox_ros_driver2 已编译完成"
        fi
    else
        echo "[INFO] 自动编译已禁用 (AUTO_BUILD_LIVOX=0)"
    fi
fi

# 若挂载了 calib_unified 且含预下载的 diffusers wheel，为系统 python3 离线安装（供 DM-Calib 回退到系统 Python 时使用）
WHEELS_DIR="${WORKSPACE}/calib_unified/thirdparty/wheels"
if [ -d "$WHEELS_DIR" ] && [ -n "$(find "$WHEELS_DIR" -maxdepth 1 -name 'diffusers-*.whl' 2>/dev/null)" ]; then
  if ! python3 -c "import diffusers" 2>/dev/null; then
    echo "[INFO] 检测到预下载 diffusers wheel，正在为系统 Python 离线安装（DM-Calib 用）..."
    set +e
    pip3 install --no-index --find-links "$WHEELS_DIR" 'numpy<2' diffusers transformers 2>/dev/null || \
      pip3 install --no-index --find-links "$WHEELS_DIR" diffusers transformers
    if python3 -c "import diffusers" 2>/dev/null; then
      echo "[INFO] diffusers/transformers 已从 $WHEELS_DIR 安装完成"
    else
      echo "[WARN] 从 wheel 安装 diffusers 未成功；若仍报 NumPy 2 错误，请宿主机执行: ./calib_unified_run.sh --download-numpy"
    fi
    set -e
  fi
fi

echo ""
echo "========================================="
echo "环境已就绪"
echo "========================================="

# 自动 source 工作空间
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    echo "[INFO] 自动 source: $WORKSPACE/install/setup.bash"
    source "$WORKSPACE/install/setup.bash"
fi

# 始终 source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

echo ""
echo "[可用命令]"
echo "  /opt/scripts/verify_env.sh          # 验证环境"
echo "  /opt/scripts/run_calib_cpp.sh       # 一键标定（C++ 主框架，与 README 一致）"
echo "  /opt/scripts/build_ros2_drivers.sh  # 手动编译 ROS2 驱动"
echo "  source $WORKSPACE/install/setup.bash"
echo ""
echo "[使用示例]"
echo "  # C++ 主框架一键标定（推荐）"
echo "  /opt/scripts/run_calib_cpp.sh"
echo "  # 或手动: cd \$WORKSPACE/unicalib_C_plus_plus/build && RUN_PIPELINE=1 ./unicalib_example ../config/sensors.yaml \$WORKSPACE/data"
echo "  # Python 参考实现"
echo "  python3 src/UniCalib/scripts/run_calibration.py --config ... --data data/ --output results/"
echo ""

if [ $# -gt 0 ]; then
    echo "[INFO] 执行命令: $*"
    exec "$@"
else
    echo "[INFO] 启动交互式 bash..."
    exec /bin/bash
fi
