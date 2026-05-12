#!/bin/bash
# =============================================================================
# UniCalib 统一标定系统 — 一键编译脚本
# 支持 Docker 和 本地 Ubuntu 22.04 环境
# =============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 项目根目录的 logs（编译/运行日志统一写入此处并带时间戳）
REPO_LOGS_DIR="${REPO_LOGS_DIR:-$(cd "$SCRIPT_DIR/.." 2>/dev/null && pwd)/logs}"
mkdir -p "$REPO_LOGS_DIR"
BUILD_DIR="${SCRIPT_DIR}/build"
INSTALL_DIR="${SCRIPT_DIR}/install"
BUILD_LOG_FILE="${REPO_LOGS_DIR}/build_$(date +%Y%m%d_%H%M%S).log"

# 颜色输出
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
BUILD_TYPE="Release"
JOBS=$(nproc)
CLEAN=0
WITH_ROS2=0
WITH_PCL_VIZ=1
VERBOSE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --debug)       BUILD_TYPE="Debug" ;;
        --clean)       CLEAN=1 ;;
        --ros2)        WITH_ROS2=1 ;;
        --no-pcl-viz)  WITH_PCL_VIZ=0 ;;
        --verbose)     VERBOSE=1 ;;
        -j*)           JOBS="${1#-j}" ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo "  --debug        Debug build"
            echo "  --clean        Clean build directory"
            echo "  --ros2         Enable ROS2 bag reading"
            echo "  --no-pcl-viz   Disable PCL visualizer"
            echo "  --verbose      Verbose cmake output"
            echo "  -j<N>          Parallel jobs (default: $(nproc))"
            exit 0 ;;
        *) warn "Unknown option: $1" ;;
    esac
    shift
done

# ---------------------------------------------------------------------------
# 环境检测
# ---------------------------------------------------------------------------
info "========== UniCalib 编译环境检测 =========="

# 检测 CMake
CMAKE_VER=$(cmake --version 2>/dev/null | head -1 | awk '{print $3}') || error "CMake 未找到"
info "CMake: $CMAKE_VER"

# 检测编译器
CXX_VER=$(${CXX:-g++} --version 2>/dev/null | head -1) || error "C++ 编译器未找到"
info "编译器: $CXX_VER"

# 检测关键依赖
check_dep() {
    if pkg-config --exists "$1" 2>/dev/null; then
        info "$1: $(pkg-config --modversion $1)"
    elif cmake --find-package -DNAME="$1" -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>/dev/null | grep -q FOUND; then
        info "$1: found"
    else
        warn "$1: 未通过 pkg-config 找到 (可能已通过其他方式安装)"
    fi
}

# 检测是否在 Docker/ROS 环境
if [ -f /.dockerenv ] || [ -n "$ROS_DISTRO" ]; then
    info "环境: Docker/ROS ($ROS_DISTRO)"
    # 在 ROS 环境中, source setup 以确保找到所有依赖
    if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
        source /opt/ros/${ROS_DISTRO}/setup.bash
        info "已 source ROS: /opt/ros/${ROS_DISTRO}/setup.bash"
    fi
fi

info "=========================================="

# ---------------------------------------------------------------------------
# 准备构建目录
# ---------------------------------------------------------------------------
if [ "$CLEAN" -eq 1 ]; then
    info "清理构建目录..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR" "$INSTALL_DIR"

# ---------------------------------------------------------------------------
# 先编译 ctraj 依赖
# ---------------------------------------------------------------------------
CTRAJ_SOURCE="${SCRIPT_DIR}/thirdparty/ctraj_full"
CTRAJ_BUILD="${BUILD_DIR}/ctraj_build"
CTRAJ_INSTALL="${SCRIPT_DIR}/thirdparty/ctraj_install"

if [ -f "${CTRAJ_SOURCE}/CMakeLists.txt" ] && [ ! -d "${CTRAJ_INSTALL}" ]; then
    info "编译 ctraj ..."
    mkdir -p "$CTRAJ_BUILD"
    
    cmake -S "$CTRAJ_SOURCE" -B "$CTRAJ_BUILD" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CTRAJ_INSTALL" \
        -DCMAKE_CXX_STANDARD=17 \
        -DSophus_DIR="${SCRIPT_DIR}/thirdparty/Sophus" \
        2>&1 | grep -v "^--" | head -20
    
    cmake --build "$CTRAJ_BUILD" -j"$JOBS" 2>&1 | tail -5
    cmake --install "$CTRAJ_BUILD" 2>&1 | tail -3
    info "ctraj 编译安装完成: $CTRAJ_INSTALL"
fi

# ---------------------------------------------------------------------------
# CMake 配置
# ---------------------------------------------------------------------------
info "配置 UniCalib CMake ..."

CMAKE_ARGS=(
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    -DUNICALIB_WITH_ROS2="$WITH_ROS2"
    -DUNICALIB_WITH_PCL_VIZ="$WITH_PCL_VIZ"
    -DUNICALIB_BUILD_APPS=ON
)

# 添加 ctraj 路径
if [ -d "$CTRAJ_INSTALL" ]; then
    CMAKE_ARGS+=(-Dctraj_DIR="${CTRAJ_INSTALL}/lib/cmake/ctraj")
fi

# 添加 iKalibr 预编译依赖路径
IKALIBR_TP="${SCRIPT_DIR}/../iKalibr/thirdparty-install"
if [ -d "${IKALIBR_TP}/opengv-install" ]; then
    CMAKE_ARGS+=(-Dopengv_DIR="${IKALIBR_TP}/opengv-install/lib/cmake/opengv-1.0")
fi
if [ -d "${IKALIBR_TP}/ufomap-install" ]; then
    CMAKE_ARGS+=(-Dufomap_DIR="${IKALIBR_TP}/ufomap-install/lib/cmake/ufomap")
fi

if [ "$VERBOSE" -eq 1 ]; then
    CMAKE_ARGS+=(--log-level=VERBOSE)
fi

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" "${CMAKE_ARGS[@]}"

# ---------------------------------------------------------------------------
# 编译（同时写入项目 logs/ 下带时间戳的编译日志）
# ---------------------------------------------------------------------------
info "开始编译 (jobs=$JOBS) ..."
info "编译日志: ${BUILD_LOG_FILE}"

if [ "$VERBOSE" -eq 1 ]; then
    cmake --build "$BUILD_DIR" -j"$JOBS" --verbose 2>&1 | tee "$BUILD_LOG_FILE"
else
    { cmake --build "$BUILD_DIR" -j"$JOBS" 2>&1 | tee "${BUILD_DIR}/build.log" | tee "$BUILD_LOG_FILE"; } | \
        grep -E "error:|warning:|undefined|^\[" | head -50
fi

# ---------------------------------------------------------------------------
# 检查编译结果
# ---------------------------------------------------------------------------
info "========== 编译结果 =========="

BINS=(
    "unicalib_imu_intrinsic"
    "unicalib_camera_intrinsic"
    "unicalib_imu_lidar"
    "unicalib_lidar_camera"
    "unicalib_cam_cam"
    "unicalib_joint"
)

ALL_OK=1
for bin in "${BINS[@]}"; do
    BIN_PATH="${BUILD_DIR}/bin/${bin}"
    if [ -f "$BIN_PATH" ]; then
        SIZE=$(du -sh "$BIN_PATH" | cut -f1)
        info "  ✓ ${bin} (${SIZE})"
    else
        warn "  ✗ ${bin} — 未生成"
        ALL_OK=0
    fi
done

LIB_PATH="${BUILD_DIR}/lib/libunicalib.so"
if [ -f "$LIB_PATH" ]; then
    SIZE=$(du -sh "$LIB_PATH" | cut -f1)
    info "  ✓ libunicalib.so (${SIZE})"
else
    warn "  ✗ libunicalib.so — 未生成"
    ALL_OK=0
fi

echo ""
if [ "$ALL_OK" -eq 1 ]; then
    info "========== 编译成功 =========="
    echo ""
    echo "可执行文件位于: ${BUILD_DIR}/bin/"
    echo "编译完整日志: ${BUILD_LOG_FILE}"
    echo ""
    echo "快速测试:"
    echo "  ${BUILD_DIR}/bin/unicalib_imu_intrinsic --help"
    echo "  ${BUILD_DIR}/bin/unicalib_camera_intrinsic --help"
    echo "  ${BUILD_DIR}/bin/unicalib_joint --help"
    echo ""
else
    warn "========== 部分编译失败, 请检查错误日志 =========="
    echo "日志文件: ${BUILD_DIR}/build.log"
    echo "项目日志: ${BUILD_LOG_FILE}"
fi

# ---------------------------------------------------------------------------
# 生成符号链接到 bin/
# ---------------------------------------------------------------------------
mkdir -p "${SCRIPT_DIR}/bin"
for bin in "${BINS[@]}"; do
    BIN_PATH="${BUILD_DIR}/bin/${bin}"
    if [ -f "$BIN_PATH" ]; then
        cp "$BIN_PATH" "${SCRIPT_DIR}/bin/${bin}"
    fi
done
info "可执行文件已复制到: ${SCRIPT_DIR}/bin/"
