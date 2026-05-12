#!/bin/bash
# =============================================================================
# rebuild_ikalibr.sh - 在 Docker 容器内重新编译 iKalibr (ROS2 Humble)
#
# 使用方法:
#   ./rebuild_ikalibr.sh              # 清理并重建整个工作空间
#   ./rebuild_ikalibr.sh --incremental # 增量编译（保留已编译部分）
#   ./rebuild_ikalibr.sh --ikalibr-only # 仅重编译 ikalibr 包
#
# 要求:
#   - Docker 已安装
#   - calib_env:humble 镜像已构建 (make build)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
DOCKER_IMAGE="calib_env:humble"
WORKSPACE="/root/calib_ws"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }
section() { echo -e "${BLUE}========================================${NC}"; echo -e "${BLUE}$1${NC}"; echo -e "${BLUE}========================================${NC}"; }

# 解析参数
MODE="full"
for arg in "$@"; do
    [ "$arg" = "--incremental" ] && MODE="incremental"
    [ "$arg" = "--ikalibr-only" ] && MODE="ikalibr-only"
done

section "UniCalib - iKalibr ROS2 重编译脚本"
info "编译模式: $MODE"
info "项目根目录: $PROJECT_ROOT"

# 检查 Docker 镜像
if ! docker image inspect "$DOCKER_IMAGE" &>/dev/null; then
    error "Docker 镜像 $DOCKER_IMAGE 不存在，请先运行: make build"
    exit 1
fi

# ─── 构建 iKalibr 第三方库 ──────────────────────────────────────────────────
# 依赖替换: veta→aslam_cv2, ctraj→basalt-headers, ufomap→devel_surfel
BUILD_THIRDPARTY_CMD=""
if [ "$MODE" = "full" ] || [ "$MODE" = "ikalibr-only" ]; then
    BUILD_THIRDPARTY_CMD="
echo '=== Step 1: 构建 iKalibr 第三方依赖 (aslam_cv2 / basalt-headers / ufomap-devel_surfel / opengv) ==='
cd ${WORKSPACE}/src/iKalibr
if [ -f build_thirdparty_ros2.sh ]; then
    chmod +x build_thirdparty_ros2.sh
    J=${NPROC:-$(nproc)} bash build_thirdparty_ros2.sh
else
    echo '[WARN] build_thirdparty_ros2.sh 不存在，跳过第三方库构建'
fi
cd ${WORKSPACE}
"
fi

# ─── colcon 编译命令 ─────────────────────────────────────────────────────────
# 向 cmake 传递第三方库路径（允许 basalt/aslam 路径可配置）
IKALIBR_SRC="${WORKSPACE}/src/iKalibr"
COLCON_CMAKE_ARGS=(
    "--cmake-args"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DUSE_THIRDPARTY_LIBS=ON"
    # basalt-headers: iKalibr 内置于 thirdparty/basalt-headers
    "-DBASALT_HEADERS_DIR=${IKALIBR_SRC}/thirdparty/basalt-headers/include"
    # aslam_cv2: iKalibr 内置于 thirdparty/aslam_cv2
    "-DASLAM_CV2_DIR=${IKALIBR_SRC}/thirdparty/aslam_cv2"
)

COLCON_FLAGS="--symlink-install"
if [ "$MODE" = "ikalibr-only" ]; then
    COLCON_CMD="colcon build ${COLCON_FLAGS} --packages-select ikalibr ${COLCON_CMAKE_ARGS[*]} 2>&1"
else
    COLCON_CMD="colcon build ${COLCON_FLAGS} ${COLCON_CMAKE_ARGS[*]} 2>&1"
fi

CLEAN_CMD=""
if [ "$MODE" = "full" ]; then
    CLEAN_CMD="
echo '=== Step 0: 清理编译缓存 ==='
cd ${WORKSPACE}
rm -rf build/ikalibr install/ikalibr log/latest_build
echo '清理完成'
"
fi

# ─── Docker 运行 ─────────────────────────────────────────────────────────────
docker run --rm \
    --network host \
    -v "${PROJECT_ROOT}/iKalibr:${WORKSPACE}/src/iKalibr:rw" \
    -v "${PROJECT_ROOT}/unicalib_C_plus_plus:${WORKSPACE}/unicalib_C_plus_plus:rw" \
    -v "${PROJECT_ROOT}/UniCalib:${WORKSPACE}/src/UniCalib:rw" \
    -v "${PROJECT_ROOT}/click_calib:${WORKSPACE}/src/click_calib:rw" \
    -v "${PROJECT_ROOT}/MIAS-LCEC:${WORKSPACE}/src/MIAS-LCEC:rw" \
    -v "${PROJECT_ROOT}/learn-to-calibrate:${WORKSPACE}/src/learn-to-calibrate:rw" \
    -v "${PROJECT_ROOT}/DM-Calib:${WORKSPACE}/src/DM-Calib:rw" \
    -v "${PROJECT_ROOT}/Transformer-IMU-Calibrator:${WORKSPACE}/src/Transformer-IMU-Calibrator:rw" \
    "$DOCKER_IMAGE" \
    bash -c "
set -e
source /opt/ros/humble/setup.bash
export WORKSPACE=${WORKSPACE}

${CLEAN_CMD}
${BUILD_THIRDPARTY_CMD}

echo '=== Step 2: colcon 编译 iKalibr ==='
cd ${WORKSPACE}

if [ ! -f src/iKalibr/package.xml ]; then
    echo '[ERROR] src/iKalibr/package.xml 不存在，挂载可能失败'
    exit 1
fi

echo '[INFO] 开始 colcon 构建 (veta→aslam_cv2, ctraj→basalt-headers)...'
${COLCON_CMD} || {
    echo '=== 编译失败，显示最后错误日志 ==='
    cat log/latest_build/ikalibr/stderr.log 2>/dev/null | tail -100 || true
    exit 1
}

echo '=== 编译成功! ==='
echo '[INFO] 可执行文件位于: ${WORKSPACE}/install/ikalibr/lib/ikalibr/'
ls install/ikalibr/lib/ikalibr/ 2>/dev/null || true
"

info "完成! 使用以下命令进入容器运行标定:"
info "  docker run --rm -it --network host \\"
info "    -v ${PROJECT_ROOT}/iKalibr:${WORKSPACE}/src/iKalibr:rw \\"
info "    $DOCKER_IMAGE bash"
