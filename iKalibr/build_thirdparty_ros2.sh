#!/bin/bash
# =============================================================================
# build_thirdparty_ros2.sh — 编译 iKalibr 第三方依赖库 (ROS2 重构版)
#
# 依赖替换策略:
#   veta   → aslam_cv2      (本地 thirdparty/aslam_cv2，aslam_cv2-backed distortion)
#   ctraj  → basalt-headers  (本地 thirdparty/basalt-headers，已内置)
#   ufomap → ufomap-devel_surfel (含 surfel/octree API)
#   opengv                   (保持，已存在)
#
# 用法:
#   cd iKalibr
#   chmod +x build_thirdparty_ros2.sh
#   ./build_thirdparty_ros2.sh
#
# 注意: 此脚本适配 ROS2 Humble 环境 (calib_env:humble Docker 镜像)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
THIRDPARTY_DIR="${PROJECT_ROOT}/thirdparty"
INSTALL_DIR="${PROJECT_ROOT}/thirdparty-install"
NPROC="${J:-$(nproc)}"

# ─── 颜色 ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
success() { echo -e "${GREEN}[PASS]${NC} $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error()   { echo -e "${RED}[FAIL]${NC} $1"; exit 1; }
info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
section() { echo -e "\n${BLUE}>>> $1${NC}"; }

# =============================================================================
# 0. 环境检查
# =============================================================================
check_env() {
    section "环境检查"
    command -v cmake  &>/dev/null || error "cmake 未安装"
    command -v g++    &>/dev/null || error "g++ 未安装"
    command -v git    &>/dev/null || error "git 未安装"
    info "cmake: $(cmake --version | head -1)"
    info "g++  : $(g++ --version | head -1)"
    # ROS2
    if [[ -z "${ROS_DISTRO:-}" ]] && [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    fi
    [[ -n "${ROS_DISTRO:-}" ]] && info "ROS  : ${ROS_DISTRO}" || warning "ROS2 未检测到"
    mkdir -p "${INSTALL_DIR}"/{include,lib/{cmake}}
    success "环境检查通过"
}

# =============================================================================
# 1. basalt-headers — header-only，已内置于 thirdparty/basalt-headers
# =============================================================================
ensure_basalt_headers() {
    section "basalt-headers (替代 ctraj)"
    local BASALT_INC="${THIRDPARTY_DIR}/basalt-headers/include"
    if [[ -f "${BASALT_INC}/basalt/spline/se3_spline.h" ]]; then
        success "basalt-headers 已就绪: ${BASALT_INC}"
        return 0
    fi
    # 尝试从 calib_unified 同级目录复制
    local CALIB_UNIFIED_BASALT="${PROJECT_ROOT}/../calib_unified/thirdparty/basalt-headers/include/basalt"
    if [[ -d "${CALIB_UNIFIED_BASALT}" ]]; then
        info "从 calib_unified 复制 basalt-headers..."
        mkdir -p "${THIRDPARTY_DIR}/basalt-headers/include"
        cp -r "${CALIB_UNIFIED_BASALT}" "${THIRDPARTY_DIR}/basalt-headers/include/"
        success "basalt-headers 复制完成"
    else
        error "basalt-headers 未找到! 请确保 thirdparty/basalt-headers/include/basalt/ 存在"
    fi
}

# =============================================================================
# 2. aslam_cv2 — 替代 veta；提供相机模型与畸变计算
#    源码来自 thirdparty/aslam_cv2（已从 calib_unified 复制）
# =============================================================================
build_aslam_cv2() {
    section "aslam_cv2 (替代 veta)"
    local ASLAM_SRC="${THIRDPARTY_DIR}/aslam_cv2"
    local ASLAM_BLD="${THIRDPARTY_DIR}/aslam_cv2-build"
    local ASLAM_INS="${INSTALL_DIR}/aslam_cv2-install"

    if [[ -f "${ASLAM_INS}/lib/libaslam_cv_cameras.a" ]]; then
        info "aslam_cv2 已安装，跳过"
        return 0
    fi

    # 检查源码
    if [[ ! -f "${ASLAM_SRC}/CMakeLists.txt" ]]; then
        # 尝试从 calib_unified 复制
        local SRC="${PROJECT_ROOT}/../calib_unified/thirdparty/aslam_cv2"
        if [[ -d "${SRC}" ]]; then
            info "从 calib_unified 复制 aslam_cv2..."
            cp -r "${SRC}" "${THIRDPARTY_DIR}/"
        else
            error "aslam_cv2 源码未找到: ${ASLAM_SRC}"
        fi
    fi

    info "编译 aslam_cv2 (glog=${ASLAM_BLD})..."
    mkdir -p "${ASLAM_BLD}"
    cmake "${ASLAM_SRC}" \
        -B "${ASLAM_BLD}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${ASLAM_INS}" \
        2>&1 | tail -5
    cmake --build "${ASLAM_BLD}" -- -j"${NPROC}" 2>&1 | tail -5
    # 手动安装 (standalone CMakeLists 无 install target — 仅静态库)
    mkdir -p "${ASLAM_INS}/lib" "${ASLAM_INS}/include"
    find "${ASLAM_BLD}" -name "libaslam_cv_cameras.a" -exec cp {} "${ASLAM_INS}/lib/" \;
    cp -r "${ASLAM_SRC}/aslam_cv_cameras/include/." "${ASLAM_INS}/include/"
    cp -r "${ASLAM_SRC}/aslam_cv_common/include/."  "${ASLAM_INS}/include/"
    success "aslam_cv2 编译完成: ${ASLAM_INS}"
}

# =============================================================================
# 3. ufomap-devel_surfel — 含 surfel/octree API (替代旧 ufomap)
# =============================================================================
build_ufomap() {
    section "ufomap-devel_surfel"
    local UFOMAP_INS="${INSTALL_DIR}/ufomap-install"

    if [[ -f "${UFOMAP_INS}/lib/cmake/ufomap/ufomapConfig.cmake" ]]; then
        info "ufomap 已安装，跳过"
        return 0
    fi

    # 源码位置搜索顺序
    local UFOMAP_SRC=""
    for candidate in \
        "${THIRDPARTY_DIR}/ufomap-devel_surfel/ufomap-devel_surfel/ufomap" \
        "${THIRDPARTY_DIR}/ufomap-devel_surfel/ufomap" \
        "${THIRDPARTY_DIR}/ufomap/ufomap" \
        "${THIRDPARTY_DIR}/ufomap" \
        "${PROJECT_ROOT}/../calib_unified/thirdparty/ufomap-devel_surfel/ufomap-devel_surfel/ufomap" \
        "${PROJECT_ROOT}/../calib_unified/thirdparty/ufomap-devel_surfel/ufomap-devel_surfel"; do
        if [[ -f "${candidate}/CMakeLists.txt" ]]; then
            UFOMAP_SRC="${candidate}"
            break
        fi
    done

    if [[ -z "${UFOMAP_SRC}" ]]; then
        # 尝试克隆 devel_surfel 分支
        warning "ufomap 源码未找到，尝试克隆 devel_surfel 分支..."
        local CLONE_DIR="${THIRDPARTY_DIR}/ufomap-devel_surfel"
        if [[ ! -d "${CLONE_DIR}" ]]; then
            git clone --branch devel_surfel --single-branch --depth 1 \
                https://github.com/Unsigned-Long/ufomap.git "${CLONE_DIR}" || \
            error "克隆 ufomap 失败，请手动放置源码至 thirdparty/ufomap-devel_surfel/"
        fi
        for candidate in "${CLONE_DIR}/ufomap" "${CLONE_DIR}"; do
            [[ -f "${candidate}/CMakeLists.txt" ]] && UFOMAP_SRC="${candidate}" && break
        done
        [[ -z "${UFOMAP_SRC}" ]] && error "ufomap CMakeLists.txt 未找到"
    fi

    info "编译 ufomap (源码: ${UFOMAP_SRC})..."
    local UFOMAP_BLD="${THIRDPARTY_DIR}/ufomap-build"
    mkdir -p "${UFOMAP_BLD}"
    cmake "${UFOMAP_SRC}" \
        -B "${UFOMAP_BLD}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${UFOMAP_INS}" \
        -DBUILD_SHARED_LIBS=ON \
        -DUFOMAP_BUILD_EXAMPLES=OFF \
        -DUFOMAP_BUILD_TESTS=OFF \
        2>&1 | tail -5
    cmake --build "${UFOMAP_BLD}" -- -j"${NPROC}" 2>&1 | tail -5
    cmake --install "${UFOMAP_BLD}"
    success "ufomap 编译完成: ${UFOMAP_INS}"
}

# =============================================================================
# 4. opengv — 几何视觉库（已存在则跳过）
# =============================================================================
build_opengv() {
    section "opengv"
    local OPENGV_INS="${INSTALL_DIR}/opengv-install"

    if [[ -f "${OPENGV_INS}/lib/cmake/opengv-1.0/opengv-config.cmake" ]] || \
       ls "${OPENGV_INS}/lib/cmake/opengv-1.0/"*.cmake &>/dev/null 2>&1; then
        info "opengv 已安装，跳过"
        return 0
    fi

    local OPENGV_SRC="${THIRDPARTY_DIR}/opengv"
    if [[ ! -f "${OPENGV_SRC}/CMakeLists.txt" ]]; then
        warning "opengv 源码未找到，尝试克隆..."
        git clone --recursive --depth 1 \
            https://github.com/laurentkneip/opengv.git "${OPENGV_SRC}" || \
        error "克隆 opengv 失败"
    fi

    info "编译 opengv..."
    local OPENGV_BLD="${THIRDPARTY_DIR}/opengv-build"
    mkdir -p "${OPENGV_BLD}"
    cmake "${OPENGV_SRC}" \
        -B "${OPENGV_BLD}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${OPENGV_INS}" \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_TESTS=OFF \
        2>&1 | tail -5
    cmake --build "${OPENGV_BLD}" -- -j"${NPROC}" 2>&1 | tail -5
    cmake --install "${OPENGV_BLD}"
    success "opengv 编译完成: ${OPENGV_INS}"
}

# =============================================================================
# 5. veta-stub — ns_veta API 兼容层（header-only INTERFACE 库，由 CMakeLists.txt 集成）
#    无需单独编译，CMakeLists.txt add_subdirectory 直接使用
# =============================================================================
check_veta_stub() {
    section "veta-stub (aslam_cv2 math backend)"
    local STUB="${THIRDPARTY_DIR}/veta-stub/include/veta/camera/pinhole.h"
    [[ -f "${STUB}" ]] && success "veta-stub 头文件就绪" || error "veta-stub 未找到: ${STUB}"
}

# =============================================================================
# 主流程
# =============================================================================
main() {
    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║  iKalibr 第三方依赖编译 (veta→aslam_cv2, ctraj→basalt) ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo ""
    info "安装目录: ${INSTALL_DIR}"
    info "并行编译: ${NPROC}"
    echo ""

    check_env
    ensure_basalt_headers
    check_veta_stub
    build_aslam_cv2
    build_ufomap
    build_opengv

    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║  第三方库编译完成                                      ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo ""
    info "已安装:"
    [[ -f "${THIRDPARTY_DIR}/basalt-headers/include/basalt/spline/se3_spline.h" ]] && \
        success "  basalt-headers  → thirdparty/basalt-headers  (ctraj 替代)"
    [[ -f "${INSTALL_DIR}/aslam_cv2-install/lib/libaslam_cv_cameras.a" ]] && \
        success "  aslam_cv2       → thirdparty-install/aslam_cv2-install  (veta 替代)"
    [[ -f "${INSTALL_DIR}/ufomap-install/lib/cmake/ufomap/ufomapConfig.cmake" ]] && \
        success "  ufomap-devel_surfel → thirdparty-install/ufomap-install"
    ls "${INSTALL_DIR}/opengv-install/lib/cmake/opengv-1.0/"*.cmake &>/dev/null 2>&1 && \
        success "  opengv          → thirdparty-install/opengv-install"
    echo ""
    info "现在可以编译 iKalibr:"
    info "  colcon build --symlink-install --packages-select ikalibr"
    info "  # 或使用:"
    info "  ./rebuild_ikalibr.sh"
}

trap 'echo ""; error "脚本被中断"' INT TERM
main "$@"
