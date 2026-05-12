#!/bin/bash
# =============================================================================
# verify_ikalibr_build.sh - 验证 iKalibr 编译和安装状态 (无 tiny-viewer 版本)
#
# 功能: 检查 iKalibr 及其第三方依赖库的编译状态
#
# 用法:
#   cd iKalibr
#   chmod +x verify_ikalibr_build.sh
#   ./verify_ikalibr_build.sh
#
# 注意: 此脚本适配 ROS2 Humble 环境，已移除 tiny-viewer 依赖
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
INSTALL_DIR="${PROJECT_ROOT}/thirdparty-install"
WORKSPACE_ROOT="${PROJECT_ROOT}/.."
INSTALL_PREFIX="${WORKSPACE_ROOT}/install"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

success() { echo -e "${GREEN}[PASS]${NC} $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[FAIL]${NC} $1"; }
info() { echo -e "${BLUE}[INFO]${NC} $1"; }
section() { echo -e "${BLUE}>>> $1${NC}"; }

print_header() {
    echo ""
    echo "========================================"
    echo "$1"
    echo "========================================"
    echo ""
}

# =============================================================================
# 检查第三方库
# =============================================================================
check_thirdparty_libs() {
    print_header "检查第三方依赖库"
    
    local all_found=true
    
    # 注意：tiny-viewer 已被移除（无法访问），不再检查
    warning "tiny-viewer: 已移除（可视化功能已禁用）"
    
    # 检查 ctraj
    if [ -d "${INSTALL_DIR}/ctraj-install/lib" ]; then
        if ls "${INSTALL_DIR}/ctraj-install/lib"/libctraj* 1> /dev/null 2>&1; then
            success "ctraj: 已安装"
        else
            warning "ctraj: 目录存在但库文件缺失"
            all_found=false
        fi
    else
        error "ctraj: 未安装"
        all_found=false
    fi
    
    # 检查 ufomap
    if [ -d "${INSTALL_DIR}/ufomap-install/lib" ]; then
        if ls "${INSTALL_DIR}/ufomap-install/lib"/libufomap* 1> /dev/null 2>&1; then
            success "ufomap: 已安装"
        else
            warning "ufomap: 目录存在但库文件缺失"
            all_found=false
        fi
    else
        error "ufomap: 未安装"
        all_found=false
    fi
    
    # 检查 veta
    if [ -d "${INSTALL_DIR}/veta-install/lib" ]; then
        if ls "${INSTALL_DIR}/veta-install/lib"/libveta* 1> /dev/null 2>&1; then
            success "veta: 已安装"
        else
            warning "veta: 目录存在但库文件缺失"
            all_found=false
        fi
    else
        error "veta: 未安装"
        all_found=false
    fi
    
    # 检查 opengv
    if [ -d "${INSTALL_DIR}/opengv-install/lib" ]; then
        if ls "${INSTALL_DIR}/opengv-install/lib"/libopengv* 1> /dev/null 2>&1; then
            success "opengv: 已安装"
        else
            warning "opengv: 目录存在但库文件缺失"
            all_found=false
        fi
    else
        error "opengv: 未安装"
        all_found=false
    fi
    
    if [ "$all_found" = true ]; then
        success "所有第三方依赖库已正确安装"
        warning "注意：tiny-viewer 已被移除，实时可视化功能已禁用"
        return 0
    else
        warning "部分第三方依赖库缺失，iKalibr 功能可能受限"
        return 1
    fi
}

# =============================================================================
# 检查 CMake 配置文件
# =============================================================================
check_cmake_configs() {
    print_header "检查 CMake 配置文件"
    
    # 注意：tiny-viewer 已被移除，不再检查
    warning "tiny-viewer: CMake 配置已移除（可视化功能已禁用）"
    
    local configs=(
        "${INSTALL_DIR}/ctraj-install/lib/cmake/ctraj/ctrajConfig.cmake"
        "${INSTALL_DIR}/ufomap-install/lib/cmake/ufomap/ufomapConfig.cmake"
        "${INSTALL_DIR}/veta-install/lib/cmake/veta/vetaConfig.cmake"
        "${INSTALL_DIR}/opengv-install/lib/cmake/opengv-1.0/opengvConfig.cmake"
    )
    
    local all_found=true
    for config in "${configs[@]}"; do
        if [ -f "${config}" ]; then
            success "$(basename $(dirname $(dirname ${config}))): CMake 配置存在"
        else
            error "$(basename $(dirname $(dirname ${config}))): CMake 配置缺失"
            all_found=false
        fi
    done
    
    if [ "$all_found" = true ]; then
        success "所有 CMake 配置文件存在（tiny-viewer 已移除）"
        return 0
    else
        warning "部分 CMake 配置文件缺失"
        return 1
    fi
}

# =============================================================================
# 检查 iKalibr 编译产物
# =============================================================================
check_ikalibr_build() {
    print_header "检查 iKalibr 编译产物"
    
    # 检查 install 目录
    if [ ! -d "${INSTALL_PREFIX}" ]; then
        error "iKalibr install 目录不存在: ${INSTALL_PREFIX}"
        return 1
    fi
    
    success "install 目录存在: ${INSTALL_PREFIX}"
    
    # 检查库文件
    local lib_dir="${INSTALL_PREFIX}/lib"
    if [ -d "${lib_dir}" ]; then
        info "已编译的库:"
        ls -lh "${lib_dir}"/libikalibr* 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}' || warning "未找到库文件"
    else
        error "库目录不存在: ${lib_dir}"
    fi
    
    # 检查可执行文件
    local bin_dir="${INSTALL_PREFIX}/lib/ikalibr"
    if [ -d "${bin_dir}" ]; then
        info "可执行文件:"
        ls -lh "${bin_dir}"/* 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}' || warning "未找到可执行文件"
    else
        warning "可执行文件目录不存在: ${bin_dir}"
    fi
    
    # 检查主要的可执行文件
    local execs=(
        "${bin_dir}/ikalibr_prog"
        "${bin_dir}/ikalibr_learn"
        "${bin_dir}/ikalibr_imu_intri_calib"
    )
    
    local all_found=true
    for exec_file in "${execs[@]}"; do
        if [ -f "${exec_file}" ] && [ -x "${exec_file}" ]; then
            success "$(basename ${exec_file}): 可执行"
        else
            warning "$(basename ${exec_file}): 不存在或不可执行"
            all_found=false
        fi
    done
    
    return 0
}

# =============================================================================
# 检查环境变量
# =============================================================================
check_environment() {
    print_header "检查环境变量"
    
    # 检查 ROS2 环境
    if [ -n "${ROS_DISTRO}" ]; then
        success "ROS_DISTRO: ${ROS_DISTRO}"
    else
        warning "ROS_DISTRO: 未设置"
    fi
    
    # 检查 AMENT_PREFIX_PATH
    if [ -n "${AMENT_PREFIX_PATH}" ]; then
        success "AMENT_PREFIX_PATH: 已设置"
    else
        warning "AMENT_PREFIX_PATH: 未设置"
    fi
    
    # 检查第三方库环境变量
    local env_vars=(
        # "tiny-viewer_DIR"  # 已移除
        "ctraj_DIR"
        "ufomap_DIR"
        "veta_DIR"
        "opengv_DIR"
    )
    
    local all_set=true
    for var in "${env_vars[@]}"; do
        if [ -n "${!var}" ]; then
            success "${var}: ${!var}"
        else
            warning "${var}: 未设置"
            all_set=false
        fi
    done
    
    if [ "$all_set" = false ]; then
        info ""
        info "提示: 运行以下命令设置第三方库环境变量:"
        # info "export tiny-viewer_DIR=${INSTALL_DIR}/tiny-viewer-install/lib/cmake/tiny-viewer"  # 已移除
        info "export ctraj_DIR=${INSTALL_DIR}/ctraj-install/lib/cmake/ctraj"
        info "export ufomap_DIR=${INSTALL_DIR}/ufomap-install/lib/cmake/ufomap"
        info "export ufomap_INCLUDE_DIR=${INSTALL_DIR}/ufomap-install/include"
        info "export veta_DIR=${INSTALL_DIR}/veta-install/lib/cmake/veta"
        info "export opengv_DIR=${INSTALL_DIR}/opengv-install/lib/cmake/opengv-1.0"
    fi
}

# =============================================================================
# 生成测试报告
# =============================================================================
generate_report() {
    print_header "验证报告"
    
    echo ""
    echo "项目信息:"
    echo "  项目根目录: ${PROJECT_ROOT}"
    echo "  第三方库安装目录: ${INSTALL_DIR}"
    echo "  iKalibr install 目录: ${INSTALL_PREFIX}"
    echo ""
    
    echo "下一步操作:"
    echo "  1. 如果第三方库缺失，运行: ./build_thirdparty_ros2.sh"
    echo "  2. 如果 iKalibr 未编译，在工作空间根目录运行: colcon build --symlink-install"
    echo "  3. 编译完成后，运行: source ${INSTALL_PREFIX}/setup.bash"
    echo ""
    
    echo "使用 iKalibr:"
    echo "  可执行文件路径: ${INSTALL_PREFIX}/lib/ikalibr/"
    echo "  示例: ${INSTALL_PREFIX}/lib/ikalibr/ikalibr_prog --help"
    echo ""
    
    echo "可视化说明:"
    echo "  注意: tiny-viewer 已被移除，实时可视化功能已禁用"
    echo "  替代方案: 使用 RViz2 查看标定结果"
    echo "  命令: rviz2 -d ${INSTALL_PREFIX}/share/ikalibr/config/default.rviz"
    echo ""
}

# =============================================================================
# 主流程
# =============================================================================
main() {
    print_header "iKalibr 编译状态验证 (无 tiny-viewer)"
    
    check_thirdparty_libs
    check_cmake_configs
    check_ikalibr_build
    check_environment
    generate_report
    
    print_header "验证完成"
    success "验证脚本执行完成"
}

# 捕获退出信号
trap 'echo ""; error "脚本被中断"; exit 1' INT TERM

# 执行主流程
main "$@"
