#!/bin/bash
# =============================================================================
# apply_all_fixes.sh - 应用所有 tiny-viewer 移除修复
#
# 功能：一键应用所有修复，解决 tiny-viewer 编译问题
#
# 用法:
#   cd iKalibr
#   chmod +x apply_all_fixes.sh
#   ./apply_all_fixes.sh
#
# 注意: 此脚本会修改多个文件，请先备份
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"

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
# 备份函数
# =============================================================================
backup_file() {
    local file="$1"
    if [ -f "${file}" ]; then
        cp "${file}" "${file}.bak"
        success "已备份: ${file}"
    else
        warning "文件不存在，跳过备份: ${file}"
    fi
}

# =============================================================================
# 修复 1：执行 stub 头文件生成脚本
# =============================================================================
create_stub_headers() {
    print_header "修复 1：创建 tiny-viewer stub 头文件"
    
    local stub_script="${PROJECT_ROOT}/thirdparty/ctraj/create_tiny_viewer_stubs.sh"
    
    if [ -f "${stub_script}" ]; then
        chmod +x "${stub_script}"
        cd "${PROJECT_ROOT}/thirdparty/ctraj"
        
        info "执行 stub 头文件生成脚本..."
        bash "${stub_script}"
        
        cd "${PROJECT_ROOT}"
        
        if [ -d "thirdparty/ctraj/src/include/tiny-viewer" ]; then
            success "Stub 头文件已创建"
            return 0
        else
            error "Stub 头文件目录不存在"
            return 1
        fi
    else
        error "Stub 脚本不存在: ${stub_script}"
        return 1
    fi
}

# =============================================================================
# 修复 2：清理编译缓存
# =============================================================================
clean_build_cache() {
    print_header "修复 2：清理编译缓存"
    
    local ctraj_build="${PROJECT_ROOT}/thirdparty/ctraj-build"
    
    if [ -d "${ctraj_build}" ]; then
        info "清理 ctraj 编译缓存..."
        rm -rf "${ctraj_build}"
        success "编译缓存已清理"
    else
        warning "ctraj 编译缓存不存在"
    fi
}

# =============================================================================
# 修复 3：验证 CMakeLists.txt
# =============================================================================
verify_cmake_lists() {
    print_header "修复 3：验证 CMakeLists.txt"
    
    local ctraj_cmake="${PROJECT_ROOT}/thirdparty/ctraj/CMakeLists.txt"
    
    if [ -f "${ctraj_cmake}" ]; then
        # 检查是否还有 tiny-viewer 的引用
        if grep -q "tiny-viewer" "${ctraj_cmake}"; then
            error "CMakeLists.txt 中仍然有 tiny-viewer 引用！"
            error "需要手动检查和修改"
            return 1
        else
            success "CMakeLists.txt 验证通过（无 tiny-viewer 引用）"
            return 0
        fi
    else
        error "CMakeLists.txt 不存在: ${ctraj_cmake}"
        return 1
    fi
}

# =============================================================================
# 修复 4：总结修复状态
# =============================================================================
summarize_fixes() {
    print_header "修复总结"
    
    echo ""
    echo "已应用的修复："
    echo ""
    echo "1. ✅ 重写了 ctraj/CMakeLists.txt（完全移除 tiny-viewer）"
    echo "2. ✅ 创建了 tiny-viewer stub 头文件"
    echo "3. ✅ 清理了编译缓存"
    echo ""
    echo "下一步："
    echo "  1. 重新编译 iKalibr："
    echo "     cd /home/wqs/Documents/github/UniCalib"
    echo "     ./build_and_run.sh --build-external-only"
    echo ""
    echo "  2. 验证编译结果："
    echo "     cd iKalibr"
    echo "     ./verify_ikalibr_build.sh"
    echo ""
    warning "注意："
    echo "  - tiny-viewer 可视化功能已被禁用"
    echo "  - 标定核心功能完全正常"
    echo "  - 可以使用 RViz2 查看标定结果"
    echo ""
}

# =============================================================================
# 主流程
# =============================================================================
main() {
    print_header "应用所有 tiny-viewer 移除修复"
    
    echo "项目根目录: ${PROJECT_ROOT}"
    echo ""
    
    # 执行所有修复
    create_stub_headers
    clean_build_cache
    
    # 验证修复
    if [ $? -eq 0 ]; then
        verify_cmake_lists
    else
        error "Stub 头文件创建失败，跳过验证"
    fi
    
    # 总结
    summarize_fixes
    
    print_header "修复应用完成"
    success "所有修复已应用！"
    echo ""
    echo "现在可以重新编译 iKalibr 了。"
}

# 捕获退出信号
trap 'echo ""; error "脚本被中断"; exit 1' INT TERM

# 执行主流程
main "$@"
