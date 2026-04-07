#!/bin/bash
# =============================================================================
# 测试脚本 - 验证 build_and_run.sh 的基本功能
# 注意：此脚本仅测试脚本语法和基本逻辑，不执行完整的编译和运行
# =============================================================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_pass() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_fail() {
    echo -e "${RED}[✗]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[i]${NC} $1"
}

print_header() {
    echo ""
    echo "========================================"
    echo "$1"
    echo "========================================"
    echo ""
}

# 统计变量
PASS_COUNT=0
FAIL_COUNT=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_SCRIPT="${SCRIPT_DIR}/build_and_run.sh"
VERIFY_SCRIPT="${SCRIPT_DIR}/verify_environment.sh"

# =============================================================================
# 测试 1: 检查脚本文件存在性
# =============================================================================

test_script_existence() {
    print_header "测试 1: 检查脚本文件存在性"
    
    local files=(
        "${BUILD_SCRIPT}:build_and_run.sh"
        "${VERIFY_SCRIPT}:verify_environment.sh"
        "${SCRIPT_DIR}/BUILD_AND_RUN_GUIDE.md:BUILD_AND_RUN_GUIDE.md"
        "${SCRIPT_DIR}/QUICK_REFERENCE.md:QUICK_REFERENCE.md"
    )
    
    for file_info in "${files[@]}"; do
        local file=${file_info%%:*}
        local name=${file_info##*:}
        
        if [ -f "${file}" ]; then
            print_pass "${name} 存在"
            ((PASS_COUNT++))
        else
            print_fail "${name} 不存在"
            ((FAIL_COUNT++))
        fi
    done
}

# =============================================================================
# 测试 2: 检查脚本可执行权限
# =============================================================================

test_script_permissions() {
    print_header "测试 2: 检查脚本可执行权限"
    
    local scripts=("${BUILD_SCRIPT}" "${VERIFY_SCRIPT}")
    
    for script in "${scripts[@]}"; do
        if [ -x "${script}" ]; then
            print_pass "$(basename ${script}) 具有可执行权限"
            ((PASS_COUNT++))
        else
            print_fail "$(basename ${script}) 缺少可执行权限"
            print_info "运行: chmod +x ${script}"
            ((FAIL_COUNT++))
        fi
    done
}

# =============================================================================
# 测试 3: 检查 Bash 语法
# =============================================================================

test_bash_syntax() {
    print_header "测试 3: 检查 Bash 语法"
    
    local scripts=("${BUILD_SCRIPT}" "${VERIFY_SCRIPT}")
    
    for script in "${scripts[@]}"; do
        if bash -n "${script}" 2>/dev/null; then
            print_pass "$(basename ${script}) 语法正确"
            ((PASS_COUNT++))
        else
            print_fail "$(basename ${script}) 语法错误"
            ((FAIL_COUNT++))
        fi
    done
}

# =============================================================================
# 测试 4: 检查帮助信息
# =============================================================================

test_help_info() {
    print_header "测试 4: 检查帮助信息"
    
    # 检查脚本头部是否包含关键信息
    local keywords=(
        "用法"
        "环境变量"
        "build_and_run.sh"
        "CALIB_DATA_DIR"
        "CALIB_RESULTS_DIR"
    )
    
    for keyword in "${keywords[@]}"; do
        if grep -q "${keyword}" "${BUILD_SCRIPT}"; then
            print_pass "包含关键字: ${keyword}"
            ((PASS_COUNT++))
        else
            print_warn "缺少关键字: ${keyword}"
        fi
    done
}

# =============================================================================
# 测试 5: 检查函数定义
# =============================================================================

test_function_definitions() {
    print_header "测试 5: 检查函数定义"
    
    local functions=(
        "check_dependencies"
        "ensure_docker_image"
        "build_cpp_project"
        "run_calibration"
        "run_shell"
        "main"
    )
    
    for func in "${functions[@]}"; do
        if grep -q "^${func}()" "${BUILD_SCRIPT}"; then
            print_pass "函数已定义: ${func}"
            ((PASS_COUNT++))
        else
            print_fail "函数未定义: ${func}"
            ((FAIL_COUNT++))
        fi
    done
}

# =============================================================================
# 测试 6: 检查参数解析
# =============================================================================

test_argument_parsing() {
    print_header "测试 6: 检查参数解析"
    
    local flags=(
        "--build-only"
        "--run-only"
        "--shell"
    )
    
    for flag in "${flags[@]}"; do
        if grep -q "\"${flag}\"" "${BUILD_SCRIPT}"; then
            print_pass "支持参数: ${flag}"
            ((PASS_COUNT++))
        else
            print_warn "可能不支持参数: ${flag}"
        fi
    done
}

# =============================================================================
# 测试 7: 检查项目文件引用
# =============================================================================

test_project_references() {
    print_header "测试 7: 检查项目文件引用"
    
    local references=(
        "unicalib_C_plus_plus"
        "docker/Dockerfile"
        "docker/docker_build.sh"
        "config/sensors.yaml"
    )
    
    for ref in "${references[@]}"; do
        if grep -q "${ref}" "${BUILD_SCRIPT}"; then
            print_pass "引用: ${ref}"
            ((PASS_COUNT++))
        else
            print_warn "未引用: ${ref}"
        fi
    done
}

# =============================================================================
# 测试 8: 检查错误处理
# =============================================================================

test_error_handling() {
    print_header "测试 8: 检查错误处理"
    
    local error_patterns=(
        "set -e"
        "print_error"
        "exit 1"
    )
    
    for pattern in "${error_patterns[@]}"; do
        if grep -q "${pattern}" "${BUILD_SCRIPT}"; then
            print_pass "包含错误处理: ${pattern}"
            ((PASS_COUNT++))
        else
            print_warn "缺少错误处理: ${pattern}"
        fi
    done
}

# =============================================================================
# 测试 9: 检查 Docker 命令
# =============================================================================

test_docker_commands() {
    print_header "测试 9: 检查 Docker 命令"
    
    local docker_commands=(
        "docker run"
        "docker image inspect"
    )
    
    for cmd in "${docker_commands[@]}"; do
        if grep -q "${cmd}" "${BUILD_SCRIPT}"; then
            print_pass "包含 Docker 命令: ${cmd}"
            ((PASS_COUNT++))
        else
            print_warn "缺少 Docker 命令: ${cmd}"
        fi
    done
}

# =============================================================================
# 测试 10: 检查环境变量使用
# =============================================================================

test_environment_variables() {
    print_header "测试 10: 检查环境变量使用"
    
    local env_vars=(
        "CALIB_DATA_DIR"
        "CALIB_RESULTS_DIR"
        "AUTO_BUILD_LIVOX"
        "DISPLAY"
    )
    
    for var in "${env_vars[@]}"; do
        if grep -q "${var}" "${BUILD_SCRIPT}"; then
            print_pass "使用环境变量: ${var}"
            ((PASS_COUNT++))
        else
            print_warn "未使用环境变量: ${var}"
        fi
    done
}

# =============================================================================
# 生成测试报告
# =============================================================================

generate_report() {
    print_header "测试报告"
    
    echo -e "总计: $((PASS_COUNT + FAIL_COUNT)) 项测试"
    echo -e "${GREEN}[✓] 通过: ${PASS_COUNT}${NC}"
    echo -e "${RED}[✗] 失败: ${FAIL_COUNT}${NC}"
    echo ""
    
    if [ $FAIL_COUNT -eq 0 ]; then
        echo -e "${GREEN}所有测试通过！${NC}"
        echo ""
        echo "脚本已准备就绪，可以运行："
        echo "  ./build_and_run.sh"
        return 0
    else
        echo -e "${RED}存在 ${FAIL_COUNT} 个失败，请修复后再使用${NC}"
        return 1
    fi
}

# =============================================================================
# 主流程
# =============================================================================

main() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "build_and_run.sh 功能测试"
    echo "========================================"
    echo -e "${NC}"
    echo "此脚本将测试 build_and_run.sh 的基本功能"
    echo "注意：此测试不会执行实际的编译和运行"
    echo ""
    
    # 执行测试
    test_script_existence
    test_script_permissions
    test_bash_syntax
    test_help_info
    test_function_definitions
    test_argument_parsing
    test_project_references
    test_error_handling
    test_docker_commands
    test_environment_variables
    
    # 生成报告
    generate_report
    
    return $?
}

# 捕获退出信号
trap 'echo ""; print_info "测试被中断"; exit 1' INT TERM

# 执行主流程
main "$@"
