#!/bin/bash
# =============================================================================
# 环境验证脚本 - 检查一键编译和运行所需的环境
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

print_warn() {
    echo -e "${YELLOW}[!]${NC} $1"
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
WARN_COUNT=0

# =============================================================================
# 检查基础工具
# =============================================================================

check_command() {
    local cmd=$1
    local name=$2
    
    if command -v "$cmd" &> /dev/null; then
        local version=$($cmd --version 2>&1 | head -1)
        print_pass "$name 已安装"
        print_info "  版本: $version"
        ((PASS_COUNT++))
        return 0
    else
        print_fail "$name 未安装"
        ((FAIL_COUNT++))
        return 1
    fi
}

# =============================================================================
# 检查 Docker
# =============================================================================

check_docker() {
    print_header "检查 Docker 环境"
    
    # 检查 Docker 命令
    if check_command "docker" "Docker"; then
        # 检查 Docker 服务状态
        if docker info &> /dev/null; then
            print_pass "Docker 服务运行正常"
            ((PASS_COUNT++))
        else
            print_fail "Docker 服务未运行"
            ((FAIL_COUNT++))
        fi
        
        # 检查 Docker 版本
        local docker_version=$(docker --version | awk '{print $3}' | sed 's/,//')
        print_info "Docker 版本: $docker_version"
    fi
    
    # 检查 docker-compose
    if command -v docker-compose &> /dev/null; then
        check_command "docker-compose" "docker-compose"
    else
        print_warn "docker-compose 未安装，使用 Docker Compose V2"
        if docker compose version &> /dev/null; then
            print_pass "Docker Compose V2 可用"
            ((PASS_COUNT++))
        else
            print_fail "Docker Compose 不可用"
            ((FAIL_COUNT++))
        fi
    fi
}

# =============================================================================
# 检查 NVIDIA 和 GPU
# =============================================================================

check_nvidia() {
    print_header "检查 NVIDIA 环境"
    
    # 检查 nvidia-smi
    if command -v nvidia-smi &> /dev/null; then
        print_pass "nvidia-smi 已安装"
        ((PASS_COUNT++))
        
        # 检查 GPU 信息
        if nvidia-smi &> /dev/null; then
            print_pass "GPU 可用"
            ((PASS_COUNT++))
            
            # 显示 GPU 信息
            local gpu_info=$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | head -1)
            print_info "GPU: $gpu_info"
            
            # 检查 CUDA 版本
            local cuda_version=$(nvidia-smi | grep "CUDA Version" | awk '{print $9}')
            print_info "CUDA 版本: $cuda_version"
        else
            print_fail "GPU 不可用或驱动未加载"
            ((FAIL_COUNT++))
        fi
    else
        print_warn "nvidia-smi 未安装"
        ((WARN_COUNT++))
    fi
    
    # 检查 nvidia-docker
    if docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &> /dev/null 2>&1; then
        print_pass "nvidia-docker 配置正确"
        ((PASS_COUNT++))
    else
        print_warn "nvidia-docker 配置可能有问题"
        print_warn "运行: docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi"
        ((WARN_COUNT++))
    fi
}

# =============================================================================
# 检查 X11
# =============================================================================

check_x11() {
    print_header "检查 X11 (GUI 支持)"
    
    if [ -n "${DISPLAY}" ]; then
        print_pass "DISPLAY 环境变量已设置: ${DISPLAY}"
        ((PASS_COUNT++))
        
        # 检查 X11 服务器
        if xset q &> /dev/null 2>&1; then
            print_pass "X11 服务器运行正常"
            ((PASS_COUNT++))
        else
            print_warn "X11 服务器可能未运行"
            ((WARN_COUNT++))
        fi
        
        # 检查 xhost 权限
        if xhost +local:docker &> /dev/null 2>&1; then
            print_pass "Docker X11 访问权限已配置"
            print_info "运行: xhost -local:docker 以撤销权限"
            ((PASS_COUNT++))
        else
            print_warn "Docker X11 访问权限未配置"
            print_info "运行: xhost +local:docker 以启用权限"
            ((WARN_COUNT++))
        fi
    else
        print_warn "DISPLAY 环境变量未设置"
        print_warn "GUI 功能将不可用（RViz2 等）"
        ((WARN_COUNT++))
    fi
}

# =============================================================================
# 检查项目文件
# =============================================================================

check_project_files() {
    print_header "检查项目文件"
    
    local project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    
    # 检查主脚本
    local files=(
        "build_and_run.sh:一键编译和运行脚本"
        "unicalib_C_plus_plus/CMakeLists.txt:C++ CMake 配置"
        "unicalib_C_plus_plus/config/sensors.yaml:传感器配置文件"
        "docker/Dockerfile:Docker 镜像文件"
        "docker/docker_build.sh:Docker 构建脚本"
    )
    
    for file_info in "${files[@]}"; do
        local file=${file_info%%:*}
        local desc=${file_info##*:}
        
        if [ -f "${project_root}/${file}" ]; then
            print_pass "$desc 存在: $file"
            ((PASS_COUNT++))
        else
            print_fail "$desc 缺失: $file"
            ((FAIL_COUNT++))
        fi
    done
    
    # 检查目录
    local dirs=(
        "unicalib_C_plus_plus/src:C++ 源码目录"
        "docker/deps:Docker 依赖目录"
    )
    
    for dir_info in "${dirs[@]}"; do
        local dir=${dir_info%%:*}
        local desc=${dir_info##*:}
        
        if [ -d "${project_root}/${dir}" ]; then
            print_pass "$desc 存在: $dir"
            ((PASS_COUNT++))
        else
            print_warn "$desc 缺失: $dir"
            ((WARN_COUNT++))
        fi
    done
}

# =============================================================================
# 检查 Docker 镜像
# =============================================================================

check_docker_image() {
    print_header "检查 Docker 镜像"
    
    local image_name="calib_env:humble"
    
    if docker image inspect "${image_name}" &> /dev/null 2>&1; then
        print_pass "Docker 镜像已存在: ${image_name}"
        ((PASS_COUNT++))
        
        # 显示镜像信息
        local image_size=$(docker image inspect "${image_name}" --format='{{.Size}}' | awk '{printf "%.2f GB", $1/1024/1024/1024}')
        local created=$(docker image inspect "${image_name}" --format='{{.Created}}' | awk '{print $1, $2, $3, $4}')
        print_info "镜像大小: $image_size"
        print_info "创建时间: $created"
    else
        print_warn "Docker 镜像不存在: ${image_name}"
        print_info "运行 ./build_and_run.sh 将自动构建镜像"
        ((WARN_COUNT++))
    fi
}

# =============================================================================
# 检查磁盘空间
# =============================================================================

check_disk_space() {
    print_header "检查磁盘空间"
    
    local project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    local df_output=$(df -h "${project_root}" | tail -1)
    local avail=$(echo "$df_output" | awk '{print $4}')
    local used_percent=$(echo "$df_output" | awk '{print $5}' | sed 's/%//')
    
    print_info "可用空间: $avail"
    print_info "已使用: ${used_percent}%"
    
    if [ "$used_percent" -lt 80 ]; then
        print_pass "磁盘空间充足"
        ((PASS_COUNT++))
    elif [ "$used_percent" -lt 90 ]; then
        print_warn "磁盘空间紧张"
        ((WARN_COUNT++))
    else
        print_fail "磁盘空间不足"
        ((FAIL_COUNT++))
    fi
}

# =============================================================================
# 检查网络连接
# =============================================================================

check_network() {
    print_header "检查网络连接"
    
    # 检查基本网络连接
    if ping -c 1 -W 2 8.8.8.8 &> /dev/null 2>&1; then
        print_pass "网络连接正常"
        ((PASS_COUNT++))
    else
        print_warn "网络连接可能有问题"
        ((WARN_COUNT++))
    fi
    
    # 检查 Docker Hub 访问
    if curl -s --connect-timeout 5 https://registry-1.docker.io/v2/ &> /dev/null 2>&1; then
        print_pass "Docker Hub 访问正常"
        ((PASS_COUNT++))
    else
        print_warn "Docker Hub 访问可能有问题"
        print_info "如遇网络问题，可使用本地依赖构建"
        ((WARN_COUNT++))
    fi
}

# =============================================================================
# 生成报告
# =============================================================================

generate_report() {
    print_header "检查报告"
    
    echo -e "总计: $((PASS_COUNT + FAIL_COUNT + WARN_COUNT)) 项检查"
    echo -e "${GREEN}[✓] 通过: ${PASS_COUNT}${NC}"
    echo -e "${RED}[✗] 失败: ${FAIL_COUNT}${NC}"
    echo -e "${YELLOW}[!] 警告: ${WARN_COUNT}${NC}"
    echo ""
    
    if [ $FAIL_COUNT -eq 0 ]; then
        if [ $WARN_COUNT -eq 0 ]; then
            echo -e "${GREEN}环境检查全部通过！${NC}"
            echo -e "${GREEN}可以运行: ./build_and_run.sh${NC}"
        else
            echo -e "${YELLOW}环境检查基本通过，存在 ${WARN_COUNT} 个警告${NC}"
            echo -e "${YELLOW}建议修复警告后运行: ./build_and_run.sh${NC}"
        fi
        return 0
    else
        echo -e "${RED}环境检查失败，存在 ${FAIL_COUNT} 个错误${NC}"
        echo -e "${RED}请修复错误后再运行: ./build_and_run.sh${NC}"
        return 1
    fi
}

# =============================================================================
# 主流程
# =============================================================================

main() {
    echo -e "${BLUE}"
    echo "========================================"
    echo "UniCalib 环境验证"
    echo "========================================"
    echo -e "${NC}"
    echo "此脚本将检查一键编译和运行所需的环境"
    echo ""
    
    # 执行检查
    check_docker
    check_nvidia
    check_x11
    check_project_files
    check_docker_image
    check_disk_space
    check_network
    
    # 生成报告
    generate_report
    
    return $?
}

# 捕获退出信号
trap 'echo ""; print_info "检查被中断"; exit 1' INT TERM

# 执行主流程
main "$@"
