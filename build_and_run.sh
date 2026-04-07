#!/bin/bash
# =============================================================================
# UniCalib 多传感器标定系统 - 一键编译和运行脚本
# 功能: Docker环境管理 → 编译C++主框架 → 执行标定流程
#
# 用法:
#   ./build_and_run.sh                        # 完整流程：构建镜像 + 编译外部工具 + C++ + 运行
#   ./build_and_run.sh --build-only           # 仅构建镜像、外部工具和C++代码，不运行标定
#   ./build_and_run.sh --build-external-only   # 仅编译外部依赖工具（DM-Calib, iKalibr等）
#   ./build_and_run.sh --run-only            # 仅运行标定（需已有构建）
#   ./build_and_run.sh --shell               # 进入容器交互式Shell
#
# 环境变量:
#   CALIB_DATA_DIR       数据目录 (默认: /tmp/calib_data)
#   CALIB_RESULTS_DIR    结果目录 (默认: /tmp/calib_results)
#   AUTO_BUILD_LIVOX     是否自动编译livox驱动 (默认: 0)
# =============================================================================

set -e
clear
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
DOCKER_DIR="${SCRIPT_DIR}/docker"
DOCKER_IMAGE="calib_env:humble"
CPP_PROJECT_DIR="${PROJECT_ROOT}/unicalib_C_plus_plus"

# 数据和结果目录
CALIB_DATA_DIR="${CALIB_DATA_DIR:-/tmp/calib_data}"
CALIB_RESULTS_DIR="${CALIB_RESULTS_DIR:-/tmp/calib_results}"

# 运行模式
MODE_BUILD_ONLY=false
MODE_RUN_ONLY=false
MODE_SHELL=false
BUILD_EXTERNAL_ONLY=false

# 解析参数
for arg in "$@"; do
    [ "$arg" = "--build-only" ] && MODE_BUILD_ONLY=true
    [ "$arg" = "--run-only" ] && MODE_RUN_ONLY=true
    [ "$arg" = "--shell" ] && MODE_SHELL=true
    [ "$arg" = "--build-external-only" ] && BUILD_EXTERNAL_ONLY=true
done

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo ""
    echo "========================================"
    echo "$1"
    echo "========================================"
    echo ""
}

# =============================================================================
# 检查依赖
# =============================================================================

check_dependencies() {
    print_header "检查依赖环境"
    
    # 检查 Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装，请先安装 Docker"
        exit 1
    fi
    print_info "Docker 版本: $(docker --version)"
    
    # 检查 Docker 是否运行
    if ! docker info &> /dev/null; then
        print_error "Docker 未运行，请启动 Docker 服务"
        exit 1
    fi
    
    # 检查 GPU 支持
    if docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &> /dev/null; then
        print_info "GPU 支持检查通过"
    else
        print_warn "未检测到 GPU 支持或 nvidia-docker 未正确配置，可能影响性能"
    fi
    
    # 检查 X11 (GUI支持)
    if [ -n "${DISPLAY}" ]; then
        print_info "DISPLAY 环境变量: ${DISPLAY}"
        xhost +local:docker 2>/dev/null || print_warn "X11 权限设置失败，GUI 可能无法使用"
    else
        print_warn "未设置 DISPLAY 环境变量，GUI 功能将不可用"
    fi
    
    print_info "依赖检查完成"
}

# =============================================================================
# 检查/构建 Docker 镜像
# =============================================================================

ensure_docker_image() {
    print_header "检查 Docker 镜像"
    
    if docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1; then
        print_info "Docker 镜像 ${DOCKER_IMAGE} 已存在"
        return 0
    fi
    
    print_warn "Docker 镜像 ${DOCKER_IMAGE} 不存在，开始构建..."
    
    # 使用项目的 docker_build.sh 构建镜像
    if [ -f "${DOCKER_DIR}/docker_build.sh" ]; then
        print_info "使用 ${DOCKER_DIR}/docker_build.sh 构建镜像..."
        bash "${DOCKER_DIR}/docker_build.sh" --build-only
    else
        print_error "未找到 ${DOCKER_DIR}/docker_build.sh"
        exit 1
    fi
    
    if docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1; then
        print_info "Docker 镜像构建成功"
        return 0
    else
        print_error "Docker 镜像构建失败"
        exit 1
    fi
}

# =============================================================================
# 编译外部依赖工具（在容器内，依赖保存到项目目录）
# =============================================================================

build_external_tools() {
    print_header "编译外部依赖工具"

    EXTERNAL_BUILD_SCRIPT="${PROJECT_ROOT}/build_external_tools.sh"

    if [ ! -f "${EXTERNAL_BUILD_SCRIPT}" ]; then
        print_error "外部工具编译脚本不存在: ${EXTERNAL_BUILD_SCRIPT}"
        return 1
    fi

    print_info "在 Docker 容器内编译外部工具"
    print_info "依赖将安装到项目目录的 .venv/ 中，下次无需重新安装"

    # 在容器内执行编译
    docker run --rm \
        --gpus all \
        --ipc=host \
        -v "${PROJECT_ROOT}:/root/calib_ws:rw" \
        -v "${PROJECT_ROOT}/docker/deps:/root/thirdparty/deps:ro" \
        -e CALIB_DATA_DIR="/root/calib_ws/data" \
        -e CALIB_RESULTS_DIR="/root/calib_ws/results" \
        -e THIRDPARTY_DEPS="/root/thirdparty/deps" \
        "${DOCKER_IMAGE}" \
        bash -c "
            set -e
            echo '[INFO] 开始编译外部工具...'
            cd /root/calib_ws
            bash build_external_tools.sh
            echo '[INFO] 外部工具编译完成'
            echo '[INFO] 虚拟环境已保存到各工具目录的 .venv/'
            echo '[INFO] 下次运行将直接使用，无需重新安装'
        "

    if [ $? -eq 0 ]; then
        print_info "外部工具编译完成"
        print_info "依赖已保存在各工具目录的 .venv/ 中"
        # 自动化验证：iKalibr 第三方库（含 ctraj）必须构建成功
        if [ -d "${PROJECT_ROOT}/iKalibr" ]; then
            CTRAJ_INSTALL="${PROJECT_ROOT}/iKalibr/thirdparty-install/ctraj-install"
            if [ -d "${CTRAJ_INSTALL}" ] && [ -f "${CTRAJ_INSTALL}/lib/libctraj.so" ] 2>/dev/null; then
                print_info "验证通过: iKalibr 第三方库 (ctraj) 已正确安装"
            elif [ -d "${CTRAJ_INSTALL}" ]; then
                print_warn "iKalibr 第三方目录存在，但未检测到 libctraj.so（可能未编译 iKalibr）"
            fi
        fi
        return 0
    else
        print_error "外部工具编译失败"
        return 1
    fi
}

# =============================================================================
# 编译 C++ 主框架
# =============================================================================

build_cpp_project() {
    print_header "编译 C++ 主框架 (unicalib_C_plus_plus)"
    
    # 检查 C++ 项目目录
    if [ ! -d "${CPP_PROJECT_DIR}" ]; then
        print_error "C++ 项目目录不存在: ${CPP_PROJECT_DIR}"
        exit 1
    fi
    
    # 检查配置文件
    CONFIG_FILE="${CPP_PROJECT_DIR}/config/sensors.yaml"
    if [ ! -f "${CONFIG_FILE}" ]; then
        print_warn "配置文件不存在: ${CONFIG_FILE}"
        print_warn "将使用默认配置"
    fi
    
    print_info "在 Docker 容器内编译 C++ 项目..."
    
    # 在容器内执行编译
    docker run --rm \
        --gpus all \
        --ipc=host \
        -v "${PROJECT_ROOT}:/root/calib_ws:rw" \
        -v "${CALIB_DATA_DIR}:/root/calib_ws/data:rw" \
        -v "${CALIB_RESULTS_DIR}:/root/calib_ws/results:rw" \
        -e CALIB_DATA_DIR="/root/calib_ws/data" \
        -e CALIB_RESULTS_DIR="/root/calib_ws/results" \
        -e AUTO_BUILD_LIVOX="${AUTO_BUILD_LIVOX:-0}" \
        "${DOCKER_IMAGE}" \
        bash -c "
            set -e
            echo '[INFO] 开始编译 C++ 项目...'
            cd /root/calib_ws/unicalib_C_plus_plus
            
            # 创建构建目录
            mkdir -p build
            cd build
            
            # CMake 配置
            if [ ! -f 'CMakeCache.txt' ]; then
                echo '[INFO] 运行 CMake...'
                cmake .. -DCMAKE_BUILD_TYPE=Release
            fi
            
            # 编译
            echo '[INFO] 开始编译...'
            make -j\$(nproc)
            
            echo '[INFO] 编译完成!'
            
            # 验证可执行文件
            if [ -f './unicalib_example' ]; then
                echo '[INFO] 可执行文件已生成: ./unicalib_example'
                ls -lh ./unicalib_example
            else
                echo '[ERROR] 未找到可执行文件: ./unicalib_example'
                exit 1
            fi
        "
    
    print_info "C++ 主框架编译完成"
}

# =============================================================================
# 运行标定流程
# =============================================================================

run_calibration() {
    print_header "运行标定流程"
    
    # 检查数据目录
    if [ ! -d "${CALIB_DATA_DIR}" ] || [ -z "$(ls -A ${CALIB_DATA_DIR})" ]; then
        print_error "数据目录不存在或为空: ${CALIB_DATA_DIR}"
        print_error "请先准备标定数据，参考 unicalib_C_plus_plus/README.md"
        exit 1
    fi
    print_info "数据目录: ${CALIB_DATA_DIR}"
    ls -lh "${CALIB_DATA_DIR}" | head -20
    
    # 创建结果目录
    mkdir -p "${CALIB_RESULTS_DIR}"
    print_info "结果目录: ${CALIB_RESULTS_DIR}"
    
    # 检查可执行文件
    EXECUTABLE="${CPP_PROJECT_DIR}/build/unicalib_example"
    if [ ! -f "${EXECUTABLE}" ]; then
        print_error "可执行文件不存在: ${EXECUTABLE}"
        print_error "请先运行编译: ./build_and_run.sh --build-only"
        exit 1
    fi
    
    print_info "开始运行标定流程..."
    echo ""
    
    # 在容器内运行标定
    docker run --rm \
        --gpus all \
        --ipc=host \
        -v "${PROJECT_ROOT}:/root/calib_ws:rw" \
        -v "${CALIB_DATA_DIR}:/root/calib_ws/data:rw" \
        -v "${CALIB_RESULTS_DIR}:/root/calib_ws/results:rw" \
        -e CALIB_DATA_DIR="/root/calib_ws/data" \
        -e CALIB_RESULTS_DIR="/root/calib_ws/results" \
        -e AUTO_BUILD_LIVOX="${AUTO_BUILD_LIVOX:-0}" \
        "${DOCKER_IMAGE}" \
        bash -c "
            set -e
            
            echo '========================================='
            echo '标定流程启动'
            echo '========================================='
            echo ''
            echo '配置文件: /root/calib_ws/unicalib_C_plus_plus/config/sensors.yaml'
            echo '数据目录:   /root/calib_ws/data'
            echo '结果目录:   /root/calib_ws/results'
            echo ''
            
            cd /root/calib_ws/unicalib_C_plus_plus/build
            
            # 运行标定流程
            RUN_PIPELINE=1 ./unicalib_example /root/calib_ws/unicalib_C_plus_plus/config/sensors.yaml /root/calib_ws/data
            
            echo ''
            echo '========================================='
            echo '标定流程完成'
            echo '========================================='
            
            # 显示结果
            if [ -d '/root/calib_ws/results' ]; then
                echo ''
                echo '标定结果:'
                ls -lh /root/calib_ws/results/
            fi
        "
    
    print_info "标定流程执行完成"
}

# =============================================================================
# 进入容器 Shell
# =============================================================================

run_shell() {
    print_header "进入容器交互式 Shell"
    
    # 配置 X11
    xhost +local:docker 2>/dev/null || true
    
    print_info "启动容器..."
    print_info "数据目录: ${CALIB_DATA_DIR}"
    print_info "结果目录: ${CALIB_RESULTS_DIR}"
    
    docker run -it --rm \
        --gpus all \
        --privileged \
        --net=host \
        --ipc=host \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e WORKSPACE=/root/calib_ws \
        -e CALIB_DATA_DIR="/root/calib_ws/data" \
        -e CALIB_RESULTS_DIR="/root/calib_ws/results" \
        -e AUTO_BUILD_LIVOX="${AUTO_BUILD_LIVOX:-0}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${HOME}/.Xauthority:/root/.Xauthority:ro" \
        -v "${PROJECT_ROOT}:/root/calib_ws:rw" \
        -v "${CALIB_DATA_DIR}:/root/calib_ws/data:rw" \
        -v "${CALIB_RESULTS_DIR}:/root/calib_ws/results:rw" \
        "${DOCKER_IMAGE}" \
        bash
}

# =============================================================================
# 主流程
# =============================================================================

main() {
    print_header "UniCalib 多传感器标定系统 - 一键编译和运行"
    
    echo "项目根目录: ${PROJECT_ROOT}"
    echo "Docker 镜像: ${DOCKER_IMAGE}"
    echo "C++ 项目目录: ${CPP_PROJECT_DIR}"
    echo "数据目录: ${CALIB_DATA_DIR}"
    echo "结果目录: ${CALIB_RESULTS_DIR}"
    echo ""
    
    # 根据模式执行
    if [ "$BUILD_EXTERNAL_ONLY" = true ]; then
        # 仅编译外部工具模式（均在容器内执行）
        check_dependencies
        ensure_docker_image
        build_external_tools
    elif [ "$MODE_RUN_ONLY" = true ]; then
        # 仅运行模式
        check_dependencies
        ensure_docker_image
        run_calibration
    elif [ "$MODE_SHELL" = true ]; then
        # Shell 模式
        check_dependencies
        ensure_docker_image
        run_shell
    elif [ "$MODE_BUILD_ONLY" = true ]; then
        # 仅构建模式
        check_dependencies
        ensure_docker_image
        build_external_tools
        build_cpp_project
        print_info "构建完成，使用 --run-only 运行标定流程"
    else
        # 完整流程
        check_dependencies
        ensure_docker_image
        build_external_tools
        build_cpp_project
        run_calibration
    fi
    
    print_header "执行完成"
}

# 捕获退出信号
trap 'echo ""; print_error "脚本被中断"; exit 1' INT TERM

# 执行主流程
main "$@"
