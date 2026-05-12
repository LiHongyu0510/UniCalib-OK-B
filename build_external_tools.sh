#!/bin/bash
# =============================================================================
# build_external_tools.sh - 编译和安装UniCalib外部依赖项目
# 功能: 检测并编译 DM-Calib, learn-to-calibrate, MIAS-LCEC, iKalibr, click_calib, Transformer-IMU-Calibrator
#
# 用法:
#   ./build_external_tools.sh                    # 编译所有检测到的工具
#   ./build_external_tools.sh --tools dm_calib    # 仅编译指定工具
#   ./build_external_tools.sh --skip ikalibr      # 跳过指定工具
#   ./build_external_tools.sh --check-only        # 仅检查，不编译
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
CALIB_ROOT="${PROJECT_ROOT}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

success() { echo -e "${GREEN}[PASS]${NC} $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[FAIL]${NC} $1"; }
info() { echo -e "${BLUE}[INFO]${NC} $1"; }
section() { echo -e "${CYN}>>> $1${NC}"; }

# 解析参数
CHECK_ONLY=false
SELECTED_TOOLS=()
SKIP_TOOLS=()

for arg in "$@"; do
    case "$arg" in
        --check-only)
            CHECK_ONLY=true
            ;;
        --tools)
            shift
            while [[ $# -gt 0 && ! "$1" =~ ^- ]]; do
                SELECTED_TOOLS+=("$1")
                shift
            done
            ;;
        --skip)
            shift
            while [[ $# -gt 0 && ! "$1" =~ ^- ]]; do
                SKIP_TOOLS+=("$1")
                shift
            done
            ;;
        *)
            # 未知参数
            ;;
    esac
done

print_header() {
    echo ""
    echo "========================================"
    echo "$1"
    echo "========================================"
    echo ""
}

# =============================================================================
# 工具检测函数
# =============================================================================

detect_dm_calib() {
    local paths=(
        "${CALIB_ROOT}/DM-Calib"
        "${CALIB_ROOT}/src/DM-Calib"
        "/home/wqs/Documents/github/calibration/DM-Calib"
    )
    for path in "${paths[@]}"; do
        if [ -d "${path}" ] && [ -f "${path}/DMCalib/tools/infer.py" ]; then
            echo "${path}"
            return 0
        fi
    done
    echo ""
}

detect_learn_to_calibrate() {
    local paths=(
        "${CALIB_ROOT}/learn-to-calibrate"
        "${CALIB_ROOT}/src/learn-to-calibrate"
        "/home/wqs/Documents/github/calibration/learn-to-calibrate"
    )
    for path in "${paths[@]}"; do
        if [ -d "${path}" ] && [ -f "${path}/demo/calib.sh" ]; then
            echo "${path}"
            return 0
        fi
    done
    echo ""
}

detect_mias_lcec() {
    local paths=(
        "${CALIB_ROOT}/MIAS-LCEC"
        "${CALIB_ROOT}/src/MIAS-LCEC"
        "/home/wqs/Documents/github/calibration/MIAS-LCEC"
    )
    for path in "${paths[@]}"; do
        if [ -d "${path}" ] && [ -f "${path}/bin/iridescence/setup.py" ]; then
            echo "${path}"
            return 0
        fi
    done
    echo ""
}

detect_ikalibr() {
    local paths=(
        "${CALIB_ROOT}/iKalibr"
        "${CALIB_ROOT}/src/iKalibr"
        "/home/wqs/Documents/github/calibration/iKalibr"
    )
    for path in "${paths[@]}"; do
        if [ -d "${path}" ] && [ -f "${path}/package.xml" ]; then
            echo "${path}"
            return 0
        fi
    done
    echo ""
}

detect_click_calib() {
    local paths=(
        "${CALIB_ROOT}/click_calib"
        "${CALIB_ROOT}/src/click_calib"
        "/home/wqs/Documents/github/calibration/click_calib"
    )
    for path in "${paths[@]}"; do
        if [ -d "${path}" ] && [ -f "${path}/source/optimize.py" ]; then
            echo "${path}"
            return 0
        fi
    done
    echo ""
}

detect_transformer_imu() {
    local paths=(
        "${CALIB_ROOT}/Transformer-IMU-Calibrator"
        "${CALIB_ROOT}/src/Transformer-IMU-Calibrator"
        "/home/wqs/Documents/github/calibration/Transformer-IMU-Calibrator"
    )
    for path in "${paths[@]}"; do
        if [ -d "${path}" ] && [ -f "${path}/eval.py" ]; then
            echo "${path}"
            return 0
        fi
    done
    echo ""
}

# =============================================================================
# 编译函数
# =============================================================================

build_dm_calib() {
    local path="$1"
    section "编译 DM-Calib (Python + PyTorch)"

    if [ "$CHECK_ONLY" = true ]; then
        if [ -f "${path}/requirements.txt" ]; then
            success "依赖文件存在: ${path}/requirements.txt"
            return 0
        else
            error "依赖文件不存在: ${path}/requirements.txt"
            return 1
        fi
    fi

    cd "${path}"

    # 检查虚拟环境是否已存在
    local venv_dir="${path}/.venv"
    if [ -d "${venv_dir}" ]; then
        info "检测到虚拟环境: ${venv_dir}"
        info "激活虚拟环境..."
        source "${venv_dir}/bin/activate"
        local pip_cmd="pip"
        info "使用已有虚拟环境（跳过重新安装）"
    else
        # 创建虚拟环境
        info "创建虚拟环境: ${venv_dir}"
        python3 -m venv "${venv_dir}" || {
            error "虚拟环境创建失败"
            return 1
        }
        info "激活虚拟环境..."
        source "${venv_dir}/bin/activate"
        local pip_cmd="pip"

        # 升级 pip
        ${pip_cmd} install --quiet --upgrade pip setuptools wheel

        # 安装依赖
        info "安装 Python 依赖..."
        if [ -f "requirements.txt" ]; then
            ${pip_cmd} install -q -r requirements.txt || {
                error "依赖安装失败"
                return 1
            }
            success "依赖安装完成"
        else
            warning "未找到 requirements.txt"
        fi

        # 创建激活脚本
        cat > "${path}/activate_venv.sh" << 'EOF'
#!/bin/bash
# 激活虚拟环境脚本
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/.venv/bin/activate"
echo "虚拟环境已激活: $(which python)"
EOF
        chmod +x "${path}/activate_venv.sh"
    fi

    # 记录虚拟环境路径
    echo "${venv_dir}" > "${path}/.venv_path"

    # 检查模型文件
    info "检查预训练模型..."
    if [ ! -d "model" ]; then
        warning "未找到模型目录，请手动下载预训练模型"
        info "模型下载: https://huggingface.co/juneyoung9/DM-Calib"
    else
        success "模型目录存在"
    fi

    success "DM-Calib 准备完成"
}

build_learn_to_calibrate() {
    local path="$1"
    section "编译 learn-to-calibrate (Python + PyTorch)"

    if [ "$CHECK_ONLY" = true ]; then
        if [ -f "${path}/demo/calib.sh" ]; then
            success "核心脚本存在: ${path}/demo/calib.sh"
            return 0
        else
            error "核心脚本不存在"
            return 1
        fi
    fi

    cd "${path}"

    info "learn-to-calibrate 为纯 Python 项目，无需编译"
    info "PyTorch 已在 Docker 镜像中预装，直接使用系统 Python"

    # 检查是否有 requirements.txt
    if [ -f "requirements.txt" ]; then
        local venv_dir="${path}/.venv"
        if [ -d "${venv_dir}" ]; then
            info "虚拟环境已存在: ${venv_dir}"
            info "跳过依赖安装（PyTorch 已在系统中安装）"
        else
            info "创建虚拟环境（仅用于脚本隔离）"
            python3 -m venv "${venv_dir}"
            source "${venv_dir}/bin/activate"
            pip install --quiet --upgrade pip setuptools wheel

            # 创建激活脚本
            cat > "${path}/activate_venv.sh" << 'EOF'
#!/bin/bash
# 激活虚拟环境脚本
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/.venv/bin/activate"
echo "虚拟环境已激活: $(which python)"
EOF
            chmod +x "${path}/activate_venv.sh"

            echo "${venv_dir}" > "${path}/.venv_path"
        fi
    fi

    success "learn-to-calibrate 准备完成"
}

build_mias_lcec() {
    local path="$1"
    section "编译 MIAS-LCEC (C++ + Python)"

    if [ "$CHECK_ONLY" = true ]; then
        if [ -f "${path}/bin/iridescence/setup.py" ]; then
            success "setup.py 存在"
            return 0
        else
            error "setup.py 不存在"
            return 1
        fi
    fi

    cd "${path}"

    # 安装 GLFW（如果未安装）
    info "检查 GLFW 是否已安装..."
    if ! pkg-config --exists glfw3 2>/dev/null; then
        info "GLFW 未安装，开始编译安装..."
        local glfw_root="${CALIB_ROOT}/docker/deps/glfw"
        if [ -d "${glfw_root}" ]; then
            info "使用本地 GLFW 源码: ${glfw_root}"
            mkdir -p /tmp/glfw_build
            cd /tmp/glfw_build
            cmake "${glfw_root}" \
                -DCMAKE_BUILD_TYPE=Release \
                -DBUILD_SHARED_LIBS=ON \
                -DGLFW_BUILD_TESTS=OFF \
                -DGLFW_BUILD_DOCS=OFF \
                -DGLFW_BUILD_EXAMPLES=OFF
            make -j$(nproc)
            make install
            cd "${path}"
            rm -rf /tmp/glfw_build
            ldconfig
            success "GLFW 编译安装完成"
        else
            error "本地 GLFW 源码未找到: ${glfw_root}"
            info "请运行: cd docker/deps && bash download_glfw.sh"
            return 1
        fi
    else
        info "GLFW 已安装: $(pkg-config --modversion glfw3)"
    fi

    # 设置 GLM 环境变量（使用本地 deps 目录）
    local glm_root="${CALIB_ROOT}/docker/deps/glm"
    if [ -d "${glm_root}" ] && [ -f "${glm_root}/glm/glm.hpp" ]; then
        export GLM_ROOT_DIR="${glm_root}"
        export PKG_CONFIG_PATH="${glm_root}:${PKG_CONFIG_PATH}"
        info "使用本地 GLM: ${glm_root}"
    else
        warning "本地 GLM 未找到，将尝试使用系统 GLM"
    fi

    # 编译 C++ 部分
    info "编译 C++ 扩展..."
    cd bin/iridescence
    if [ -f "setup.py" ]; then
        python3 setup.py build_ext --inplace || {
            error "编译失败"
            return 1
        }
        success "C++ 扩展编译完成"
    fi

    # Python 依赖使用系统 Python（PyTorch 已在镜像中安装）
    info "Python 依赖使用系统 Python（PyTorch 已在镜像中预装）"

    # 检查模型文件
    info "检查预训练模型..."
    if [ -f "model/pretrained_overlap_transformer.pth.tar" ]; then
        success "预训练模型存在"
    else
        warning "未找到预训练模型，请手动下载"
        info "模型应放置在: ${path}/model/pretrained_overlap_transformer.pth.tar"
    fi

    success "MIAS-LCEC 编译完成"
}

build_ikalibr() {
    local path="$1"
    section "编译 iKalibr (ROS2 + C++)"
    
    if [ "$CHECK_ONLY" = true ]; then
        if [ -f "${path}/package.xml" ]; then
            success "ROS2 包存在"
            return 0
        else
            error "ROS2 包不存在"
            return 1
        fi
    fi
    
    cd "${path}"
    
    # 检查 ROS2 环境
    if [ -z "${ROS_DISTRO}" ] && [ -f "/opt/ros/humble/setup.bash" ]; then
        info "加载 ROS2 Humble 环境..."
        source /opt/ros/humble/setup.bash
    elif [ -z "${ROS_DISTRO}" ]; then
        error "未找到 ROS2 环境"
        info "请先安装 ROS2 或使用 Docker 环境"
        return 1
    fi
    
    # 检查 colcon
    if ! command -v colcon &> /dev/null; then
        error "colcon 未安装，请先安装: sudo apt install python3-colcon-common-extensions"
        return 1
    fi
    
    # ===== 关键步骤：先编译第三方依赖库 =====
    info "第一步：编译 iKalibr 第三方依赖库..."
    if [ -f "build_thirdparty_ros2.sh" ]; then
        chmod +x build_thirdparty_ros2.sh
        ./build_thirdparty_ros2.sh || {
            error "第三方依赖库编译失败"
            return 1
        }
        success "第三方依赖库编译完成"
    else
        warning "未找到 build_thirdparty_ros2.sh，跳过第三方依赖库编译"
        warning "iKalibr 可能功能受限"
    fi
    
    # 设置第三方库的环境变量（注意：tiny-viewer 已被移除）
    local INSTALL_DIR="${path}/thirdparty-install"
    # 注意：tiny-viewer 已被移除（无法访问），不再设置相关环境变量
    # export tiny-viewer_DIR="${INSTALL_DIR}/tiny-viewer-install/lib/cmake/tiny-viewer"
    export ctraj_DIR="${INSTALL_DIR}/ctraj-install/lib/cmake/ctraj"
    export ufomap_DIR="${INSTALL_DIR}/ufomap-install/lib/cmake/ufomap"
    export ufomap_INCLUDE_DIR="${INSTALL_DIR}/ufomap-install/include"
    export veta_DIR="${INSTALL_DIR}/veta-install/lib/cmake/veta"
    export opengv_DIR="${INSTALL_DIR}/opengv-install/lib/cmake/opengv-1.0"
    
    info "已设置第三方库环境变量（不含 tiny-viewer）"
    
    # 统一使用 docker/deps/glm，避免 colcon 发现多处 glm 导致重复包名错误
    local glm_cmake="${path}/../docker/deps/glm/cmake/glm"
    if [ -f "${glm_cmake}/glmConfig.cmake" ]; then
        export glm_DIR="$(cd "${path}/.." && pwd)/docker/deps/glm/cmake/glm"
        info "使用统一 GLM: glm_DIR=${glm_DIR}"
    fi
    
    # ===== 移除 ctraj 已安装头文件中的 tiny-viewer 引用（保证无 tiny-viewer 可编译）=====
    if [ -f "${path}/patch_ctraj_remove_tinyviewer.sh" ]; then
        chmod +x "${path}/patch_ctraj_remove_tinyviewer.sh"
        "${path}/patch_ctraj_remove_tinyviewer.sh" || warning "ctraj 头文件 patch 未完全应用"
    fi
    
    # ===== 第二步：使用 colcon 编译 iKalibr =====
    info "第二步：使用 colcon 编译 iKalibr (这可能需要较长时间)..."
    
    # 返回到 workspace 根目录（colcon 需要在 src 目录的上级执行）
    cd "${path}/.."
    
    colcon build --symlink-install \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DUSE_THIRDPARTY_LIBS=ON \
        -DUSE_VETA_STUB=ON \
        ${glm_DIR:+-Dglm_DIR="$glm_DIR"} || {
        error "编译失败"
        return 1
    }
    
    success "iKalibr 编译完成"
    info "请运行: source install/setup.bash"
}

build_click_calib() {
    local path="$1"
    section "编译 click_calib (Python)"

    if [ "$CHECK_ONLY" = true ]; then
        if [ -f "${path}/source/optimize.py" ]; then
            success "核心脚本存在: ${path}/source/optimize.py"
            return 0
        else
            error "核心脚本不存在"
            return 1
        fi
    fi

    cd "${path}"

    info "click_calib 为纯 Python 项目，无需编译"

    # 检查虚拟环境是否已存在
    local venv_dir="${path}/.venv"
    if [ -d "${venv_dir}" ]; then
        info "检测到虚拟环境: ${venv_dir}"
        info "激活虚拟环境..."
        source "${venv_dir}/bin/activate"
        local pip_cmd="pip"
        info "使用已有虚拟环境（跳过重新安装）"
    else
        # 创建虚拟环境
        info "创建虚拟环境: ${venv_dir}"
        python3 -m venv "${venv_dir}" || {
            error "虚拟环境创建失败"
            return 1
        }
        info "激活虚拟环境..."
        source "${venv_dir}/bin/activate"
        local pip_cmd="pip"

        # 升级 pip
        ${pip_cmd} install --quiet --upgrade pip setuptools wheel

        # 安装依赖
        if [ -f "requirements.txt" ]; then
            info "安装 Python 依赖..."
            ${pip_cmd} install -q -r requirements.txt || {
                error "依赖安装失败"
                return 1
            }
            success "依赖安装完成"
        fi

        # 创建激活脚本
        cat > "${path}/activate_venv.sh" << 'EOF'
#!/bin/bash
# 激活虚拟环境脚本
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/.venv/bin/activate"
echo "虚拟环境已激活: $(which python)"
EOF
        chmod +x "${path}/activate_venv.sh"
    fi

    # 记录虚拟环境路径
    echo "${venv_dir}" > "${path}/.venv_path"

    success "click_calib 准备完成"
}

build_transformer_imu() {
    local path="$1"
    section "编译 Transformer-IMU-Calibrator (Python + PyTorch)"
    
    if [ "$CHECK_ONLY" = true ]; then
        if [ -f "${path}/eval.py" ]; then
            success "核心脚本存在: ${path}/eval.py"
            return 0
        else
            error "核心脚本不存在"
            return 1
        fi
    fi
    
    cd "${path}"
    
    info "Transformer-IMU-Calibrator 为纯 Python 项目，无需编译"
    info "确保已安装 PyTorch 和相关依赖"
    
    success "Transformer-IMU-Calibrator 准备完成"
}

# =============================================================================
# 主流程
# =============================================================================

main() {
    print_header "UniCalib 外部工具编译"
    
    info "项目根目录: ${CALIB_ROOT}"
    info "检查模式: ${CHECK_ONLY}"
    echo ""
    
    # 检测所有工具
    section "检测外部工具..."
    
    declare -A TOOL_PATHS
    
    DM_CALIB_PATH=$(detect_dm_calib)
    if [ -n "${DM_CALIB_PATH}" ]; then
        TOOL_PATHS[dm_calib]="${DM_CALIB_PATH}"
        success "DM-Calib: ${DM_CALIB_PATH}"
    else
        warning "DM-Calib: 未找到"
    fi
    
    L2C_PATH=$(detect_learn_to_calibrate)
    if [ -n "${L2C_PATH}" ]; then
        TOOL_PATHS[learn_to_calibrate]="${L2C_PATH}"
        success "learn-to-calibrate: ${L2C_PATH}"
    else
        warning "learn-to-calibrate: 未找到"
    fi
    
    MIAS_PATH=$(detect_mias_lcec)
    if [ -n "${MIAS_PATH}" ]; then
        TOOL_PATHS[mias_lcec]="${MIAS_PATH}"
        success "MIAS-LCEC: ${MIAS_PATH}"
    else
        warning "MIAS-LCEC: 未找到"
    fi
    
    IKALIBR_PATH=$(detect_ikalibr)
    if [ -n "${IKALIBR_PATH}" ]; then
        TOOL_PATHS[ikalibr]="${IKALIBR_PATH}"
        success "iKalibr: ${IKALIBR_PATH}"
    else
        warning "iKalibr: 未找到"
    fi
    
    CLICK_PATH=$(detect_click_calib)
    if [ -n "${CLICK_PATH}" ]; then
        TOOL_PATHS[click_calib]="${CLICK_PATH}"
        success "click_calib: ${CLICK_PATH}"
    else
        warning "click_calib: 未找到"
    fi
    
    TIC_PATH=$(detect_transformer_imu)
    if [ -n "${TIC_PATH}" ]; then
        TOOL_PATHS[transformer_imu]="${TIC_PATH}"
        success "Transformer-IMU-Calibrator: ${TIC_PATH}"
    else
        warning "Transformer-IMU-Calibrator: 未找到"
    fi
    
    echo ""
    info "检测到 ${#TOOL_PATHS[@]} 个工具"
    
    if [ "$CHECK_ONLY" = true ]; then
        print_header "检查完成"
        return 0
    fi
    
    # 确定要编译的工具列表
    local tools_to_build=()
    
    if [ ${#SELECTED_TOOLS[@]} -gt 0 ]; then
        # 用户指定了工具
        for tool in "${SELECTED_TOOLS[@]}"; do
            if [ -n "${TOOL_PATHS[$tool]}" ]; then
                tools_to_build+=("$tool")
            else
                error "工具 $tool 未找到或未检测到"
            fi
        done
    else
        # 编译所有检测到的工具
        for tool in dm_calib learn_to_calibrate mias_lcec ikalibr click_calib transformer_imu; do
            if [ -n "${TOOL_PATHS[$tool]}" ]; then
                # 检查是否在跳过列表中
                local skip=false
                for skip_tool in "${SKIP_TOOLS[@]}"; do
                    if [ "$tool" = "$skip_tool" ]; then
                        skip=true
                        break
                    fi
                done
                if [ "$skip" = false ]; then
                    tools_to_build+=("$tool")
                fi
            fi
        done
    fi
    
    # 编译工具
    echo ""
    section "开始编译..."
    
    for tool in "${tools_to_build[@]}"; do
        path="${TOOL_PATHS[$tool]}"
        case "$tool" in
            dm_calib)
                build_dm_calib "${path}"
                ;;
            learn_to_calibrate)
                build_learn_to_calibrate "${path}"
                ;;
            mias_lcec)
                build_mias_lcec "${path}"
                ;;
            ikalibr)
                build_ikalibr "${path}"
                ;;
            click_calib)
                build_click_calib "${path}"
                ;;
            transformer_imu)
                build_transformer_imu "${path}"
                ;;
        esac
        echo ""
    done
    
    # 更新配置文件
    section "更新配置文件"
    
    CONFIG_FILE="${CALIB_ROOT}/unicalib_C_plus_plus/config/sensors.yaml"
    
    if [ -f "${CONFIG_FILE}" ]; then
        for tool in "${!TOOL_PATHS[@]}"; do
            path="${TOOL_PATHS[$tool]}"
            if grep -q "^  ${tool}:" "${CONFIG_FILE}" 2>/dev/null; then
                sed -i "s|^  ${tool}:.*|  ${tool}: \"${path}\"  # auto-detected|" "${CONFIG_FILE}"
            elif grep -q "^  # ${tool}:" "${CONFIG_FILE}" 2>/dev/null; then
                sed -i "s|^  # ${tool}:.*|  ${tool}: \"${path}\"  # auto-detected|" "${CONFIG_FILE}"
            else
                sed -i "/^third_party:/a\\\\  ${tool}: \"${path}\"  # auto-detected" "${CONFIG_FILE}"
            fi
            info "已更新 ${tool}: ${path}"
        done
        success "配置文件已更新: ${CONFIG_FILE}"
    fi
    
    print_header "编译完成"
    info "成功编译 ${#tools_to_build[@]} 个工具"
    
    echo ""
    info "下一步操作："
    echo "  1. 运行 UniCalib 主框架: ./build_and_run.sh"
    echo "  2. 或仅运行: ./build_and_run.sh --run-only"
    echo ""
}

# 捕获退出信号
trap 'echo ""; error "脚本被中断"; exit 1' INT TERM

# 执行主流程
main "$@"
