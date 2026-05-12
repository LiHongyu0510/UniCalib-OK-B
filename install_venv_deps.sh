#!/bin/bash
# =============================================================================
# 为外部工具创建虚拟环境并安装依赖（仅执行一次）
# 功能: 在项目目录中创建 .venv 虚拟环境，安装所有依赖
# 用法: ./install_venv_deps.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

success() { echo -e "${GREEN}[PASS]${NC} $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[FAIL]${NC} $1"; }
info() { echo -e "${BLUE}[INFO]${NC} $1"; }
section() { echo -e "${CYAN}>>> $1${NC}"; }

echo ""
echo "========================================"
echo "为外部工具安装虚拟环境依赖"
echo "========================================"
echo ""
info "依赖将安装到各工具目录的 .venv/ 中"
info "完成后每次挂载到 Docker 容器即可直接使用"
echo ""

# 安装单个工具的依赖
install_tool_deps() {
    local tool_name="$1"
    local tool_path="$2"
    local req_file="$3"

    section "安装 ${tool_name} 依赖"

    if [ ! -d "${tool_path}" ]; then
        warning "工具目录不存在: ${tool_path}"
        return 0
    fi

    cd "${tool_path}"

    # 创建虚拟环境
    local venv_dir="${tool_path}/.venv"
    if [ -d "${venv_dir}" ]; then
        info "虚拟环境已存在: ${venv_dir}"
    else
        info "创建虚拟环境: ${venv_dir}"
        python3 -m venv "${venv_dir}" || {
            error "虚拟环境创建失败"
            return 1
        }
    fi

    # 激活虚拟环境
    info "激活虚拟环境..."
    source "${venv_dir}/bin/activate"
    local pip_cmd="pip"

    # 升级 pip
    info "升级 pip..."
    ${pip_cmd} install --quiet --upgrade pip setuptools wheel

    # 安装依赖
    if [ -f "${req_file}" ]; then
        info "安装依赖..."
        ${pip_cmd} install --quiet -r "${req_file}" || {
            error "依赖安装失败"
            deactivate
            return 1
        }
        success "依赖安装完成"
    else
        warning "未找到 requirements.txt，跳过"
    fi

    # 创建激活脚本（方便在 Docker 中使用）
    cat > "${tool_path}/activate_venv.sh" << 'EOF'
#!/bin/bash
# 激活虚拟环境脚本
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/.venv/bin/activate"
echo "虚拟环境已激活: $(which python)"
EOF
    chmod +x "${tool_path}/activate_venv.sh"

    # 退出虚拟环境
    deactivate

    # 记录虚拟环境路径
    echo "${venv_dir}" > "${tool_path}/.venv_path"

    success "${tool_name} 安装完成"
    echo ""
}

# 主流程
main() {
    # 检查 python3-venv
    if ! python3 -m venv --help &> /dev/null; then
        error "python3-venv 未安装"
        echo ""
        echo "请运行:"
        echo "  sudo apt install python3-venv"
        exit 1
    fi

    # 安装各工具依赖
    install_tool_deps "DM-Calib" "DM-Calib" "DM-Calib/requirements.txt"
    install_tool_deps "click_calib" "click_calib" "click_calib/requirements.txt"

    # learn-to-calibrate 和 Transformer-IMU-Calibrator 主要是 PyTorch（已在系统安装）
    info "learn-to-calibrate 和 Transformer-IMU-Calibrator 主要使用 PyTorch"
    info "PyTorch 已在 Docker 镜像中预装，无需额外安装"
    echo ""

    # MIAS-LCEC 需要编译 C++ 扩展
    section "MIAS-LCEC 需要手动编译"
    warning "MIAS-LCEC 需要 C++ 扩展编译"
    info "请在 Docker 容器内运行:"
    echo "  ./build_and_run.sh --shell"
    echo "  cd MIAS-LCEC/bin"
    echo "  python3 setup.py build_ext --inplace"
    echo ""

    echo "========================================"
    success "所有依赖安装完成！"
    echo "========================================"
    echo ""
    info "下一步："
    echo "  1. 在 Docker 容器中激活虚拟环境："
    echo "     source DM-Calib/activate_venv.sh"
    echo "     source click_calib/activate_venv.sh"
    echo ""
    echo "  2. 或直接运行工具（脚本会自动激活）："
    echo "     ./build_and_run.sh"
    echo ""
}

# 执行主流程
main "$@"
