#!/bin/bash
# PEP 668 修复验证脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

success() { echo -e "${GREEN}[PASS]${NC} $1"; }
error() { echo -e "${RED}[FAIL]${NC} $1"; }
info() { echo -e "${YELLOW}[INFO]${NC} $1"; }

echo "========================================="
echo "PEP 668 修复验证"
echo "========================================="
echo ""

# 检测环境
info "检测运行环境..."
if [ -f /.dockerenv ] || grep -q 'docker\|lxc' /proc/1/cgroup 2>/dev/null; then
    success "运行在 Docker 容器内"
    IN_DOCKER=true
else
    success "运行在宿主机"
    IN_DOCKER=false
fi
echo ""

# 测试 Python 虚拟环境创建
info "测试 Python 虚拟环境创建..."
TEST_VENV_DIR="/tmp/test_venv_$$"
python3 -m venv "${TEST_VENV_DIR}" || {
    error "Python 虚拟环境创建失败"
    exit 1
}
source "${TEST_VENV_DIR}/bin/activate"
success "虚拟环境创建成功: ${TEST_VENV_DIR}"

# 测试 pip 安装
info "测试 pip install..."
pip install --quiet --upgrade pip || {
    error "pip 升级失败"
    rm -rf "${TEST_VENV_DIR}"
    exit 1
}
success "pip 工作正常"

# 清理测试环境
deactivate
rm -rf "${TEST_VENV_DIR}"
info "测试环境已清理"
echo ""

# 检查修改后的脚本
info "检查 build_external_tools.sh..."
if grep -q "python3 -m venv" build_external_tools.sh; then
    success "脚本已包含虚拟环境支持"
else
    error "脚本未包含虚拟环境支持"
    exit 1
fi

# 检查是否有虚拟环境缓存逻辑
if grep -q "venv_dir" build_external_tools.sh; then
    success "脚本包含虚拟环境缓存逻辑"
else
    error "脚本未包含虚拟环境缓存逻辑"
    exit 1
fi
echo ""

# 尝试编译 DM-Calib（测试最小工具）
info "尝试编译 DM-Calib（验证脚本逻辑）..."
if [ -d "DM-Calib" ] && [ -f "DM-Calib/requirements.txt" ]; then
    info "DM-Calib 存在，执行编译测试..."
    ./build_external_tools.sh --tools dm_calib --check-only || {
        error "检查模式失败"
        exit 1
    }
    success "检查模式通过"
else
    info "DM-Calib 不存在，跳过实际编译测试"
fi
echo ""

echo "========================================="
success "所有验证通过！"
echo "========================================="
echo ""
info "下一步："
if [ "$IN_DOCKER" = false ]; then
    echo "  1. 运行完整编译: ./build_external_tools.sh"
    echo "  2. 虚拟环境将自动创建在各工具目录的 .venv/"
    echo "  3. 或使用 Docker 方式: ./build_and_run.sh --shell"
else
    echo "  1. 运行完整编译: ./build_external_tools.sh"
    echo "  2. 将使用系统 pip（不受 PEP 668 限制）"
fi
echo ""
