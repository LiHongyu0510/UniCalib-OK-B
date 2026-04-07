#!/bin/bash
# =============================================================================
# 下载 GLFW 到本地 deps 目录
# 用途: 避免构建时从网络下载，提高构建速度和可靠性
#
# 使用方法:
#   cd docker/deps
#   bash download_glfw.sh
# =============================================================================

set -e

GLFW_VERSION="3.3.8"
GLFW_REPO_URL="https://github.com/glfw/glfw.git"
DEPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GLFW_DIR="${DEPS_DIR}/glfw"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================="
echo "GLFW 下载脚本"
echo "=========================================="
echo ""

# 检查是否已存在
if [ -d "${GLFW_DIR}" ] && [ -f "${GLFW_DIR}/include/GLFW/glfw3.h" ]; then
    echo -e "${GREEN}[INFO]${NC} GLFW 已存在于: ${GLFW_DIR}"
    echo "跳过下载"
    exit 0
fi

# 方法 1: 尝试从 GitHub Releases 下载（最快）
echo -e "${YELLOW}[INFO]${NC} 方法 1: 从 GitHub Releases 下载..."
cd "${DEPS_DIR}"

if command -v wget &> /dev/null; then
    echo "使用 wget 下载..."
    if wget -q --show-progress "https://github.com/glfw/glfw/releases/download/${GLFW_VERSION}/glfw-${GLFW_VERSION}.zip" -O "glfw.zip" 2>&1; then
        unzip -q glfw.zip
        mv glfw-${GLFW_VERSION} glfw
        rm -f glfw.zip
        echo -e "${GREEN}[SUCCESS]${NC} 从 GitHub Releases 下载成功"
    else
        echo -e "${YELLOW}[WARN]${NC} GitHub Releases 下载失败，尝试方法 2..."
        rm -f glfw.zip
        METHOD1_FAILED=true
    fi
elif command -v curl &> /dev/null; then
    echo "使用 curl 下载..."
    if curl -L -o glfw.zip "https://github.com/glfw/glfw/releases/download/${GLFW_VERSION}/glfw-${GLFW_VERSION}.zip" --progress-bar 2>&1; then
        unzip -q glfw.zip
        mv glfw-${GLFW_VERSION} glfw
        rm -f glfw.zip
        echo -e "${GREEN}[SUCCESS]${NC} 从 GitHub Releases 下载成功"
    else
        echo -e "${YELLOW}[WARN]${NC} GitHub Releases 下载失败，尝试方法 2..."
        rm -f glfw.zip
        METHOD1_FAILED=true
    fi
else
    echo -e "${YELLOW}[WARN]${NC} 未找到 wget 或 curl，尝试方法 2..."
    METHOD1_FAILED=true
fi

# 方法 2: 从 GitHub 克隆（备选）
if [ "${METHOD1_FAILED}" = true ]; then
    echo -e "${YELLOW}[INFO]${NC} 方法 2: 从 GitHub 克隆..."
    if command -v git &> /dev/null; then
        echo "使用 git clone --depth 1..."
        git clone --depth 1 --branch "${GLFW_VERSION}" "${GLFW_REPO_URL}" glfw
        echo -e "${GREEN}[SUCCESS]${NC} 从 GitHub 克隆成功"
    else
        echo -e "${RED}[ERROR]${NC} 未找到 git 命令"
        exit 1
    fi
fi

# 验证下载
if [ ! -d "${GLFW_DIR}" ]; then
    echo -e "${RED}[ERROR]${NC} GLFW 目录不存在，下载失败"
    exit 1
fi

if [ ! -f "${GLFW_DIR}/include/GLFW/glfw3.h" ]; then
    echo -e "${RED}[ERROR]${NC} GLFW 头文件缺失，下载不完整"
    exit 1
fi

echo ""
echo -e "${GREEN}[SUCCESS]${NC} GLFW 下载完成"
echo "目录: ${GLFW_DIR}"
echo ""
echo "目录内容:"
ls -lh "${GLFW_DIR}" | head -20
echo ""
echo "后续操作:"
echo "  1. 构建镜像: cd docker && bash docker_build.sh"
echo "  2. GLFW 会自动编译并安装到容器中"
