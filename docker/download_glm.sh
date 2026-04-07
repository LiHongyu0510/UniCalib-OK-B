#!/bin/bash
# =============================================================================
# 下载 GLM (OpenGL Mathematics) 到本地 deps 目录
# 用途: 避免构建时从网络下载，提高构建速度和可靠性
#
# 使用方法:
#   cd docker/deps
#   bash download_glm.sh
# =============================================================================

set -e

GLM_VERSION="0.9.9.8"
GLM_REPO_URL="https://github.com/g-truc/glm.git"
GLM_RELEASE_URL="https://github.com/g-truc/glm/releases/download/${GLM_VERSION}/glm-${GLM_VERSION}.zip"
DEPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GLM_DIR="${DEPS_DIR}/glm"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================="
echo "GLM 下载脚本"
echo "=========================================="
echo ""

# 检查是否已存在
if [ -d "${GLM_DIR}" ] && [ -f "${GLM_DIR}/glm/glm.hpp" ]; then
    echo -e "${GREEN}[INFO]${NC} GLM 已存在于: ${GLM_DIR}"
    echo "跳过下载"
    exit 0
fi

# 方法 1: 尝试从 GitHub Releases 下载（最快）
echo -e "${YELLOW}[INFO]${NC} 方法 1: 从 GitHub Releases 下载..."
cd "${DEPS_DIR}"

if command -v wget &> /dev/null; then
    echo "使用 wget 下载..."
    if wget -q --show-progress "${GLM_RELEASE_URL}" -O "glm.zip" 2>&1; then
        unzip -q glm.zip
        rm -f glm.zip
        echo -e "${GREEN}[SUCCESS]${NC} 从 GitHub Releases 下载成功"
    else
        echo -e "${YELLOW}[WARN]${NC} GitHub Releases 下载失败，尝试方法 2..."
        rm -f glm.zip
        METHOD1_FAILED=true
    fi
elif command -v curl &> /dev/null; then
    echo "使用 curl 下载..."
    if curl -L -o glm.zip "${GLM_RELEASE_URL}" --progress-bar 2>&1; then
        unzip -q glm.zip
        rm -f glm.zip
        echo -e "${GREEN}[SUCCESS]${NC} 从 GitHub Releases 下载成功"
    else
        echo -e "${YELLOW}[WARN]${NC} GitHub Releases 下载失败，尝试方法 2..."
        rm -f glm.zip
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
        git clone --depth 1 --branch "${GLM_VERSION}" "${GLM_REPO_URL}" glm
        echo -e "${GREEN}[SUCCESS]${NC} 从 GitHub 克隆成功"
    else
        echo -e "${RED}[ERROR]${NC} 未找到 git 命令"
        exit 1
    fi
fi

# 验证下载
if [ ! -d "${GLM_DIR}" ]; then
    echo -e "${RED}[ERROR]${NC} GLM 目录不存在，下载失败"
    exit 1
fi

if [ ! -f "${GLM_DIR}/glm/glm.hpp" ]; then
    echo -e "${RED}[ERROR]${NC} GLM 头文件缺失，下载不完整"
    exit 1
fi

echo ""
echo -e "${GREEN}[SUCCESS]${NC} GLM 下载完成"
echo "目录: ${GLM_DIR}"
echo ""
echo "目录内容:"
ls -lh "${GLM_DIR}" | head -20
echo ""
echo "后续操作:"
echo "  1. 构建镜像: cd docker && bash docker_build.sh"
echo "  2. GLM 会自动复制到容器 /usr/local/include/glm"
