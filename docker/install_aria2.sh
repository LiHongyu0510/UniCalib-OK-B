#!/bin/bash
#=============================================================================
# aria2c 安装脚本
# 用途：一键安装 aria2c 以获得最佳下载性能
#=============================================================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}===== aria2c 安装脚本 =====${NC}"
echo ""

# 检查是否已安装
if command -v aria2c >/dev/null 2>&1; then
    echo -e "${GREEN}✅ aria2c 已安装${NC}"
    aria2c --version | head -1
    echo ""
    echo "可以直接使用高速下载脚本："
    echo "  bash download_opencv_deps_fast.sh"
    exit 0
fi

echo -e "${YELLOW}⚠️  aria2c 未安装，正在安装...${NC}"
echo ""

# 检测系统类型
if [ -f /etc/debian_version ]; then
    # Debian/Ubuntu
    echo "检测到 Debian/Ubuntu 系统"
    echo "执行: sudo apt-get update && sudo apt-get install -y aria2"
    echo ""
    sudo apt-get update && sudo apt-get install -y aria2
elif [ -f /etc/redhat-release ]; then
    # RHEL/CentOS
    echo "检测到 RHEL/CentOS 系统"
    echo "执行: sudo yum install -y epel-release && sudo yum install -y aria2"
    echo ""
    sudo yum install -y epel-release && sudo yum install -y aria2
elif [ -f /etc/arch-release ]; then
    # Arch Linux
    echo "检测到 Arch Linux 系统"
    echo "执行: sudo pacman -S aria2"
    echo ""
    sudo pacman -S aria2
else
    echo -e "${RED}❌ 无法自动检测系统类型${NC}"
    echo "请手动安装 aria2c："
    echo "  - Ubuntu/Debian: sudo apt-get install aria2"
    echo "  - CentOS/RHEL: sudo yum install aria2"
    echo "  - Arch: sudo pacman -S aria2"
    exit 1
fi

# 验证安装
if command -v aria2c >/dev/null 2>&1; then
    echo ""
    echo -e "${GREEN}✅ aria2c 安装成功！${NC}"
    echo ""
    aria2c --version | head -1
    echo ""
    echo "现在可以使用高速下载脚本："
    echo "  bash download_opencv_deps_fast.sh"
else
    echo -e "${RED}❌ aria2c 安装失败${NC}"
    exit 1
fi
