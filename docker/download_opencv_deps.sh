#!/bin/bash
#=============================================================================
# OpenCV 依赖文件下载脚本
# 用途：预下载 IPPICV 等构建时依赖，避免 Docker 构建时网络请求
#=============================================================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 配置
OPENCV_VERSION="4.8.0"
OPENCV_DEPS_DIR="./deps/opencv_deps"

# 创建目录
mkdir -p "${OPENCV_DEPS_DIR}"

echo -e "${GREEN}===== OpenCV 依赖下载脚本 =====${NC}"
echo "OpenCV 版本: ${OPENCV_VERSION}"
echo "下载目录: ${OPENCV_DEPS_DIR}"
echo ""

# ============================================================================
# 1. IPPICV (Intel Integrated Performance Primitives)
# ============================================================================
echo -e "${YELLOW}=== 下载 IPPICV ===${NC}"

IPPICV_VERSION="2021.8"
IPPICV_HASH="1224f78da6684df04397ac0f40c961ed37f79ccb"
IPPICV_FILENAME="ippicv_${IPPICV_VERSION}_lnx_intel64_20230330_general.tgz"
IPPICV_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}"
IPPICV_FILE="${OPENCV_DEPS_DIR}/${IPPICV_FILENAME}"

if [ -f "${IPPICV_FILE}" ]; then
    echo -e "${GREEN}✅ IPPICV 已存在，跳过下载${NC}"
else
    echo "正在下载: ${IPPICV_FILENAME}"
    echo "URL: ${IPPICV_URL}"

    # 尝试 wget，失败则使用 curl
    if command -v wget >/dev/null 2>&1; then
        wget --continue --timeout=30 -O "${IPPICV_FILE}" "${IPPICV_URL}"
    elif command -v curl >/dev/null 2>&1; then
        curl -L -C - --connect-timeout 30 -o "${IPPICV_FILE}" "${IPPICV_URL}"
    else
        echo -e "${RED}❌ 错误: 需要 wget 或 curl 来下载文件${NC}"
        exit 1
    fi

    if [ -f "${IPPICV_FILE}" ]; then
        FILE_SIZE=$(du -h "${IPPICV_FILE}" | cut -f1)
        echo -e "${GREEN}✅ IPPICV 下载完成 (${FILE_SIZE})${NC}"

        # 显示校验和
        if command -v sha256sum >/dev/null 2>&1; then
            echo "SHA256: $(sha256sum "${IPPICV_FILE}" | awk '{print $1}')"
        elif command -v shasum >/dev/null 2>&1; then
            echo "SHA256: $(shasum -a 256 "${IPPICV_FILE}" | awk '{print $1}')"
        fi
    else
        echo -e "${RED}❌ IPPICV 下载失败${NC}"
        exit 1
    fi
fi

# ============================================================================
# 2. Face Landmark Model (可选，用于 DNN 模块)
# ============================================================================
echo ""
echo -e "${YELLOW}=== 下载 Face Landmark Model (可选) ===${NC}"

FACE_LANDMARK_HASH="8afa57abc8299e034cb84c6aee778c83281d239f"
FACE_LANDMARK_FILENAME="face_landmark_model.dat"
FACE_LANDMARK_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/res10_300x300_ssd_iter_140000_fp16.caffemodel"
FACE_LANDMARK_FILE="${OPENCV_DEPS_DIR}/${FACE_LANDMARK_FILENAME}"

if [ -f "${FACE_LANDMARK_FILE}" ]; then
    echo -e "${GREEN}✅ Face Landmark Model 已存在，跳过下载${NC}"
else
    echo "正在下载: ${FACE_LANDMARK_FILENAME}"
    echo "URL: ${FACE_LANDMARK_URL}"

    if command -v wget >/dev/null 2>&1; then
        wget --continue --timeout=30 -O "${FACE_LANDMARK_FILE}" "${FACE_LANDMARK_URL}"
    elif command -v curl >/dev/null 2>&1; then
        curl -L -C - --connect-timeout 30 -o "${FACE_LANDMARK_FILE}" "${FACE_LANDMARK_URL}"
    fi

    if [ -f "${FACE_LANDMARK_FILE}" ]; then
        FILE_SIZE=$(du -h "${FACE_LANDMARK_FILE}" | cut -f1)
        echo -e "${GREEN}✅ Face Landmark Model 下载完成 (${FILE_SIZE})${NC}"
    else
        echo -e "${YELLOW}⚠️  Face Landmark Model 下载失败（非必需，可忽略）${NC}"
    fi
fi

# ============================================================================
# 总结
# ============================================================================
echo ""
echo -e "${GREEN}===== 下载完成 =====${NC}"
echo "依赖文件目录: ${OPENCV_DEPS_DIR}"
echo ""
echo "文件列表:"
ls -lh "${OPENCV_DEPS_DIR}/" | grep -E "\.(tgz|dat)$" || echo "  (无文件)"
echo ""
echo -e "${GREEN}✅ 所有必需的 OpenCV 依赖已准备好！${NC}"
echo -e "${GREEN}✅ 现在可以运行 docker 构建了${NC}"
