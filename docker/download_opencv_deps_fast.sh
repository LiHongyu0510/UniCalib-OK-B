#!/bin/bash
#=============================================================================
# OpenCV 依赖文件高速下载脚本（优化版）
# 优化点：
#  1. 使用国内镜像源加速
#  2. aria2c 多线程下载
#  3. 自动降级到 curl/wget
#=============================================================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置
OPENCV_VERSION="4.8.0"
OPENCV_DEPS_DIR="./deps/opencv_deps"
MAX_RETRIES=3

# 创建目录
mkdir -p "${OPENCV_DEPS_DIR}"

echo -e "${GREEN}===== OpenCV 依赖高速下载脚本 =====${NC}"
echo "OpenCV 版本: ${OPENCV_VERSION}"
echo "下载目录: ${OPENCV_DEPS_DIR}"
echo ""

# ============================================================================
# 工具检测
# ============================================================================
check_download_tool() {
    if command -v aria2c >/dev/null 2>&1; then
        echo "aria2c"
    elif command -v curl >/dev/null 2>&1; then
        echo "curl"
    elif command -v wget >/dev/null 2>&1; then
        echo "wget"
    else
        echo ""
    fi
}

DOWNLOAD_TOOL=$(check_download_tool)
if [ -z "${DOWNLOAD_TOOL}" ]; then
    echo -e "${RED}❌ 错误: 需要 aria2c/curl/wget 来下载文件${NC}"
    echo -e "${YELLOW}建议安装 aria2c 以获得最佳性能: sudo apt-get install aria2${NC}"
    exit 1
fi

echo -e "${BLUE}✓ 检测到下载工具: ${DOWNLOAD_TOOL}${NC}"

# ============================================================================
# 下载函数（支持镜像源和多线程）
# ============================================================================
download_file() {
    local filename=$1
    local primary_url=$2
    local mirror_urls=$3
    local output_file=$4
    local is_optional=${5:-false}

    if [ -f "${output_file}" ]; then
        local file_size=$(du -h "${output_file}" | cut -f1)
        echo -e "${GREEN}✅ ${filename} 已存在 (${file_size})，跳过下载${NC}"
        return 0
    fi

    echo -e "${YELLOW}=== 下载 ${filename} ===${NC}"
    echo "目标文件: ${output_file}"

    local retry_count=0
    local download_success=false

    while [ ${retry_count} -lt ${MAX_RETRIES} ]; do
        retry_count=$((retry_count + 1))
        echo -e "${BLUE}[尝试 ${retry_count}/${MAX_RETRIES}] 正在下载...${NC}"

        # 构建所有可用的 URL
        local all_urls="${primary_url}"
        if [ -n "${mirror_urls}" ]; then
            all_urls="${all_urls} ${mirror_urls}"
        fi

        case "${DOWNLOAD_TOOL}" in
            aria2c)
                # aria2c 多线程下载（16线程，最优化参数）
                if aria2c -c -x 16 -s 16 -k 1M --timeout=60 --connect-timeout=30 \
                    --max-tries=${MAX_RETRIES} --retry-wait=5 \
                    --file-allocation=none \
                    ${all_urls} \
                    -d "$(dirname "${output_file}")" \
                    -o "$(basename "${output_file}")"; then
                    download_success=true
                    break
                fi
                ;;
            curl)
                # curl 带镜像源支持
                if curl -L -C - --connect-timeout 30 --max-time 600 --retry ${MAX_RETRIES} --retry-delay 5 \
                    -o "${output_file}" "${primary_url}"; then
                    download_success=true
                    break
                fi
                ;;
            wget)
                # wget 带镜像源支持
                if wget --continue --timeout=30 --tries=${MAX_RETRIES} --waitretry=5 \
                    -O "${output_file}" "${primary_url}"; then
                    download_success=true
                    break
                fi
                ;;
        esac

        if [ ${retry_count} -lt ${MAX_RETRIES} ]; then
            echo -e "${YELLOW}⚠️  下载失败，5秒后重试...${NC}"
            sleep 5
        fi
    done

    if [ "${download_success}" = true ]; then
        if [ -f "${output_file}" ]; then
            local file_size=$(du -h "${output_file}" | cut -f1)
            echo -e "${GREEN}✅ ${filename} 下载完成 (${file_size})${NC}"

            # 显示校验和
            if command -v sha256sum >/dev/null 2>&1; then
                echo "SHA256: $(sha256sum "${output_file}" | awk '{print $1}')"
            elif command -v shasum >/dev/null 2>&1; then
                echo "SHA256: $(shasum -a 256 "${output_file}" | awk '{print $1}')"
            fi
            return 0
        fi
    fi

    if [ "${is_optional}" = true ]; then
        echo -e "${YELLOW}⚠️  ${filename} 下载失败（非必需，已跳过）${NC}"
        return 0
    else
        echo -e "${RED}❌ ${filename} 下载失败${NC}"
        return 1
    fi
}

# ============================================================================
# 1. IPPICV (Intel Integrated Performance Primitives)
# ============================================================================
echo ""
echo -e "${YELLOW}=== 下载 IPPICV (核心依赖) ===${NC}"

IPPICV_VERSION="2021.8"
IPPICV_HASH="1224f78da6684df04397ac0f40c961ed37f79ccb"
IPPICV_FILENAME="ippicv_${IPPICV_VERSION}_lnx_intel64_20230330_general.tgz"

# GitHub 原始 URL
IPPICV_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}"

# 镜像源（优先级从高到低）
IPPICV_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}
"

IPPICV_FILE="${OPENCV_DEPS_DIR}/${IPPICV_FILENAME}"

if ! download_file "${IPPICV_FILENAME}" "${IPPICV_URL}" "${IPPICV_MIRRORS}" "${IPPICV_FILE}" false; then
    echo -e "${RED}❌ IPPICV 下载失败，Docker 构建可能会失败${NC}"
    exit 1
fi

# ============================================================================
# 2. Face Landmark Model (可选，用于 DNN 模块)
# ============================================================================
echo ""
echo -e "${YELLOW}=== 下载 Face Landmark Model (可选) ===${NC}"

FACE_LANDMARK_HASH="8afa57abc8299e034cb84c6aee778c83281d239f"
FACE_LANDMARK_FILENAME="face_landmark_model.dat"
FACE_LANDMARK_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/res10_300x300_ssd_iter_140000_fp16.caffemodel"

FACE_LANDMARK_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/res10_300x300_ssd_iter_140000_fp16.caffemodel
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/res10_300x300_ssd_iter_140000_fp16.caffemodel
"

FACE_LANDMARK_FILE="${OPENCV_DEPS_DIR}/${FACE_LANDMARK_FILENAME}"

download_file "${FACE_LANDMARK_FILENAME}" "${FACE_LANDMARK_URL}" "${FACE_LANDMARK_MIRRORS}" "${FACE_LANDMARK_FILE}" true

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
echo ""
echo "下一步:"
echo "  bash docker_build.sh"
