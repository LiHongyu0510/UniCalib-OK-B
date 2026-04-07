#!/bin/bash
#=============================================================================
# OpenCV 依赖文件超高速下载脚本（极端优化版）
# 适用于网络环境极差的情况
# 优化点：
#  1. 多层镜像源（国内+国际）
#  2. 激进的超时配置
#  3. 降低并发以减少服务器压力
#  4. 自动降级策略
#
# 使用方法：
#   bash download_opencv_deps_ultra.sh              # 默认模式：检查文件完整性
#   bash download_opencv_deps_ultra.sh --skip-existing  # 跳过已存在文件（不检查大小）
#=============================================================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 配置
OPENCV_VERSION="4.8.0"
OPENCV_DEPS_DIR="./deps/opencv_deps"
MAX_RETRIES=5  # 增加到 5 次重试

# 确保在 docker 目录下执行，保证路径正确
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(pwd)" != "${SCRIPT_DIR}" ]; then
    echo -e "${YELLOW}⚠️  切换到脚本所在目录以确保路径正确${NC}"
    cd "${SCRIPT_DIR}"
fi

# 解析命令行参数
SKIP_EXISTING=false
if [ "$1" = "--skip-existing" ]; then
    SKIP_EXISTING=true
    echo -e "${BLUE}模式: 跳过已存在文件（不检查完整性）${NC}"
else
    echo -e "${BLUE}模式: 检查文件完整性${NC}"
fi
echo ""

# 创建目录
mkdir -p "${OPENCV_DEPS_DIR}"

echo -e "${GREEN}===== OpenCV 依赖超高速下载脚本（极端优化版） =====${NC}"
echo "OpenCV 版本: ${OPENCV_VERSION}"
echo "脚本目录: ${SCRIPT_DIR}"
echo "下载目录: ${SCRIPT_DIR}/${OPENCV_DEPS_DIR}"
echo ""

# ============================================================================
# 工具检测
# ============================================================================
DOWNLOAD_TOOL=""
if command -v aria2c >/dev/null 2>&1; then
    DOWNLOAD_TOOL="aria2c"
    echo -e "${BLUE}✓ 使用下载工具: aria2c (多线程)${NC}"
elif command -v curl >/dev/null 2>&1; then
    DOWNLOAD_TOOL="curl"
    echo -e "${BLUE}✓ 使用下载工具: curl${NC}"
elif command -v wget >/dev/null 2>&1; then
    DOWNLOAD_TOOL="wget"
    echo -e "${BLUE}✓ 使用下载工具: wget${NC}"
else
    echo -e "${RED}❌ 错误: 需要 aria2c/curl/wget 来下载文件${NC}"
    exit 1
fi

# ============================================================================
# 下载函数（超优化版）
# ============================================================================
download_file() {
    local filename=$1
    local primary_url=$2
    local mirror_urls=$3
    local output_file=$4
    local is_optional=${5:-false}

    # 检查文件是否已存在
    if [ -f "${output_file}" ]; then
        local file_size=$(du -b "${output_file}" | cut -f1)
        echo -e "${BLUE}检测到已存在文件: ${filename} (${file_size} bytes)${NC}"

        if [ "${SKIP_EXISTING}" = true ]; then
            # 跳过模式：只要目标路径下已有文件就跳过（不检查大小）
            echo -e "${GREEN}✅ ${filename} 已存在，跳过下载${NC}"
            return 0
        else
            # 完整性检查模式：检查文件大小
            # IPPICV 应该约 31MB
            if [[ "${filename}" == *"ippicv"* ]]; then
                if [ ${file_size} -lt 30000000 ]; then
                    echo -e "${YELLOW}⚠️  文件大小异常（预期 ~31MB，实际 ${file_size} bytes）${NC}"
                    echo -e "${YELLOW}⚠️  删除不完整文件并重新下载...${NC}"
                    rm -f "${output_file}"
                else
                    echo -e "${GREEN}✅ ${filename} 文件完整，跳过下载${NC}"
                    return 0
                fi
            else
                # 其他文件：只要存在就跳过
                echo -e "${GREEN}✅ ${filename} 已存在，跳过下载${NC}"
                return 0
            fi
        fi
    fi

    echo -e "${YELLOW}=== 下载 ${filename} ===${NC}"
    echo "目标文件: ${output_file}"
    # 先打印手动下载链接，失败时可浏览器下载后拷贝
    local manual_url="${primary_url}"
    if [ -n "${mirror_urls}" ]; then
        manual_url=$(echo "${mirror_urls}" | grep -v '^[[:space:]]*$' | head -1 | tr -d '[:space:]')
        [ -z "${manual_url}" ] && manual_url="${primary_url}"
    fi
    echo -e "${BLUE}手动下载链接（若自动下载失败可用浏览器打开，下载后拷贝到目标路径）:${NC}"
    echo "  ${manual_url}"
    echo "  拷贝到: ${output_file}"
    echo ""

    local retry_count=0
    local download_success=false

    while [ ${retry_count} -lt ${MAX_RETRIES} ]; do
        retry_count=$((retry_count + 1))
        echo -e "${BLUE}[尝试 ${retry_count}/${MAX_RETRIES}] 正在下载...${NC}"

        # 构建所有可用的 URL（清理空行和空白）
        # 注意：primary_url 是主 URL，但在国内网络环境下优先使用镜像源
        local all_urls=""
        if [ -n "${mirror_urls}" ]; then
            # 过滤掉空行和空白行，优先使用镜像源
            local cleaned_mirrors=$(echo "${mirror_urls}" | grep -v '^[[:space:]]*$' | tr '\n' ' ')
            all_urls="${cleaned_mirrors}"
        fi
        # 将主 URL 放在最后作为兜底
        all_urls="${all_urls} ${primary_url}"

        case "${DOWNLOAD_TOOL}" in
            aria2c)
                # aria2c 超优化参数（适用于网络差的情况）
                # 降低并发，增加超时
                # 注意: 移除 --split-method 参数（1.37.0 版本不支持 random 值）
                if aria2c -c -x 4 -s 4 -k 2M --timeout=180 --connect-timeout=60 \
                    --max-tries=3 --retry-wait=10 \
                    --file-allocation=none \
                    --min-split-size=2M \
                    ${all_urls} \
                    -d "$(dirname "${output_file}")" \
                    -o "$(basename "${output_file}")"; then
                    download_success=true
                    break
                fi
                ;;
            curl)
                # curl 超长超时
                if curl -L -C - --connect-timeout 60 --max-time 1800 --retry ${MAX_RETRIES} --retry-delay 10 --retry-max-time 600 \
                    -o "${output_file}" "${primary_url}"; then
                    download_success=true
                    break
                fi
                ;;
            wget)
                # wget 超长超时
                if wget --continue --timeout=60 --tries=${MAX_RETRIES} --waitretry=10 \
                    -O "${output_file}" "${primary_url}"; then
                    download_success=true
                    break
                fi
                ;;
        esac

        if [ ${retry_count} -lt ${MAX_RETRIES} ]; then
            echo -e "${YELLOW}⚠️  下载失败，10秒后重试...${NC}"
            sleep 10
        fi
    done

    if [ "${download_success}" = true ]; then
        if [ -f "${output_file}" ]; then
            local file_size=$(du -h "${output_file}" | cut -f1)
            local file_size_bytes=$(du -b "${output_file}" | cut -f1)

            echo -e "${GREEN}✅ ${filename} 下载完成 (${file_size})${NC}"

            # IPPICV 完整性检查
            if [[ "${filename}" == *"ippicv"* ]]; then
                if [ ${file_size_bytes} -lt 30000000 ]; then
                    echo -e "${RED}❌ 文件可能不完整（< 30MB），请删除重试${NC}"
                    echo -e "${YELLOW}执行: rm -f ${output_file} && bash download_opencv_deps_ultra.sh${NC}"
                    return 1
                fi
            fi

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
        echo -e "${BLUE}手动下载后拷贝:${NC}"
        echo "  ${manual_url}"
        echo "  cp /path/to/已下载文件 ${output_file}"
        return 0
    else
        echo -e "${RED}❌ ${filename} 下载失败${NC}"
        echo -e "${YELLOW}请手动下载并拷贝:${NC}"
        echo "  1. 浏览器打开: ${manual_url}"
        echo "  2. 下载完成后执行: cp /path/to/已下载文件 ${output_file}"
        return 1
    fi
}

# ============================================================================
# 1. IPPICV (Intel Integrated Performance Primitives) - 核心依赖
# ============================================================================
echo ""
echo -e "${YELLOW}=== 下载 IPPICV (核心依赖，必须成功) ===${NC}"

IPPICV_VERSION="2021.8"
IPPICV_HASH="1224f78da6684df04397ac0f40c961ed37f79ccb"
IPPICV_FILENAME="ippicv_${IPPICV_VERSION}_lnx_intel64_20230330_general.tgz"

# GitHub 原始 URL
IPPICV_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}"

# 镜像源（多层防护）- 不使用注释，aria2c 无法解析
IPPICV_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}
    https://cdn.jsdelivr.net/gh/opencv/opencv_3rdparty@${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}
    https://raw.githubusercontent.com/opencv/opencv_3rdparty/${IPPICV_HASH}/ippicv/${IPPICV_FILENAME}?download=true
"

IPPICV_FILE="${OPENCV_DEPS_DIR}/${IPPICV_FILENAME}"

if ! download_file "${IPPICV_FILENAME}" "${IPPICV_URL}" "${IPPICV_MIRRORS}" "${IPPICV_FILE}" false; then
    echo -e "${RED}❌ IPPICV 下载失败，Docker 构建可能会失败。请按上方「手动下载并拷贝」步骤操作后重新运行本脚本。${NC}"
    exit 1
fi

# ============================================================================
# 2. Face Detection 模型（可选，用于 DNN 模块）
# ============================================================================
echo ""
echo -e "${YELLOW}=== 下载 Face Detection 模型 (可选，失败可跳过) ===${NC}"

# Face Detection 模型配置
# 注意：这是 res10_300x300_ssd_iter_140000_fp16.caffemodel（人脸检测模型）
FACE_LANDMARK_HASH="8afa57abc8299e034cb84c6aee778c83281d239f"
FACE_LANDMARK_FILENAME="res10_300x300_ssd_iter_140000_fp16.caffemodel"
FACE_LANDMARK_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/${FACE_LANDMARK_FILENAME}"

# 镜像源 - 优先使用国内镜像
FACE_LANDMARK_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/${FACE_LANDMARK_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/${FACE_LANDMARK_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/opencv/opencv_3rdparty/${FACE_LANDMARK_HASH}/${FACE_LANDMARK_FILENAME}
    https://cdn.jsdelivr.net/gh/opencv/opencv_3rdparty@${FACE_LANDMARK_HASH}/${FACE_LANDMARK_FILENAME}
"

FACE_LANDMARK_FILE="${OPENCV_DEPS_DIR}/${FACE_LANDMARK_FILENAME}"

download_file "Face Detection Model" "${FACE_LANDMARK_URL}" "${FACE_LANDMARK_MIRRORS}" "${FACE_LANDMARK_FILE}" true

# ============================================================================
# 3. WeChat QR Code 模型（用于 wechat_qrcode 模块）
# ============================================================================
echo ""
echo -e "${YELLOW}=== 下载 WeChat QR Code 模型 (可选，失败可跳过) ===${NC}"

# WeChat QR Code 模型配置
# 参考：https://github.com/WeChatCV/opencv_3rdparty
WECHAT_HASH="a8b69ccc738421293254aec5ddb38bd523503252"
WECHAT_DETECT_FILENAME="detect.caffemodel"
WECHAT_PROTOTXT_FILENAME="detect.prototxt"

WECHAT_DETECT_URL="https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}"
WECHAT_PROTOTXT_URL="https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_PROTOTXT_FILENAME}"

# 镜像源 - 优先使用国内镜像
WECHAT_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
    https://cdn.jsdelivr.net/gh/WeChatCV/opencv_3rdparty@${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
"

# 为每个文件定义独立的镜像源
WECHAT_DETECT_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
    https://cdn.jsdelivr.net/gh/WeChatCV/opencv_3rdparty@${WECHAT_HASH}/${WECHAT_DETECT_FILENAME}
"

WECHAT_PROTOTXT_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_PROTOTXT_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_PROTOTXT_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_PROTOTXT_FILENAME}
    https://cdn.jsdelivr.net/gh/WeChatCV/opencv_3rdparty@${WECHAT_HASH}/${WECHAT_PROTOTXT_FILENAME}
"

WECHAT_DETECT_FILE="${OPENCV_DEPS_DIR}/${WECHAT_DETECT_FILENAME}"
WECHAT_PROTOTXT_FILE="${OPENCV_DEPS_DIR}/${WECHAT_PROTOTXT_FILENAME}"

# WeChat QR Code Super Resolution 模型
WECHAT_SR_CAFFE_FILENAME="sr.caffemodel"
WECHAT_SR_PROTOTXT_FILENAME="sr.prototxt"

WECHAT_SR_CAFFE_URL="https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_CAFFE_FILENAME}"
WECHAT_SR_PROTOTXT_URL="https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_PROTOTXT_FILENAME}"

WECHAT_SR_CAFFE_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_CAFFE_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_CAFFE_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_CAFFE_FILENAME}
    https://cdn.jsdelivr.net/gh/WeChatCV/opencv_3rdparty@${WECHAT_HASH}/${WECHAT_SR_CAFFE_FILENAME}
"

WECHAT_SR_PROTOTXT_MIRRORS="
    https://ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_PROTOTXT_FILENAME}
    https://mirror.ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_PROTOTXT_FILENAME}
    https://ghproxy.net/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${WECHAT_SR_PROTOTXT_FILENAME}
    https://cdn.jsdelivr.net/gh/WeChatCV/opencv_3rdparty@${WECHAT_HASH}/${WECHAT_SR_PROTOTXT_FILENAME}
"

WECHAT_SR_CAFFE_FILE="${OPENCV_DEPS_DIR}/${WECHAT_SR_CAFFE_FILENAME}"
WECHAT_SR_PROTOTXT_FILE="${OPENCV_DEPS_DIR}/${WECHAT_SR_PROTOTXT_FILENAME}"

# 下载 detect.caffemodel
download_file "WeChat QR Code Detection Model" "${WECHAT_DETECT_URL}" "${WECHAT_DETECT_MIRRORS}" "${WECHAT_DETECT_FILE}" true

# 下载 detect.prototxt
download_file "WeChat QR Code Detection Config" "${WECHAT_PROTOTXT_URL}" "${WECHAT_PROTOTXT_MIRRORS}" "${WECHAT_PROTOTXT_FILE}" true

# 下载 sr.caffemodel (Super Resolution 模型)
download_file "WeChat QR Code SR Model" "${WECHAT_SR_CAFFE_URL}" "${WECHAT_SR_CAFFE_MIRRORS}" "${WECHAT_SR_CAFFE_FILE}" true

# 下载 sr.prototxt (Super Resolution 配置)
download_file "WeChat QR Code SR Config" "${WECHAT_SR_PROTOTXT_URL}" "${WECHAT_SR_PROTOTXT_MIRRORS}" "${WECHAT_SR_PROTOTXT_FILE}" true

# ============================================================================
# 4. ADE (Automatic Differentiation Engine) - 用于 cv::graph 模块
# ============================================================================
echo ""
echo -e "${YELLOW}=== 下载 ADE (可选，失败可跳过) ===${NC}"

# ADE 版本配置
ADE_VERSION="v0.1.2a"
ADE_FILENAME="ade-${ADE_VERSION}.zip"
ADE_URL="https://github.com/opencv/ade/archive/${ADE_VERSION}.zip"

# 镜像源 - 优先使用国内镜像
ADE_MIRRORS="
    https://ghproxy.com/https://github.com/opencv/ade/archive/${ADE_VERSION}.zip
    https://mirror.ghproxy.com/https://github.com/opencv/ade/archive/${ADE_VERSION}.zip
    https://ghproxy.net/https://github.com/opencv/ade/archive/${ADE_VERSION}.zip
    https://cdn.jsdelivr.net/gh/opencv/ade@${ADE_VERSION}/archive/${ADE_VERSION}.zip
"

ADE_FILE="${OPENCV_DEPS_DIR}/${ADE_FILENAME}"

download_file "ADE ${ADE_VERSION}" "${ADE_URL}" "${ADE_MIRRORS}" "${ADE_FILE}" true

# ============================================================================
# 总结
# ============================================================================
echo ""
echo -e "${GREEN}===== 下载完成 =====${NC}"
echo "依赖文件目录: ${OPENCV_DEPS_DIR}"
echo ""
echo "文件列表:"
ls -lh "${OPENCV_DEPS_DIR}/" 2>/dev/null | grep -E "\.(tgz|dat|caffemodel|prototxt|zip)$" || echo "  (无文件)"
echo ""
echo -e "${GREEN}✅ 所有必需的 OpenCV 依赖已准备好！${NC}"
echo -e "${GREEN}✅ 现在可以运行 docker 构建了${NC}"
echo ""
echo "下一步:"
echo "  bash docker_build.sh"

echo ""
echo "提示："
echo "  - 如需跳过已存在文件（不检查完整性），使用: bash download_opencv_deps_ultra.sh --skip-existing"
