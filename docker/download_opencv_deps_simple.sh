#!/bin/bash
#=============================================================================
# OpenCV 依赖文件简单下载脚本（curl 备用方案）
# 当 aria2c 超时时使用
#=============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENCV_DEPS_DIR="${SCRIPT_DIR}/../deps/opencv_deps"
mkdir -p "${OPENCV_DEPS_DIR}"

echo "===== OpenCV 依赖下载脚本（curl 备用方案） ====="
echo "下载目录: ${OPENCV_DEPS_DIR}"
echo ""

# WeChat QR Code 依赖
WECHAT_HASH="a8b69ccc738421293254aec5ddb38bd523503252"

# 文件列表
declare -a FILES=(
    "detect.caffemodel"
    "detect.prototxt"
    "sr.caffemodel"
    "sr.prototxt"
)

for filename in "${FILES[@]}"; do
    local_file="${OPENCV_DEPS_DIR}/${filename}"

    # 检查文件是否已存在
    if [ -f "${local_file}" ]; then
        size=$(stat -f%z "${local_file}" 2>/dev/null || stat -c%s "${local_file}" 2>/dev/null || echo "0")
        echo "[INFO] 文件已存在: ${filename} (${size} bytes)"
        continue
    fi

    echo "[INFO] 下载: ${filename}"

    # 使用 curl 从原始地址下载
    url="https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${filename}"

    # 尝试多次镜像源
    success=false

    # 1. 尝试原始地址
    echo "[INFO]  尝试原始地址..."
    if curl -L -s -o "${local_file}" "${url}" --connect-timeout 30 --max-time 300; then
        size=$(stat -f%z "${local_file}" 2>/dev/null || stat -c%s "${local_file}" 2>/dev/null || echo "0")
        if [ "${size}" -gt 100 ]; then
            echo "[OK]   下载成功: ${filename} (${size} bytes)"
            success=true
        else
            rm -f "${local_file}"
            echo "[FAIL] 文件太小，删除重试..."
        fi
    fi

    # 2. 尝试镜像源（如果原始地址失败）
    if [ "${success}" = false ]; then
        mirrors=(
            "https://ghproxy.com/https://raw.githubusercontent.com/WeChatCV/opencv_3rdparty/${WECHAT_HASH}/${filename}"
            "https://cdn.jsdelivr.net/gh/WeChatCV/opencv_3rdparty@${WECHAT_HASH}/${filename}"
        )

        for mirror_url in "${mirrors[@]}"; do
            echo "[INFO]  尝试镜像: ${mirror_url}"
            if curl -L -s -o "${local_file}" "${mirror_url}" --connect-timeout 30 --max-time 300; then
                size=$(stat -f%z "${local_file}" 2>/dev/null || stat -c%s "${local_file}" 2>/dev/null || echo "0")
                if [ "${size}" -gt 100 ]; then
                    echo "[OK]   下载成功: ${filename} (${size} bytes)"
                    success=true
                    break
                else
                    rm -f "${local_file}"
                    echo "[FAIL] 文件太小，尝试下一个镜像..."
                fi
            fi
        done
    fi

    if [ "${success}" = false ]; then
        echo "[ERROR] ${filename} 下载失败"
        echo "[INFO]  请手动下载并拷贝:"
        echo "       ${url}"
        echo "       cp /path/to/${filename} ${local_file}"
    fi
done

echo ""
echo "===== 下载完成 ====="
echo "依赖文件目录: ${OPENCV_DEPS_DIR}"
ls -lh "${OPENCV_DEPS_DIR}"
