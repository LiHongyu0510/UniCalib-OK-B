#!/bin/bash
#=============================================================================
# 从 Docker 镜像中提取 OpenCV 依赖文件
#
# 用法:
#   ./extract_opencv_deps.sh [镜像名]
#
# 说明:
#   - 从 Docker 镜像中提取 /root/thirdparty/opencv_deps 目录
#   - 保存到 ../deps/opencv_deps/
#   - 下次构建时可直接使用，避免重新下载
#=============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="${1:-calib_env:humble}"
OPENCV_DEPS_DIR="${SCRIPT_DIR}/../deps/opencv_deps"

echo "===== OpenCV 依赖提取脚本 ====="
echo "镜像名称: ${IMAGE_NAME}"
echo "目标目录: ${OPENCV_DEPS_DIR}"
echo ""

# 检查镜像是否存在
if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "[ERROR] 镜像 ${IMAGE_NAME} 不存在"
    echo "[INFO]  请先运行: docker build -t ${IMAGE_NAME} ."
    exit 1
fi

# 创建临时容器
echo "[INFO] 创建临时容器..."
CONTAINER_ID=$(docker create "${IMAGE_NAME}" echo "临时容器")

if [ -z "${CONTAINER_ID}" ]; then
    echo "[ERROR] 无法创建临时容器"
    exit 1
fi

trap "docker rm -f ${CONTAINER_ID} 2>/dev/null || true" EXIT

# 检查容器中是否有 opencv_deps 目录
echo "[INFO] 检查容器中的 opencv_deps 目录..."
if ! docker exec "${CONTAINER_ID}" test -d /root/thirdparty/opencv_deps 2>/dev/null; then
    echo "[WARN] 容器中没有 /root/thirdparty/opencv_deps 目录"
    echo "[INFO] Dockerfile 可能在构建后删除了该目录"
    echo "[INFO] 尝试查找其他可能的路径..."

    # 尝试查找 OpenCV 缓存目录
    CACHE_DIRS=(
        "/root/.cache"
        "/tmp"
    )

    CACHE_FOUND=false
    for cache_dir in "${CACHE_DIRS[@]}"; do
        if docker exec "${CONTAINER_ID}" test -d "${cache_dir}" 2>/dev/null; then
            echo "[INFO]  检查 ${cache_dir}..."

            # 查找 caffemodel/prototxt 文件
            if docker exec "${CONTAINER_ID}" find "${cache_dir}" -name "*.caffemodel" -o -name "*.prototxt" 2>/dev/null | grep -q .; then
                echo "[INFO]  在 ${cache_dir} 找到相关文件"
                echo "[WARN]  但这些文件可能在缓存目录中，不便于提取"
                CACHE_FOUND=true
            fi
        fi
    done

    if [ "${CACHE_FOUND}" = false ]; then
        echo "[WARN] 未找到 OpenCV 依赖文件"
        echo "[INFO] 这可能是因为:"
        echo "       1. Dockerfile 构建后删除了 opencv_deps 目录"
        echo "       2. 文件未保存到可访问的位置"
        echo ""
        echo "[INFO] 建议方案:"
        echo "       1. 检查 Dockerfile 中是否删除了 opencv_deps 目录"
        echo "       2. 修改 Dockerfile 保留该目录"
        echo "       3. 或者手动下载文件到 ${OPENCV_DEPS_DIR}"
        exit 1
    fi

    docker rm -f "${CONTAINER_ID}" 2>/dev/null || true
    exit 1
fi

# 列出容器中的文件
echo "[INFO] 容器中的 opencv_deps 文件列表:"
docker exec "${CONTAINER_ID}" ls -lh /root/thirdparty/opencv_deps/ 2>/dev/null || true

echo ""
echo "[INFO] 开始提取文件..."

# 创建目标目录
mkdir -p "${OPENCV_DEPS_DIR}"

# 提取文件
EXTRACTED_COUNT=0
for file in docker exec "${CONTAINER_ID}" ls /root/thirdparty/opencv_deps/ 2>/dev/null; do
    if [ -n "$file" ]; then
        source_file="/root/thirdparty/opencv_deps/${file}"
        target_file="${OPENCV_DEPS_DIR}/${file}"

        # 检查目标文件是否已存在
        if [ -f "${target_file}" ]; then
            source_size=$(docker exec "${CONTAINER_ID}" stat -c%s "${source_file}" 2>/dev/null || echo "0")
            target_size=$(stat -c%s "${target_file}" 2>/dev/null || echo "0")

            if [ "${source_size}" -eq "${target_size}" ]; then
                echo "[SKIP] ${file} 已存在且大小相同 (${target_size} bytes)"
                continue
            fi
        fi

        echo "[COPY] ${file}"
        docker cp "${CONTAINER_ID}:${source_file}" "${target_file}" || {
            echo "[WARN] 复制失败: ${file}"
            continue
        }
        EXTRACTED_COUNT=$((EXTRACTED_COUNT + 1))
    fi
done

# 清理容器
docker rm -f "${CONTAINER_ID}" 2>/dev/null || true
trap - EXIT

if [ "${EXTRACTED_COUNT}" -gt 0 ]; then
    echo ""
    echo "[SUCCESS] 成功提取 ${EXTRACTED_COUNT} 个文件"
    echo "[INFO]   依赖目录: ${OPENCV_DEPS_DIR}"
    echo ""
    echo "文件列表:"
    ls -lh "${OPENCV_DEPS_DIR}"
else
    echo ""
    echo "[INFO] 没有提取到新文件（所有文件已存在）"
fi

echo ""
echo "下次构建时将使用本地缓存文件，避免重新下载"
