#!/bin/bash
# =============================================================================
# 多标定项目 Docker 镜像构建与运行脚本（参考 reference 流程）
# 功能: 检查/构建镜像 → 可选导出归档 → 启动容器（GPU + X11 + 卷挂载）
#
# 用法:
#   ./docker_build.sh           # 无镜像则构建并运行
#   ./docker_build.sh --run-only # 仅运行，不构建（无镜像则报错）
#   ./docker_build.sh --build-only # 仅构建并导出，不运行
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="calib_env:humble"
IMAGE_ARCHIVE="${SCRIPT_DIR}/calib_env_humble.tar"
OPENCV_DEPS_DIR="${SCRIPT_DIR}/../deps/opencv_deps"

# 从镜像中提取 OpenCV 构建时下载的依赖文件
extract_opencv_deps_from_image() {
    local container_id
    local temp_dir
    local deps_extracted=false

    echo "[INFO] 创建临时容器用于提取依赖文件..."

    # 创建临时容器
    container_id=$(docker create "${IMAGE_NAME}" || echo "")

    if [ -z "${container_id}" ]; then
        echo "[ERROR] 无法创建临时容器"
        return 1
    fi

    # 创建临时目录
    temp_dir=$(mktemp -d)
    trap "rm -rf ${temp_dir}; docker rm -f ${container_id} 2>/dev/null || true" EXIT

    # 尝试从容器中复制 OpenCV 缓存目录
    if docker cp "${container_id}:/root/thirdparty/opencv_deps" "${temp_dir}/" 2>/dev/null; then
        echo "[INFO] 成功从容器中复制 opencv_deps 目录"

        # 检查是否有新文件
        if [ -d "${temp_dir}/opencv_deps" ]; then
            # 确保目标目录存在
            mkdir -p "${OPENCV_DEPS_DIR}"

            # 复制新文件到 deps 目录
            for file in "${temp_dir}"/opencv_deps/*; do
                if [ -f "${file}" ]; then
                    filename=$(basename "${file}")
                    target_file="${OPENCV_DEPS_DIR}/${filename}"

                    # 检查文件是否已存在
                    if [ ! -f "${target_file}" ]; then
                        echo "[INFO] 提取新文件: ${filename}"
                        cp "${file}" "${target_file}"
                        deps_extracted=true
                    else
                        # 比较文件大小，如果不同则更新
                        local old_size=$(stat -f%z "${target_file}" 2>/dev/null || stat -c%s "${target_file}" 2>/dev/null || echo "0")
                        local new_size=$(stat -f%z "${file}" 2>/dev/null || stat -c%s "${file}" 2>/dev/null || echo "0")

                        if [ "${new_size}" -gt "${old_size}" ]; then
                            echo "[INFO] 更新文件（大小变化）: ${filename} (${old_size} -> ${new_size} bytes)"
                            cp "${file}" "${target_file}"
                            deps_extracted=true
                        fi
                    fi
                fi
            done
        fi
    else
        echo "[WARN] 容器中没有 /root/thirdparty/opencv_deps 目录"
        echo "[INFO] OpenCV 可能在构建时直接下载，未保存到缓存目录"
    fi

    # 清理容器
    docker rm -f "${container_id}" 2>/dev/null || true

    if [ "${deps_extracted}" = true ]; then
        echo "[INFO] 成功提取依赖文件到: ${OPENCV_DEPS_DIR}"
        ls -lh "${OPENCV_DEPS_DIR}"
        return 0
    else
        echo "[INFO] 没有提取到新的依赖文件（可能已全部存在）"
        return 0
    fi
}

# 若环境中已有镜像则直接使用；否则尝试从归档载入；否则返回 1
ensure_image() {
    if docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
        echo "[INFO] 镜像 ${IMAGE_NAME} 已存在，直接使用"
        return 0
    fi
    if [ -f "${IMAGE_ARCHIVE}" ]; then
        echo "[INFO] 从归档载入: ${IMAGE_ARCHIVE}"
        docker load -i "${IMAGE_ARCHIVE}" || { echo "[ERROR] docker load 失败"; return 1; }
        return 0
    fi
    return 1
}

RUN_ONLY=false
BUILD_ONLY=false
for arg in "$@"; do
    [ "$arg" = "--run-only" ]   && RUN_ONLY=true
    [ "$arg" = "--build-only" ] && BUILD_ONLY=true
done

# 先尝试使用已有镜像或归档
if ! ensure_image; then
    if [ "$RUN_ONLY" = true ]; then
        echo "[ERROR] 无镜像 ${IMAGE_NAME} 且无归档 ${IMAGE_ARCHIVE}，请先执行构建（不带 --run-only）"
        exit 1
    fi
    
    # 检查 OpenCV 依赖文件
    echo "[INFO] 检查 OpenCV 依赖文件..."
    if [ ! -f "${OPENCV_DEPS_DIR}/ippicv_2021.8_lnx_intel64_20230330_general.tgz" ]; then
        echo "[WARN] IPPICV 文件未找到，开始自动下载..."
        "${SCRIPT_DIR}/download_opencv_deps.sh" || {
            echo "[ERROR] OpenCV 依赖下载失败，请手动运行:"
            echo "  cd docker && ./download_opencv_deps.sh"
            exit 1
        }
    else
        echo "[INFO] OpenCV 依赖文件已就绪"
    fi
    
    echo "[INFO] 开始构建镜像..."
    ( cd "${SCRIPT_DIR}" && docker build -t "${IMAGE_NAME}" . ) 2>&1 | tee "${SCRIPT_DIR}/build.log"
    BUILD_EXIT=${PIPESTATUS[0]}
    if [ "$BUILD_EXIT" -ne 0 ]; then
        echo "[ERROR] 构建失败 (exit $BUILD_EXIT)"
        exit "$BUILD_EXIT"
    fi
    echo "[INFO] 导出镜像到 ${IMAGE_ARCHIVE} ..."
    docker save -o "${IMAGE_ARCHIVE}" "${IMAGE_NAME}" || true

    # 从镜像中提取 OpenCV 构建时下载的依赖文件
    echo "[INFO] 从镜像中提取 OpenCV 依赖文件..."
    extract_opencv_deps_from_image || {
        echo "[WARN] 提取 OpenCV 依赖文件失败（可能需要手动操作）"
    }
fi

ensure_image || exit 1

if [ "$BUILD_ONLY" = true ]; then
    echo "[INFO] 仅构建模式，不启动容器"
    exit 0
fi

# 运行容器：GPU、特权、网络、X11、卷挂载（与 docker-compose 卷约定一致）
xhost +local:docker 2>/dev/null || true
docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e WORKSPACE=/root/calib_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOME}/.Xauthority:/root/.Xauthority:ro" \
  -v "${HOME}/calib_ws:/root/calib_ws:rw" \
  -v "${CALIB_DATA_DIR:-${HOME}/data}:/root/calib_ws/data:rw" \
  -v "${CALIB_RESULTS_DIR:-/tmp/calib_results}:/root/calib_ws/results:rw" \
  "${IMAGE_NAME}" "$@"
