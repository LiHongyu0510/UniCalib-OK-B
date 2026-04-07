#!/bin/bash
# -------- 检查依赖 --------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="${SCRIPT_DIR}/deps"
DOWNLOAD_DIR="${DEPS_DIR}/downloads"

# 若 downloads 中已有 tarball 或已解压目录，则补齐到 deps/cmake、deps/ispc
ensure_deps_from_downloads() {
    # CMake 3.24.0：优先使用已解压目录，否则从 tarball 解压
    if [ ! -f "${DEPS_DIR}/cmake/bin/cmake" ]; then
        if [ -f "${DOWNLOAD_DIR}/cmake-3.24.0-linux-x86_64/bin/cmake" ]; then
            echo "[INFO] 使用 downloads 中已解压的 CMake 3.24.0..."
            rm -rf "${DEPS_DIR}/cmake"
            mv "${DOWNLOAD_DIR}/cmake-3.24.0-linux-x86_64" "${DEPS_DIR}/cmake"
        elif [ -f "${DOWNLOAD_DIR}/cmake-3.24.0-linux-x86_64.tar.gz" ]; then
            if ! tar -tzf "${DOWNLOAD_DIR}/cmake-3.24.0-linux-x86_64.tar.gz" >/dev/null 2>&1; then
                echo "[WARN] cmake-3.24.0-linux-x86_64.tar.gz 已损坏或不完整（可能下载被中断），已删除。"
                rm -f "${DOWNLOAD_DIR}/cmake-3.24.0-linux-x86_64.tar.gz"
            else
                echo "[INFO] 从 downloads 解压 CMake 3.24.0..."
                rm -rf "${DEPS_DIR}/cmake"
                mkdir -p "${DEPS_DIR}/cmake"
                tar -xzf "${DOWNLOAD_DIR}/cmake-3.24.0-linux-x86_64.tar.gz" -C "${DEPS_DIR}/cmake" --strip-components=1
            fi
        fi
    fi
    # ISPC 1.16.1：优先使用已解压目录，否则从 tarball 解压
    if [ ! -f "${DEPS_DIR}/ispc/bin/ispc" ]; then
        if [ -f "${DOWNLOAD_DIR}/ispc-v1.16.1-linux/bin/ispc" ]; then
            echo "[INFO] 使用 downloads 中已解压的 ISPC 1.16.1..."
            rm -rf "${DEPS_DIR}/ispc"
            mv "${DOWNLOAD_DIR}/ispc-v1.16.1-linux" "${DEPS_DIR}/ispc"
            chmod +x "${DEPS_DIR}/ispc/bin/ispc" 2>/dev/null || true
        elif [ -f "${DOWNLOAD_DIR}/ispc-v1.16.1-linux.tar.gz" ]; then
            if ! tar -tzf "${DOWNLOAD_DIR}/ispc-v1.16.1-linux.tar.gz" >/dev/null 2>&1; then
                echo "[WARN] ispc-v1.16.1-linux.tar.gz 已损坏或不完整，已删除。"
                rm -f "${DOWNLOAD_DIR}/ispc-v1.16.1-linux.tar.gz"
            else
                echo "[INFO] 从 downloads 解压 ISPC 1.16.1..."
                mkdir -p "${DEPS_DIR}/ispc"
                tar -xzf "${DOWNLOAD_DIR}/ispc-v1.16.1-linux.tar.gz" -C "${DEPS_DIR}/ispc" --strip-components=1
                chmod +x "${DEPS_DIR}/ispc/bin/ispc" 2>/dev/null || true
            fi
        fi
    fi
}

# 检查依赖是否已下载（或可从 downloads 解压）
check_deps() {
    ensure_deps_from_downloads

    local missing_deps=()

    if [ ! -f "${DEPS_DIR}/cmake/bin/cmake" ]; then
        missing_deps+=("CMake 3.24.0")
    fi

    if [ ! -f "${DEPS_DIR}/ispc/bin/ispc" ]; then
        missing_deps+=("ISPC 1.16.1")
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        echo "========================================="
        echo "  错误: 缺少必要的依赖!"
        echo "========================================="
        for dep in "${missing_deps[@]}"; do
            echo "  - $dep"
        done
        echo ""
        echo "请先运行以下命令下载依赖:"
        echo "  bash ${SCRIPT_DIR}/scripts/download_deps.sh"
        echo ""
        echo "或手动下载到 ${SCRIPT_DIR}/deps/downloads/:"
        echo "  - cmake-3.24.0-linux-x86_64.tar.gz"
        echo "  - ispc-v1.16.1-linux.tar.gz"
        echo "========================================="
        exit 1
    fi
}

check_deps

echo "✓ CMake 3.24.0 已找到"
"${SCRIPT_DIR}/deps/cmake/bin/cmake" --version
echo "✓ ISPC 1.16.1 已找到"
"${SCRIPT_DIR}/deps/ispc/bin/ispc" --version
echo ""

# -------- 镜像名与导出路径 --------
IMAGE_NAME="automap-env:humble"
IMAGE_ARCHIVE="${SCRIPT_DIR}/automap-env_humble.tar"

# 先检查环境中是否有镜像；没有则检查文件夹归档并载入；都没有则返回 1（由调用方决定是否构建）
ensure_image() {
    if docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
        echo "[INFO] 镜像 ${IMAGE_NAME} 已存在于环境中，直接使用"
        return 0
    fi
    if [ -f "${IMAGE_ARCHIVE}" ]; then
        echo "[INFO] 环境中无镜像，从文件夹 ${IMAGE_ARCHIVE} 载入..."
        docker load -i "${IMAGE_ARCHIVE}" || { echo "[ERROR] docker load 失败"; return 1; }
        echo "[INFO] 镜像载入完成"
        return 0
    fi
    return 1
}

# -------- 构建、导出与运行 --------
# 若仅需运行（不构建），可传参 --run-only
RUN_ONLY=false
for arg in "$@"; do
    [ "$arg" = "--run-only" ] && RUN_ONLY=true && break
done

# 先检查环境有无镜像 → 无则检查文件夹并载入 → 无则构建（非 --run-only）
ensure_image
if [ $? -ne 0 ]; then
    if [ "$RUN_ONLY" = true ]; then
        echo "[ERROR] 环境中无镜像 ${IMAGE_NAME}，且文件夹内无归档 ${IMAGE_ARCHIVE}，请先执行构建（不带 --run-only）"
        exit 1
    fi
    echo "[INFO] 环境中无镜像且文件夹内无归档，开始重新构建..."
    docker build -t "${IMAGE_NAME}" . 2>&1 | tee build.log
    BUILD_EXIT=${PIPESTATUS[0]}
    if [ "$BUILD_EXIT" -ne 0 ]; then
        echo "[ERROR] 构建失败 (exit $BUILD_EXIT)"
        exit "$BUILD_EXIT"
    fi
    echo "[INFO] 将镜像导出到 ${IMAGE_ARCHIVE} ..."
    docker save -o "${IMAGE_ARCHIVE}" "${IMAGE_NAME}" || { echo "[WARN] 导出失败，继续运行"; }
fi

ensure_image || exit 1
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v $HOME/automap_ws:/root/automap_ws:rw \
  -v $HOME/data:/data:rw \
  "${IMAGE_NAME}"
