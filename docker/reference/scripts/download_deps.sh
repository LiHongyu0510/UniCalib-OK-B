#!/bin/bash
# =============================================================================
# AutoMap-Pro 依赖下载脚本
# 用途: 预下载需要网络访问的依赖库,避免Docker构建时网络问题
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/deps"
DOWNLOAD_DIR="${DEPS_DIR}/downloads"

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 创建下载目录
mkdir -p "${DOWNLOAD_DIR}"

# =============================================================================
# 1. CMake 3.24.0 (Open3D/系统依赖兼容性)
#     官方预编译二进制: https://github.com/Kitware/CMake/releases/download/v3.24.0
# =============================================================================
download_cmake() {
    local CMAKE_VERSION="3.24.0"
    local CMAKE_FILE="cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz"
    local CMAKE_URL="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_FILE}"
    # 官方完整校验和见 https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0-SHA-256.txt
    local CMAKE_SHA256="6055d2473255c7416993c829a0d1a7a649d60526d7d"
    local CMAKE_INSTALL_DIR="${DEPS_DIR}/cmake"
    local CMAKE_DOWNLOAD="${DOWNLOAD_DIR}/${CMAKE_FILE}"

    log_info "检查 CMake ${CMAKE_VERSION}..."

    # 检查是否已安装
    if [ -f "${CMAKE_INSTALL_DIR}/bin/cmake" ]; then
        log_info "CMake 已存在于 ${CMAKE_INSTALL_DIR}"
        "${CMAKE_INSTALL_DIR}/bin/cmake" --version
        return 0
    fi

    # 检查下载文件是否存在
    if [ -f "${CMAKE_DOWNLOAD}" ]; then
        log_warn "下载文件已存在，验证..."
        if [ ${#CMAKE_SHA256} -eq 64 ] && echo "${CMAKE_SHA256}  ${CMAKE_DOWNLOAD}" | sha256sum -c - > /dev/null 2>&1; then
            log_info "SHA256 验证通过"
        elif tar -tzf "${CMAKE_DOWNLOAD}" >/dev/null 2>&1; then
            log_info "压缩包完整性检查通过（未做 SHA256，因脚本中校验和不完整）"
        else
            log_warn "文件损坏或不完整，重新下载"
            rm -f "${CMAKE_DOWNLOAD}"
        fi
    fi

    # 下载
    if [ ! -f "${CMAKE_DOWNLOAD}" ]; then
        log_info "从 GitHub 下载 CMake v${CMAKE_VERSION}..."
        curl -L --retry 3 --retry-delay 10 --connect-timeout 30 \
             -o "${CMAKE_DOWNLOAD}" "${CMAKE_URL}"

        # 验证下载文件
        log_info "验证下载文件..."
        if [ ${#CMAKE_SHA256} -eq 64 ]; then
            if ! echo "${CMAKE_SHA256}  ${CMAKE_DOWNLOAD}" | sha256sum -c -; then
                log_error "CMake 下载文件 SHA256 校验失败!"
                rm -f "${CMAKE_DOWNLOAD}"
                exit 1
            fi
            log_info "SHA256 校验通过"
        elif ! tar -tzf "${CMAKE_DOWNLOAD}" >/dev/null 2>&1; then
            log_error "CMake 下载文件损坏或非合法 tar.gz!"
            rm -f "${CMAKE_DOWNLOAD}"
            exit 1
        else
            log_info "压缩包完整性检查通过"
        fi
    fi

    # 解压
    log_info "解压 CMake 到 ${CMAKE_INSTALL_DIR}..."
    rm -rf "${CMAKE_INSTALL_DIR}"
    mkdir -p "${CMAKE_INSTALL_DIR}"
    tar -xzf "${CMAKE_DOWNLOAD}" -C "${CMAKE_INSTALL_DIR}" --strip-components=1

    # 验证安装
    log_info "验证 CMake 安装..."
    "${CMAKE_INSTALL_DIR}/bin/cmake" --version

    log_info "CMake 安装完成: ${CMAKE_INSTALL_DIR}"
}

# =============================================================================
# 2. ISPC 编译器 (Open3D 依赖)
#    版本: 1.16.1
#    下载地址: https://github.com/ispc/ispc/releases/download/v1.16.1/ispc-v1.16.1-linux.tar.gz
#    SHA256: 88db3d0461147c10ed81053a561ec87d3e14265227c03318f4fcaaadc831037f
# =============================================================================
download_ispc() {
    local ISPC_VERSION="1.16.1"
    local ISPC_FILE="ispc-v${ISPC_VERSION}-linux.tar.gz"
    local ISPC_URL="https://github.com/ispc/ispc/releases/download/v${ISPC_VERSION}/${ISPC_FILE}"
    local ISPC_SHA256="88db3d0461147c10ed81053a561ec87d3e14265227c03318f4fcaaadc831037f"
    local ISPC_DIR="${DEPS_DIR}/ispc"
    local ISPC_DOWNLOAD="${DOWNLOAD_DIR}/${ISPC_FILE}"

    log_info "检查 ISPC 编译器 v${ISPC_VERSION}..."

    # 检查是否已存在
    if [ -f "${ISPC_DIR}/bin/ispc" ]; then
        log_info "ISPC 已存在于 ${ISPC_DIR}"
        "${ISPC_DIR}/bin/ispc" --version
        return 0
    fi

    # 检查下载文件是否存在
    if [ -f "${ISPC_DOWNLOAD}" ]; then
        log_warn "下载文件已存在，验证SHA256..."
        if echo "${ISPC_SHA256}  ${ISPC_DOWNLOAD}" | sha256sum -c - > /dev/null 2>&1; then
            log_info "下载文件验证通过"
        else
            log_warn "下载文件校验失败，重新下载"
            rm -f "${ISPC_DOWNLOAD}"
        fi
    fi

    # 下载
    if [ ! -f "${ISPC_DOWNLOAD}" ]; then
        log_info "从 GitHub 下载 ISPC v${ISPC_VERSION}..."
        curl -L --retry 3 --retry-delay 10 --connect-timeout 30 \
             -o "${ISPC_DOWNLOAD}" "${ISPC_URL}"

        # 验证SHA256
        log_info "验证下载文件..."
        if ! echo "${ISPC_SHA256}  ${ISPC_DOWNLOAD}" | sha256sum -c -; then
            log_error "ISPC 下载文件校验失败!"
            rm -f "${ISPC_DOWNLOAD}"
            exit 1
        fi
        log_info "SHA256 校验通过"
    fi

    # 解压
    log_info "解压 ISPC 到 ${ISPC_DIR}..."
    mkdir -p "${ISPC_DIR}"
    tar -xzf "${ISPC_DOWNLOAD}" -C "${ISPC_DIR}" --strip-components=1

    # 设置可执行权限
    chmod +x "${ISPC_DIR}/bin/ispc"

    # 验证安装
    log_info "验证 ISPC 安装..."
    "${ISPC_DIR}/bin/ispc" --version

    log_info "ISPC 安装完成: ${ISPC_DIR}"
}

# =============================================================================
# 3. Open3D 3rdparty 预下载 (避免 Docker 内 HTTP/2 下载失败 status_code 92)
#    使用 --http1.1 降低 GitHub 连接问题；落盘到 3rdparty_downloads 供 CMake file:// 优先使用
# =============================================================================
OPEN3D_3RDPARTY="${DEPS_DIR}/Open3D/3rdparty_downloads"
download_open3d_3rdparty() {
    if [ ! -d "${DEPS_DIR}/Open3D" ]; then
        log_warn "未找到 ${DEPS_DIR}/Open3D，跳过 Open3D 3rdparty 预下载"
        return 0
    fi
    mkdir -p "${OPEN3D_3RDPARTY}"
    local CURL_CMD="curl -L --retry 3 --retry-delay 5 --connect-timeout 30 --http1.1"
    log_info "Open3D 3rdparty 下载目录: ${OPEN3D_3RDPARTY}"

    download_one() {
        local name="$1"
        local url="$2"
        local file="$3"
        local dir="${OPEN3D_3RDPARTY}/${name}"
        mkdir -p "${dir}"
        if [ -f "${dir}/${file}" ]; then
            log_info "[${name}] 已存在 ${dir}/${file}，跳过"
            return 0
        fi
        log_info "[${name}] 下载 ${file} ..."
        if ${CURL_CMD} -o "${dir}/${file}" "${url}"; then
            log_info "[${name}] 完成"
        else
            log_error "[${name}] 下载失败: ${url}"
            return 1
        fi
    }

    download_one "qhull" "https://github.com/qhull/qhull/archive/refs/tags/v8.0.2.tar.gz" "v8.0.2.tar.gz" || return 1
    download_one "nanoflann" "https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.5.0.tar.gz" "v1.5.0.tar.gz" || return 1
    download_one "stdgpu" "https://github.com/stotko/stdgpu/archive/e10f6f3ccc9902d693af4380c3bcd188ec34a2e6.tar.gz" "e10f6f3ccc9902d693af4380c3bcd188ec34a2e6.tar.gz" || return 1
    download_one "embree" "https://github.com/embree/embree/archive/refs/tags/v3.13.3.tar.gz" "v3.13.3.tar.gz" || return 1
    download_one "parallelstl" "https://github.com/oneapi-src/oneDPL/archive/refs/tags/20190522.tar.gz" "20190522.tar.gz" || return 1
    download_one "msgpack" "https://github.com/msgpack/msgpack-c/releases/download/cpp-3.3.0/msgpack-3.3.0.tar.gz" "msgpack-3.3.0.tar.gz" || return 1
    download_one "poisson" "https://github.com/isl-org/Open3D-PoissonRecon/archive/90f3f064e275b275cff445881ecee5a7c495c9e0.tar.gz" "90f3f064e275b275cff445881ecee5a7c495c9e0.tar.gz" || return 1
    download_one "tinygltf" "https://github.com/syoyo/tinygltf/archive/72f4a55edd54742bca1a71ade8ac70afca1d3f07.tar.gz" "72f4a55edd54742bca1a71ade8ac70afca1d3f07.tar.gz" || return 1
    download_one "tinyobjloader" "https://github.com/tinyobjloader/tinyobjloader/archive/refs/tags/v2.0.0rc8.tar.gz" "v2.0.0rc8.tar.gz" || return 1

    log_info "Open3D 3rdparty 预下载完成"
}

# =============================================================================
# 4. nanoflann (修复 Open3D 0.18.0 ExternalProject bug)
#    版本: 1.5.0
#    下载地址: https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.5.0.tar.gz
#    SHA256: 89aecfef1a956ccba7e40f24561846d064f309bc547cc184af7f4426e42f8e65
# =============================================================================
download_nanoflann() {
    local NANOFLANN_VERSION="1.5.0"
    local NANOFLANN_FILE="nanoflann-${NANOFLANN_VERSION}.tar.gz"
    local NANOFLANN_URL="https://github.com/jlblancoc/nanoflann/archive/refs/tags/v${NANOFLANN_VERSION}.tar.gz"
    local NANOFLANN_SHA256="89aecfef1a956ccba7e40f24561846d064f309bc547cc184af7f4426e42f8e65"
    local NANOFLANN_DIR="${DEPS_DIR}/nanoflann"
    local NANOFLANN_DOWNLOAD="${DOWNLOAD_DIR}/${NANOFLANN_FILE}"

    log_info "检查 nanoflann v${NANOFLANN_VERSION}..."

    # 检查是否已存在
    if [ -f "${NANOFLANN_DIR}/include/nanoflann.hpp" ]; then
        log_info "nanoflann 已存在于 ${NANOFLANN_DIR}"
        return 0
    fi

    # 检查下载文件是否存在
    if [ -f "${NANOFLANN_DOWNLOAD}" ]; then
        log_warn "下载文件已存在，验证SHA256..."
        if echo "${NANOFLANN_SHA256}  ${NANOFLANN_DOWNLOAD}" | sha256sum -c - > /dev/null 2>&1; then
            log_info "下载文件验证通过"
        else
            log_warn "下载文件校验失败，重新下载"
            rm -f "${NANOFLANN_DOWNLOAD}"
        fi
    fi

    # 下载
    if [ ! -f "${NANOFLANN_DOWNLOAD}" ]; then
        log_info "从 GitHub 下载 nanoflann v${NANOFLANN_VERSION}..."
        curl -L --retry 3 --retry-delay 5 --connect-timeout 30 --http1.1 \
             -o "${NANOFLANN_DOWNLOAD}" "${NANOFLANN_URL}"

        # 验证SHA256
        log_info "验证下载文件..."
        if ! echo "${NANOFLANN_SHA256}  ${NANOFLANN_DOWNLOAD}" | sha256sum -c -; then
            log_error "nanoflann 下载文件校验失败!"
            rm -f "${NANOFLANN_DOWNLOAD}"
            exit 1
        fi
        log_info "SHA256 校验通过"
    fi

    # 解压 (nanoflann 是 header-only 库)
    log_info "解压 nanoflann 到 ${NANOFLANN_DIR}..."
    mkdir -p "${NANOFLANN_DIR}"
    tar -xzf "${NANOFLANN_DOWNLOAD}" -C "${NANOFLANN_DIR}" --strip-components=1

    # 验证安装
    log_info "验证 nanoflann 安装..."
    if [ ! -f "${NANOFLANN_DIR}/include/nanoflann.hpp" ]; then
        log_error "nanoflann 安装失败，未找到 nanoflann.hpp"
        exit 1
    fi

    log_info "nanoflann 安装完成: ${NANOFLANN_DIR}"
}

# =============================================================================
# 5. tinyply (TEASER++ 依赖，头文件库，用于 PLY 读写)
#    仓库: https://github.com/ddiakopoulos/tinyply
#    用途: Docker 构建时避免 FetchContent 从 GitHub 拉取导致网络超时
# =============================================================================
download_tinyply() {
    local TINYPLY_URL="https://github.com/ddiakopoulos/tinyply/archive/refs/heads/main.zip"
    local TINYPLY_DIR="${DEPS_DIR}/tinyply"
    local TINYPLY_ZIP="${DOWNLOAD_DIR}/tinyply-main.zip"

    log_info "检查 tinyply (TEASER++ 依赖)..."

    if [ -f "${TINYPLY_DIR}/source/tinyply.h" ]; then
        log_info "tinyply 已存在于 ${TINYPLY_DIR}"
        return 0
    fi

    if [ ! -f "${TINYPLY_ZIP}" ]; then
        log_info "从 GitHub 下载 tinyply..."
        curl -L --retry 3 --retry-delay 5 --connect-timeout 30 \
             -o "${TINYPLY_ZIP}" "${TINYPLY_URL}"
    fi

    log_info "解压 tinyply 到 ${TINYPLY_DIR}..."
    rm -rf "${TINYPLY_DIR}"
    unzip -q -o "${TINYPLY_ZIP}" -d "${DEPS_DIR}"
    mv "${DEPS_DIR}/tinyply-main" "${TINYPLY_DIR}"

    if [ ! -f "${TINYPLY_DIR}/source/tinyply.h" ]; then
        log_error "tinyply 解压后未找到 source/tinyply.h"
        exit 1
    fi
    log_info "tinyply 安装完成: ${TINYPLY_DIR}"
}

# =============================================================================
# 主流程
# =============================================================================
main() {
    log_info "开始下载 AutoMap-Pro 依赖..."
    log_info "下载目录: ${DOWNLOAD_DIR}"
    log_info "安装目录: ${DEPS_DIR}"

    # 检查必要的工具
    for cmd in curl tar sha256sum unzip; do
        if ! command -v $cmd &> /dev/null; then
            log_error "缺少必要工具: $cmd"
            exit 1
        fi
    done

    # 下载 CMake
    download_cmake

    # 下载 ISPC
    download_ispc

    # 下载 Open3D 3rdparty（若存在 deps/Open3D）
    download_open3d_3rdparty || true

    # 下载 nanoflann（修复 Open3D 0.18.0 ExternalProject bug）
    download_nanoflann

    # 下载 tinyply（TEASER++ 依赖，避免 Docker 构建时从 GitHub 拉取超时）
    download_tinyply

    log_info "========================================="
    log_info "所有依赖下载完成!"
    log_info "========================================="
}

# 执行主流程
main "$@"
