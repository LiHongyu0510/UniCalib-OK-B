#!/bin/bash
# =============================================================================
# 多标定项目 Docker 依赖预下载脚本（可选）
# 用途: 预下载需网络访问的依赖，便于离线/弱网环境构建镜像
#
# 本镜像主要使用 docker/deps 内已有源码（OpenCV/Ceres/GTSAM/Sophus/magic_enum），
# 若需升级 CMake 或预下载 PyTorch wheel 等，可在此扩展。
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 脚本在 docker/scripts，deps 在 docker/deps
DEPS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/deps"
DOWNLOAD_DIR="${DEPS_DIR}/downloads"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

mkdir -p "${DOWNLOAD_DIR}"
log_info "下载目录: ${DOWNLOAD_DIR}"
log_info "本镜像主要依赖已包含在 docker/deps 中（opencv/ceres-solver/gtsam/Sophus/magic_enum）"
log_info "当前脚本为占位，可按需添加 CMake/PyTorch wheel 等预下载逻辑。"
log_info "完成（无额外下载项）。"
