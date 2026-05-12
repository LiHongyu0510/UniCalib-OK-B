#!/usr/bin/env bash
#
# 下载 MobileSAM 预训练权重 (mobile_sam.pt, ~40MB)
# 用于纯 PyTorch 粗标定 (mias_lcec_pytorch) 的图像分割分支。
#
# 用法:
#   ./download_mobile_sam.sh [目标目录]
#   默认目标: 当前目录/weights/mobile_sam.pt 或 MIAS-LCEC/bin/MobileSAM/weights/
#

set -e

# 官方权重 URL (GitHub raw; 若为 LFS 则需用下方 clone 方式)
MOBILESAM_RAW_URL="https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt"

# 目标目录: 第一个参数，或脚本所在目录的 weights，或 MIAS-LCEC 下
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_DIR="${SCRIPT_DIR}/weights"
DEFAULT_MIAS_DIR=""
if [ -d "${SCRIPT_DIR}/../../MIAS-LCEC/bin/MobileSAM/weights" ]; then
    DEFAULT_MIAS_DIR="${SCRIPT_DIR}/../../MIAS-LCEC/bin/MobileSAM/weights"
fi

TARGET_DIR="${1:-$DEFAULT_DIR}"
mkdir -p "$TARGET_DIR"
TARGET_FILE="${TARGET_DIR}/mobile_sam.pt"

echo "[MobileSAM] 目标文件: $TARGET_FILE"

if [ -f "$TARGET_FILE" ]; then
    echo "[MobileSAM] 已存在，跳过下载。删除该文件可重新下载。"
    echo "[MobileSAM] 使用: export MOBILESAM_CHECKPOINT=$TARGET_FILE"
    exit 0
fi

# 方式 1: wget
if command -v wget &>/dev/null; then
    echo "[MobileSAM] 使用 wget 下载..."
    if wget -q --show-progress -O "$TARGET_FILE" "$MOBILESAM_RAW_URL" 2>/dev/null; then
        if [ -s "$TARGET_FILE" ] && ! grep -q "version https://git-lfs" "$TARGET_FILE" 2>/dev/null; then
            echo "[MobileSAM] 下载成功: $TARGET_FILE"
            echo "  export MOBILESAM_CHECKPOINT=$TARGET_FILE"
            exit 0
        fi
        rm -f "$TARGET_FILE"
    fi
fi

# 方式 2: curl
if command -v curl &>/dev/null; then
    echo "[MobileSAM] 使用 curl 下载..."
    if curl -sSL -o "$TARGET_FILE" "$MOBILESAM_RAW_URL"; then
        if [ -s "$TARGET_FILE" ] && ! grep -q "version https://git-lfs" "$TARGET_FILE" 2>/dev/null; then
            echo "[MobileSAM] 下载成功: $TARGET_FILE"
            echo "  export MOBILESAM_CHECKPOINT=$TARGET_FILE"
            exit 0
        fi
        rm -f "$TARGET_FILE"
    fi
fi

# 方式 3: git clone 仓库后复制 (适合 LFS 或网络受限)
echo "[MobileSAM] 直接 URL 下载未成功，尝试 git clone 仓库..."
CLONE_DIR="${SCRIPT_DIR}/.MobileSAM_clone"
if [ ! -d "$CLONE_DIR" ]; then
    git clone --depth 1 https://github.com/ChaoningZhang/MobileSAM.git "$CLONE_DIR"
fi
if [ -d "$CLONE_DIR" ]; then
    (cd "$CLONE_DIR" && git lfs pull 2>/dev/null || true)
    if [ -f "$CLONE_DIR/weights/mobile_sam.pt" ]; then
        cp "$CLONE_DIR/weights/mobile_sam.pt" "$TARGET_FILE"
        echo "[MobileSAM] 从 clone 复制成功: $TARGET_FILE"
        echo "  export MOBILESAM_CHECKPOINT=$TARGET_FILE"
        rm -rf "$CLONE_DIR"
        exit 0
    fi
fi

echo "[MobileSAM] 自动下载失败，请手动操作:"
echo "  1) 打开 https://github.com/ChaoningZhang/MobileSAM"
echo "  2) 进入 weights/ 目录，下载 mobile_sam.pt"
echo "  3) 放到 $TARGET_FILE"
echo "  4) 或设置: export MOBILESAM_CHECKPOINT=/path/to/mobile_sam.pt"
exit 1
