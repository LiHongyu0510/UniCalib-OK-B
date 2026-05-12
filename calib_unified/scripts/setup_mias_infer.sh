#!/usr/bin/env bash
# 将 infer_unicalib 模板复制到 MIAS-LCEC 的 bin/python/ 目录，供粗标定管线 headless 调用。
# 用法: ./setup_mias_infer.sh <MIAS_LCEC_ROOT>
# 示例: ./setup_mias_infer.sh /path/to/ai_models/MIAS-LCEC
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/infer_unicalib_template_for_MIAS_LCEC.py"
if [[ $# -lt 1 ]]; then
  echo "用法: $0 <MIAS_LCEC_ROOT>" >&2
  echo "示例: $0 /path/to/MIAS-LCEC  # 将复制模板到 MIAS_LCEC_ROOT/bin/python/infer_unicalib.py" >&2
  exit 1
fi
MIAS_ROOT="$1"
DEST_DIR="${MIAS_ROOT}/bin/python"
DEST_FILE="${DEST_DIR}/infer_unicalib.py"
if [[ ! -f "$TEMPLATE" ]]; then
  echo "错误: 模板不存在: $TEMPLATE" >&2
  exit 1
fi
mkdir -p "$DEST_DIR"
cp "$TEMPLATE" "$DEST_FILE"
echo "已复制: $TEMPLATE -> $DEST_FILE"
echo "若 MIAS-LCEC 使用 Python 3.10 虚拟环境，请将 third_party.mias_lcec.python_exe 或 UNICALIB_MIAS_LCEC_PYTHON 指向该环境解释器。"
