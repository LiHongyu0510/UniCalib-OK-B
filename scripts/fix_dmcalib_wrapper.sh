#!/bin/bash
# 修复 .calib_scripts/dmcalib_python_wrapper.sh：必须使用 exec python3 "$@" 传参，
# 否则会出现 "can't find '__main__' module in '...'"（因 exec python3 "" 未传脚本路径）
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WRAPPER="${REPO_ROOT}/.calib_scripts/dmcalib_python_wrapper.sh"
if [[ ! -f "${WRAPPER}" ]]; then
  echo "[ERROR] 未找到: ${WRAPPER}"
  exit 1
fi
CONTENT='#!/bin/bash
# DM-Calib Python 包装：可选 NumPy 1.x 覆盖层，将参数原样传给 python3
# 若需 NumPy 1.x（如容器内为 NumPy 2.x），由调用方设置 DMCALIB_LIB 指向含 numpy<2 的目录
if [[ -n "${DMCALIB_LIB:-}" ]]; then
  export PYTHONPATH="${DMCALIB_LIB}:${PYTHONPATH:-}"
fi
exec python3 "$@"'
echo "[INFO] 写入 ${WRAPPER} (可能需要 sudo)"
if printf '%s\n' "$CONTENT" | sudo tee "${WRAPPER}" >/dev/null; then
  sudo chmod +x "${WRAPPER}"
  echo "[OK] 已修复 dmcalib_python_wrapper.sh (exec python3 \"\$@\")"
else
  echo "[ERROR] 写入失败，请手动将 exec python3 \"\" 改为 exec python3 \"\$@\""
  exit 1
fi
