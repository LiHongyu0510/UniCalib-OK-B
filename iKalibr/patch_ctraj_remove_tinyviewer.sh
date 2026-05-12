#!/usr/bin/env bash
# 从已安装的 ctraj 头文件中移除对 tiny-viewer 的依赖，使 iKalibr 在无 tiny-viewer 环境下可编译。
# 优先用已修补的源码覆盖安装头文件；若不可写则尝试 sed 原地替换。
# 使用场景：thirdparty-install 已存在时，在 colcon 编译前运行本脚本（build_external_tools.sh 会自动调用）。
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${SCRIPT_DIR}/thirdparty-install/ctraj-install"
SRC_DIR="${SCRIPT_DIR}/thirdparty/ctraj/src/include/ctraj"
if [ ! -d "${INSTALL_DIR}/include" ]; then
    echo "错误: 未找到 ctraj 安装目录 ${INSTALL_DIR}"
    exit 1
fi

# 确保安装目录可写（Docker 挂载时可能只读）
chmod -R u+w "${INSTALL_DIR}" 2>/dev/null || true

# 需要移除 tiny-viewer 的头文件列表
FILES=(
    "utils/eigen_utils.hpp"
    "utils/sophus_utils.hpp"
    "core/pose.hpp"
    "factor/marginalization_factor.h"
)

PATCHED=0
# 优先：用已修补的源码覆盖安装头文件
if [ -d "${SRC_DIR}" ]; then
    for rel in "${FILES[@]}"; do
        src="${SRC_DIR}/${rel}"
        dst="${INSTALL_DIR}/include/ctraj/${rel}"
        if [ -f "${src}" ] && [ -d "$(dirname "${dst}")" ]; then
            need_copy=0
            grep -q 'tiny-viewer/entity/utils.h' "${dst}" 2>/dev/null && need_copy=1
            grep -q 'tiny-viewer removed:.*tiny-viewer removed' "${dst}" 2>/dev/null && need_copy=1
            if [ "${need_copy}" = "1" ]; then
                cp -f "${src}" "${dst}"
                echo "已用源码覆盖: ${rel}"
                PATCHED=1
            fi
        fi
    done
fi

# 若未覆盖成功，则 sed 原地替换
for rel in "${FILES[@]}"; do
    f="${INSTALL_DIR}/include/ctraj/${rel}"
    if [ -f "${f}" ] && grep -q 'tiny-viewer/entity/utils.h' "${f}"; then
        sed -i.bak 's|#include "tiny-viewer/entity/utils.h"|// tiny-viewer removed|g' "${f}" 2>/dev/null && rm -f "${f}.bak" 2>/dev/null
        echo "已修补: ${rel}"
        PATCHED=1
    fi
done

# 修复曾被错误替换产生的嵌套注释（eigen_utils.hpp）
eigen_h="${INSTALL_DIR}/include/ctraj/utils/eigen_utils.hpp"
if [ -f "${eigen_h}" ] && grep -q 'tiny-viewer removed:.*tiny-viewer removed' "${eigen_h}" 2>/dev/null; then
    sed -i.bak 's|/\* tiny-viewer removed: /\* tiny-viewer removed \*/ not needed for this header \*/|// tiny-viewer removed|g' "${eigen_h}" 2>/dev/null
    sed -i.bak 's|/\* tiny-viewer removed \*/|// tiny-viewer removed|g' "${eigen_h}" 2>/dev/null
    rm -f "${eigen_h}.bak" 2>/dev/null
    echo "已修复 eigen_utils.hpp 嵌套注释"
    PATCHED=1
fi

if [ "${PATCHED}" = "1" ]; then
    echo "完成: ctraj 头文件已移除 tiny-viewer 依赖。"
else
    echo "跳过: 未发现需修补的 tiny-viewer 引用。"
fi
