#!/bin/bash
# 临时补丁脚本：修改 build_external_tools.sh 中的 build_ikalibr 函数
# 用于跳过 iKalibr 编译（ROS1 vs ROS2 不兼容）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_SCRIPT="${SCRIPT_DIR}/build_external_tools.sh"

echo "=========================================="
echo "应用 iKalibr 补丁"
echo "=========================================="
echo ""

# 备份原文件
cp "${BUILD_SCRIPT}" "${BUILD_SCRIPT}.backup"

# 查找 build_ikalibr 函数并添加跳过逻辑
sed -i '/build_ikalibr()/,/build_ikalibr() {/a\
    # 快速跳过：iKalibr 使用 ROS1 (catkin)，不兼容 ROS2 Humble (ament)\
    if [ "${SKIP_IKALIBR:-0}" = "1" ]; then\
        warning "跳过 iKalibr 编译（SKIP_IKALIBR=1）"\
        return 0\
    fi\
' "${BUILD_SCRIPT}"

echo ""
echo "✅ 补丁已应用"
echo ""
echo "使用方法："
echo "  bash build_and_run.sh  # 默认会询问是否跳过"
echo "  SKIP_IKALIBR=1 bash build_and_run.sh  # 直接跳过"
echo ""
echo "恢复原文件："
echo "  mv ${BUILD_SCRIPT}.backup ${BUILD_SCRIPT}"
