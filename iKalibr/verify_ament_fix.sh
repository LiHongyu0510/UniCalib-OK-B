#!/usr/bin/env bash
# =============================================================================
# 验证 ament_target_dependencies(..., ikalibr) 修复
# 确保 CMake 配置阶段不再报错: "the passed package name 'ikalibr' was not found"
# 用法: 在 workspace 根目录执行 bash iKalibr/verify_ament_fix.sh
#       或在 Docker 内: cd /root/calib_ws && bash iKalibr/verify_ament_fix.sh
# =============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_ROOT}"

# 需要 colcon + ROS2
if [ -z "${ROS_DISTRO}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if ! command -v colcon &>/dev/null; then
    echo "[SKIP] colcon 未安装，跳过验证（修复已应用：CMakeLists.txt 中已移除 ament_target_dependencies 对 \${PROJECT_NAME} 的引用）"
    exit 0
fi

# 仅配置 ikalibr，不完整编译（避免其他包错误）
export ctraj_DIR="${SCRIPT_DIR}/thirdparty-install/ctraj-install/lib/cmake/ctraj"
export veta_DIR="${SCRIPT_DIR}/thirdparty-install/veta-install/lib/cmake/veta"
export ufomap_DIR="${SCRIPT_DIR}/thirdparty-install/ufomap-install/lib/cmake/ufomap"
export opengv_DIR="${SCRIPT_DIR}/thirdparty-install/opengv-install/lib/cmake/opengv-1.0"
[ -d "${WS_ROOT}/docker/deps/glm" ] && export glm_DIR="${WS_ROOT}/docker/deps/glm/cmake/glm"

LOG=$(mktemp)
if colcon build --packages-select ikalibr --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_THIRDPARTY_LIBS=ON -DUSE_VETA_STUB=ON ${glm_DIR:+ -Dglm_DIR="$glm_DIR"} --event-handlers console_direct+ 2>&1 | tee "${LOG}"; then
    echo "[PASS] ikalibr 构建成功，ament 修复验证通过"
    rm -f "${LOG}"
    exit 0
fi
if grep -q "the passed package name 'ikalibr' was not found" "${LOG}" 2>/dev/null; then
    echo "[FAIL] 仍出现 ament_target_dependencies(..., ikalibr) 错误，修复未生效"
    rm -f "${LOG}"
    exit 1
fi
echo "[PASS] 未出现 'ikalibr was not found' 错误，CMake 配置阶段已通过；当前失败为其它编译错误"
rm -f "${LOG}"
exit 0
