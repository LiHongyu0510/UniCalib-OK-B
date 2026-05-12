#!/bin/bash
# =============================================================================
# auto_config_env.sh - 自动检测并配置UniCalib第三方工具路径
# 用法: source auto_config_env.sh  或  ./auto_config_env.sh > env_setup.sh && source env_setup.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_ROOT="${SCRIPT_DIR}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

success() { echo -e "${GREEN}[PASS]${NC} $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[FAIL]${NC} $1"; }
info() { echo -e "${BLUE}[INFO]${NC} $1"; }
section() { echo -e "${CYAN}>>> $1${NC}"; }

echo "=========================================="
echo "  UniCalib 环境自动配置"
echo "=========================================="
echo ""

# 检测到的工具路径
declare -A DETECTED_PATHS

# 配置文件路径
CONFIG_FILE="${CALIB_ROOT}/unicalib_C_plus_plus/config/sensors.yaml"
BASHRC="${HOME}/.bashrc"

# ==============================================================================
# 自动检测工具路径
# =============================================================================

section "检测第三方工具..."

# 1. DM-Calib
DM_CALIB_PATHS=(
    "${CALIB_ROOT}/DM-Calib"
    "${CALIB_ROOT}/src/DM-Calib"
    "/opt/DM-Calib"
    "${HOME}/DM-Calib"
)
for path in "${DM_CALIB_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/DMCalib/tools/infer.py" ]; then
        DETECTED_PATHS[dm_calib]="${path}"
        success "DM-Calib: ${path}"
        break
    fi
done

# 2. learn-to-calibrate (L2Calib)
L2C_PATHS=(
    "${CALIB_ROOT}/learn-to-calibrate"
    "${CALIB_ROOT}/src/learn-to-calibrate"
    "${CALIB_ROOT}/L2Calib"
    "${CALIB_ROOT}/src/L2Calib"
    "/opt/learn-to-calibrate"
    "/opt/L2Calib"
    "${HOME}/learn-to-calibrate"
    "${HOME}/L2Calib"
)
for path in "${L2C_PATHS[@]}"; do
    # 检查多种可能的标识文件
    if [ -d "${path}" ] && [ -f "${path}/demo/calib.sh" ]; then
        DETECTED_PATHS[learn_to_calibrate]="${path}"
        success "learn-to-calibrate: ${path}"
        break
    fi
done

# 3. MIAS-LCEC
MIAS_PATHS=(
    "${CALIB_ROOT}/MIAS-LCEC"
    "${CALIB_ROOT}/src/MIAS-LCEC"
    "/opt/MIAS-LCEC"
    "${HOME}/MIAS-LCEC"
)
for path in "${MIAS_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/model/pretrained_overlap_transformer.pth.tar" ]; then
        DETECTED_PATHS[mias_lcec]="${path}"
        success "MIAS-LCEC: ${path}"
        break
    fi
done

# 4. Transformer-IMU-Calibrator
TIC_PATHS=(
    "${CALIB_ROOT}/Transformer-IMU-Calibrator"
    "${CALIB_ROOT}/src/Transformer-IMU-Calibrator"
    "/opt/Transformer-IMU-Calibrator"
    "${HOME}/Transformer-IMU-Calibrator"
)
for path in "${TIC_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/checkpoint/TIC_13.pth" ]; then
        DETECTED_PATHS[transformer_imu]="${path}"
        success "Transformer-IMU-Calibrator: ${path}"
        break
    fi
done

# 5. iKalibr (ROS2包)
IKALIBR_PATHS=(
    "${CALIB_ROOT}/iKalibr"
    "${CALIB_ROOT}/src/iKalibr"
    "/opt/ros/*/share/ikalibr"
    "${HOME}/iKalibr"
)
for path in "${IKALIBR_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/package.xml" ]; then
        DETECTED_PATHS[ikalibr]="${path}"
        success "iKalibr: ${path}"
        break
    fi
done

# 6. click_calib
CLICK_PATHS=(
    "${CALIB_ROOT}/click_calib"
    "${CALIB_ROOT}/src/click_calib"
    "/opt/click_calib"
    "${HOME}/click_calib"
)
for path in "${CLICK_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/source/optimize.py" ]; then
        DETECTED_PATHS[click_calib]="${path}"
        success "click_calib: ${path}"
        break
    fi
done

echo ""
section "检测结果汇总"
for tool in dm_calib learn_to_calibrate mias_lcec transformer_imu ikalibr click_calib; do
    if [ -n "${DETECTED_PATHS[$tool]}" ]; then
        success "${tool}: ${DETECTED_PATHS[$tool]}"
    else
        warning "${tool}: 未找到"
    fi
done

# ==============================================================================
# 生成环境变量配置
# =============================================================================

echo ""
section "生成环境变量配置..."

ENV_OUTPUT=""
ENV_COMMENT="# UniCalib 环境变量配置 (由 auto_config_env.sh 生成)"
ENV_COMMENT="${ENV_COMMENT}\n# 用法: source auto_config_env.sh 或 source env_setup.sh"
ENV_COMMENT="${ENV_COMMENT}\n# 生成时间: $(date)"

for tool in dm_calib learn_to_calibrate mias_lcec transformer_imu ikalibr click_calib; do
    var_name="UNICALIB_$(echo $tool | tr '[:lower:]' '[:upper:]')"
    if [ -n "${DETECTED_PATHS[$tool]}" ]; then
        ENV_OUTPUT="${ENV_OUTPUT}\nexport ${var_name}=\"${DETECTED_PATHS[$tool]}\""
    else
        ENV_OUTPUT="${ENV_OUTPUT}\n# export ${var_name}=\"\"  # 未找到，请手动设置"
    fi
done

echo -e "$ENV_OUTPUT"
echo ""

# ==============================================================================
# 更新配置文件
# =============================================================================

section "更新配置文件: ${CONFIG_FILE}"

# 检查配置文件是否存在
if [ ! -f "${CONFIG_FILE}" ]; then
    warning "配置文件不存在: ${CONFIG_FILE}"
    info "将创建示例配置..."
    mkdir -p "$(dirname "${CONFIG_FILE}")"
    cat > "${CONFIG_FILE}" << 'EOF'
# UniCalib 传感器配置示例
system:
  output_dir: "./calib_results"

# 可选：第三方标定工具根目录（未配置则用环境变量 UNICALIB_*）
# 建议：使用 ./auto_config_env.sh 自动检测和设置
third_party:
  # dm_calib: ""              # 针孔内参 (DM-Calib)
  # learn_to_calibrate: ""     # IMU-LiDAR粗外参 (learn-to-calibrate)
  # mias_lcec: ""             # LiDAR-Camera外参 (MIAS-LCEC)
  # ikalibr: ""               # 联合优化 (iKalibr)
  # click_calib: ""           # Camera-Camera BA (click_calib)
  # transformer_imu: ""       # IMU内参备选 (Transformer-IMU)

sensors:
  - sensor_id: cam_front
    sensor_type: camera_pinhole
    topic: /camera/image_raw
    frame_id: cam_front_optical
    resolution: [1920, 1080]

  - sensor_id: imu
    sensor_type: imu
    topic: /imu/data
    frame_id: imu_link
    rate: 200

  - sensor_id: lidar
    sensor_type: lidar
    topic: /lidar/points
    frame_id: lidar_link
EOF
    success "已创建示例配置: ${CONFIG_FILE}"
fi

# 尝试更新配置文件中的third_party部分
if [ -f "${CONFIG_FILE}" ]; then
    # 创建临时文件
    TEMP_CONFIG=$(mktemp)
    
    # 使用sed直接更新配置文件（更简单可靠）
    info "更新配置文件中的 third_party 路径..."
    
    for tool in dm_calib learn_to_calibrate mias_lcec transformer_imu ikalibr click_calib; do
        if [ -n "${DETECTED_PATHS[$tool]}" ]; then
            var_name="UNICALIB_$(echo $tool | tr '[:lower:]' '[:upper:]')"
            path="${DETECTED_PATHS[$tool]}"
            
            # 使用sed更新或添加配置
            if grep -q "^  ${tool}:" "${CONFIG_FILE}" 2>/dev/null; then
                # 更新现有配置
                sed -i "s|^  ${tool}:.*|  ${tool}: \"${path}\"  # auto-detected|" "${CONFIG_FILE}"
            elif grep -q "^  # ${tool}:" "${CONFIG_FILE}" 2>/dev/null; then
                # 取消注释并更新
                sed -i "s|^  # ${tool}:.*|  ${tool}: \"${path}\"  # auto-detected|" "${CONFIG_FILE}"
            else
                # 添加新配置
                sed -i "/^third_party:/a\\  ${tool}: \"${path}\"  # auto-detected" "${CONFIG_FILE}"
            fi
            success "已更新 ${tool}: ${path}"
        fi
    done
    
    success "配置文件已更新: ${CONFIG_FILE}"
    info "检测到的路径已写入 third_party 部分"
fi

# ==============================================================================
# 生成shell脚本（可选）
# =============================================================================

ENV_SETUP_FILE="${CALIB_ROOT}/env_setup.sh"

echo "#!/bin/bash" > "${ENV_SETUP_FILE}"
echo "# UniCalib 环境变量设置脚本" >> "${ENV_SETUP_FILE}"
echo "# 由 auto_config_env.sh 生成 - $(date)" >> "${ENV_SETUP_FILE}"
echo "" >> "${ENV_SETUP_FILE}"
echo -e "$ENV_COMMENT" >> "${ENV_SETUP_FILE}"
echo "" >> "${ENV_SETUP_FILE}"
echo -e "$ENV_OUTPUT" >> "${ENV_SETUP_FILE}"

chmod +x "${ENV_SETUP_FILE}"
success "环境变量脚本已生成: ${ENV_SETUP_FILE}"
info "使用方法: source ${ENV_SETUP_FILE}"

# ==============================================================================
# 总结
# =============================================================================

echo ""
echo "=========================================="
echo "  配置完成"
echo "=========================================="
echo ""
info "检测到的工具数: ${#DETECTED_PATHS[@]}"
echo ""
info "下一步操作："
echo "  1. 直接使用（环境变量已生效）:"
echo "     ./build_and_run.sh"
echo ""
echo "  2. 或持久化环境变量:"
echo "     source ${ENV_SETUP_FILE}  # 临时生效"
echo "     echo 'source ${ENV_SETUP_FILE}' >> ~/.bashrc  # 永久生效"
echo ""
echo "  3. 或使用配置文件（已自动更新）:"
echo "     第三方路径已写入: ${CONFIG_FILE}"
echo ""
info "如需重新检测，再次运行: source auto_config_env.sh"
echo ""

# 如果脚本被source，导出环境变量
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    info "正在导出环境变量..."
    for tool in dm_calib learn_to_calibrate mias_lcec transformer_imu ikalibr click_calib; do
        if [ -n "${DETECTED_PATHS[$tool]}" ]; then
            var_name="UNICALIB_$(echo $tool | tr '[:lower:]' '[:upper:]')"
            export ${var_name}="${DETECTED_PATHS[$tool]}"
            success "export ${var_name}=\"${DETECTED_PATHS[$tool]}\""
        fi
    done
fi
