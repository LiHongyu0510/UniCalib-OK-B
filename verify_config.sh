#!/bin/bash
# =============================================================================
# verify_config.sh - 验证UniCalib第三方工具配置
# 用法: ./verify_config.sh [--fix-auto]
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_ROOT="${SCRIPT_DIR}"

# 参数解析
AUTO_FIX=false
if [ "$1" == "--fix-auto" ]; then
    AUTO_FIX=true
fi

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
subsection() { echo -e "${CYAN}    └─ $1${NC}"; }

echo "=========================================="
echo "  UniCalib 配置验证工具"
echo "=========================================="
echo ""

# 配置文件路径
CONFIG_FILE="${CALIB_ROOT}/unicalib_C_plus_plus/config/sensors.yaml"
ENV_SETUP_FILE="${CALIB_ROOT}/env_setup.sh"

# 统计
PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

# ==============================================================================
# 第一步：检查配置文件
# =============================================================================

section "检查配置文件..."

if [ -f "${CONFIG_FILE}" ]; then
    success "配置文件存在: ${CONFIG_FILE}"
    ((PASS_COUNT++))
else
    error "配置文件不存在: ${CONFIG_FILE}"
    subsection "将创建示例配置..."
    mkdir -p "$(dirname "${CONFIG_FILE}")"
    cat > "${CONFIG_FILE}" << 'EOF'
# UniCalib 传感器配置示例
system:
  output_dir: "./calib_results"

# 第三方标定工具路径配置
# 建议：运行 ./auto_config_env.sh 自动检测和设置
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
    ((PASS_COUNT++))
fi

# ==============================================================================
# 第二步：检查环境变量
# =============================================================================

section "检查环境变量..."

check_env() {
    local var_name="$1"
    local tool_name="$2"
    local importance="${3:-optional}"
    
    if [ -n "${!var_name}" ]; then
        if [ -d "${!var_name}" ]; then
            success "${var_name}: ${!var_name}"
            ((PASS_COUNT++))
            return 0
        else
            warning "${var_name} 已设置但目录不存在: ${!var_name}"
            ((WARN_COUNT++))
            return 1
        fi
    else
        if [ "${importance}" == "required" ]; then
            error "${var_name} 未设置"
            ((FAIL_COUNT++))
            return 2
        else
            warning "${var_name} 未设置 (${importance})"
            ((WARN_COUNT++))
            return 1
        fi
    fi
}

check_env "UNICALIB_DM_CALIB" "DM-Calib" "required"
check_env "UNICALIB_LEARN_TO_CALIB" "learn-to-calibrate" "required"
check_env "UNICALIB_MIAS_LCEC" "MIAS-LCEC" "optional"
check_env "UNICALIB_IKALIBR" "iKalibr" "optional"
check_env "UNICALIB_CLICK_CALIB" "click_calib" "optional"
check_env "UNICALIB_TRANSFORMER_IMU" "Transformer-IMU" "optional"

# ==============================================================================
# 第三步：检查配置文件中的路径
# =============================================================================

section "检查配置文件中的路径..."

if [ -f "${CONFIG_FILE}" ]; then
    # 提取third_party路径
    extract_path() {
        local tool="$1"
        local path=$(grep -A 10 "third_party:" "${CONFIG_FILE}" | grep "${tool}:" | awk '{print $2}' | tr -d '"')
        echo "${path}"
    }
    
    check_config_path() {
        local tool="$1"
        local config_name="$2"
        local importance="${3:-optional}"
        
        local path=$(extract_path "${tool}")
        
        if [ -z "${path}" ] || [ "${path}" == '""' ]; then
            info "${config_name}: 未在配置文件中设置"
            return 1
        fi
        
        if [ -d "${path}" ]; then
            success "${config_name}: ${path}"
            ((PASS_COUNT++))
            return 0
        else
            warning "${config_name}: 路径不存在 - ${path}"
            ((WARN_COUNT++))
            return 1
        fi
    }
    
    check_config_path "dm_calib" "third_party.dm_calib" "required"
    check_config_path "learn_to_calibrate" "third_party.learn_to_calibrate" "required"
    check_config_path "mias_lcec" "third_party.mias_lcec" "optional"
    check_config_path "ikalibr" "third_party.ikalibr" "optional"
    check_config_path "click_calib" "third_party.click_calib" "optional"
    check_config_path "transformer_imu" "third_party.transformer_imu" "optional"
else
    warning "配置文件不存在，跳过路径检查"
fi

# ==============================================================================
# 第四步：自动检测工具路径
# =============================================================================

section "自动检测工具路径..."

declare -A AUTO_DETECTED

# 检测DM-Calib
DM_PATHS=(
    "${CALIB_ROOT}/DM-Calib"
    "${CALIB_ROOT}/src/DM-Calib"
    "/opt/DM-Calib"
)
for path in "${DM_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/DMCalib/tools/infer.py" ]; then
        AUTO_DETECTED[dm_calib]="${path}"
        success "DM-Calib: ${path}"
        ((PASS_COUNT++))
        break
    fi
done

# 检测learn-to-calibrate
L2C_PATHS=(
    "${CALIB_ROOT}/learn-to-calibrate"
    "${CALIB_ROOT}/src/learn-to-calibrate"
    "/opt/learn-to-calibrate"
)
for path in "${L2C_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/rl_solver/calib_rl.py" ]; then
        AUTO_DETECTED[learn_to_calibrate]="${path}"
        success "learn-to-calibrate: ${path}"
        ((PASS_COUNT++))
        break
    fi
done

# 检测MIAS-LCEC
MIAS_PATHS=(
    "${CALIB_ROOT}/MIAS-LCEC"
    "${CALIB_ROOT}/src/MIAS-LCEC"
    "/opt/MIAS-LCEC"
)
for path in "${MIAS_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/model/pretrained_overlap_transformer.pth.tar" ]; then
        AUTO_DETECTED[mias_lcec]="${path}"
        success "MIAS-LCEC: ${path}"
        ((PASS_COUNT++))
        break
    fi
done

# 检测Transformer-IMU
TIC_PATHS=(
    "${CALIB_ROOT}/Transformer-IMU-Calibrator"
    "${CALIB_ROOT}/src/Transformer-IMU-Calibrator"
    "/opt/Transformer-IMU-Calibrator"
)
for path in "${TIC_PATHS[@]}"; do
    if [ -d "${path}" ] && [ -f "${path}/checkpoint/TIC_13.pth" ]; then
        AUTO_DETECTED[transformer_imu]="${path}"
        success "Transformer-IMU-Calibrator: ${path}"
        ((PASS_COUNT++))
        break
    fi
done

# ==============================================================================
# 第五步：生成配置建议
# =============================================================================

echo ""
section "配置建议..."

MISSING_REQUIRED=false

# 检查必需工具
if [ -z "${UNICALIB_DM_CALIB}" ] && [ -z "${AUTO_DETECTED[dm_calib]}" ]; then
    subsection "❌ DM-Calib 未配置（必需）"
    echo "        建议："
    echo "          方式1（环境变量）:"
    echo "            export UNICALIB_DM_CALIB=/path/to/DM-Calib"
    echo "          方式2（配置文件）:"
    echo "            在 ${CONFIG_FILE} 中添加:"
    echo "              third_party:"
    echo "                dm_calib: \"/path/to/DM-Calib\""
    MISSING_REQUIRED=true
fi

if [ -z "${UNICALIB_LEARN_TO_CALIB}" ] && [ -z "${AUTO_DETECTED[learn_to_calibrate]}" ]; then
    subsection "❌ learn-to-calibrate 未配置（必需）"
    echo "        建议："
    echo "          方式1（环境变量）:"
    echo "            export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate"
    echo "          方式2（配置文件）:"
    echo "            在 ${CONFIG_FILE} 中添加:"
    echo "              third_party:"
    echo "                learn_to_calibrate: \"/path/to/learn-to-calibrate\""
    MISSING_REQUIRED=true
fi

if [ -z "${UNICALIB_MIAS_LCEC}" ] && [ -z "${AUTO_DETECTED[mias_lcec]}" ]; then
    subsection "⚠️  MIAS-LCEC 未配置（可选）"
    echo "        说明：MIAS-LCEC用于LiDAR-Camera外参标定"
    echo "        建议："
    echo "          方式1（环境变量）:"
    echo "            export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC"
    echo "          方式2（配置文件）:"
    echo "            在 ${CONFIG_FILE} 中添加:"
    echo "              third_party:"
    echo "                mias_lcec: \"/path/to/MIAS-LCEC\""
fi

if [ -z "${UNICALIB_IKALIBR}" ]; then
    subsection "ℹ️  iKalibr 未配置（可选）"
    echo "        说明：iKalibr用于多传感器联合优化，需要ROS2环境"
fi

if [ "${MISSING_REQUIRED}" = true ]; then
    echo ""
    subsection "🚀 快速配置（推荐）:"
    echo "        运行自动配置脚本:"
    echo "          ./auto_config_env.sh"
    echo ""
    echo "        这将："
    echo "          • 自动检测所有工具路径"
    echo "          • 生成环境变量脚本: ${ENV_SETUP_FILE}"
    echo "          • 更新配置文件: ${CONFIG_FILE}"
    echo "          • 导出环境变量到当前shell"
fi

# ==============================================================================
# 第六步：自动修复（如果请求）
# =============================================================================

if [ "${AUTO_FIX}" = true ]; then
    section "应用自动修复..."
    
    # 调用自动配置脚本
    if [ -f "${CALIB_ROOT}/auto_config_env.sh" ]; then
        info "运行 auto_config_env.sh..."
        bash "${CALIB_ROOT}/auto_config_env.sh"
        success "自动配置完成"
    else
        error "auto_config_env.sh 不存在"
    fi
fi

# ==============================================================================
# 总结
# =============================================================================

echo ""
echo "=========================================="
echo "  验证结果汇总"
echo "=========================================="
echo ""
echo -e "  通过: ${GREEN}${PASS_COUNT}${NC}"
echo -e "  警告: ${YELLOW}${WARN_COUNT}${NC}"
echo -e "  失败: ${RED}${FAIL_COUNT}${NC}"
echo ""

# 生成配置示例
CONFIG_EXAMPLE="# UniCalib 配置建议\n"
CONFIG_EXAMPLE="# 复制以下内容到 ${CONFIG_FILE}\n"
CONFIG_EXAMPLE="# 或设置为环境变量\n\n"
CONFIG_EXAMPLE="third_party:\n"

for tool in dm_calib learn_to_calibrate mias_lcec ikalibr click_calib; do
    if [ -n "${AUTO_DETECTED[$tool]}" ]; then
        CONFIG_EXAMPLE="  ${tool}: \"${AUTO_DETECTED[$tool]}\"  # 自动检测\n"
    elif [ -n "${!UNICALIB_$(echo $tool | tr '[:lower:]' '[:upper:]')}" ]; then
        var_name="UNICALIB_$(echo $tool | tr '[:lower:]' '[:upper:]')"
        CONFIG_EXAMPLE="  ${tool}: \"${!var_name}\"  # 当前环境变量\n"
    else
        CONFIG_EXAMPLE="  # ${tool}: \"/path/to/${tool}\"  # 需要设置\n"
    fi
done

echo "=========================================="
echo "  配置示例"
echo "=========================================="
echo ""
echo -e "$CONFIG_EXAMPLE"

# 输出建议
echo ""
echo "=========================================="
echo "  下一步操作"
echo "=========================================="
echo ""

if [ ${FAIL_COUNT} -gt 0 ] || [ ${WARN_COUNT} -gt 0 ]; then
    echo "发现配置问题，建议按以下步骤修复："
    echo ""
    echo "  1️⃣  快速修复（推荐）:"
    echo "     ./auto_config_env.sh"
    echo ""
    echo "  2️⃣  手动修复:"
    echo "     方式A: 设置环境变量"
    echo "       export UNICALIB_DM_CALIB=/path/to/DM-Calib"
    echo "       export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate"
    echo "       export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC"
    echo ""
    echo "     方式B: 编辑配置文件"
    echo "       编辑 ${CONFIG_FILE}"
    echo "       取消 third_party 下的注释并设置路径"
    echo ""
    echo "  3️⃣  持久化环境变量:"
    echo "     source ${ENV_SETUP_FILE}  # 临时生效"
    echo "     echo 'source ${ENV_SETUP_FILE}' >> ~/.bashrc  # 永久生效"
    echo ""
    echo "  4️⃣  重新验证:"
    echo "     ./verify_config.sh"
    echo ""
else
    echo -e "${GREEN}✓ 所有配置检查通过！${NC}"
    echo ""
    echo "可以开始运行标定流程:"
    echo "  ./build_and_run.sh"
    echo ""
    echo "或手动运行:"
    echo "  cd unicalib_C_plus_plus/build"
    echo "  RUN_PIPELINE=1 ./unicalib_example ../config/sensors.yaml /path/to/data"
    echo ""
fi

# 返回码
if [ ${FAIL_COUNT} -gt 0 ]; then
    exit 1
else
    exit 0
fi
