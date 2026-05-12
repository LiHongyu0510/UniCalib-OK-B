#!/bin/bash
# setup_models.sh - 修复所有工程的模型加载路径
# 使用方法: ./setup_models.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIB_ROOT="${SCRIPT_DIR}"

echo "=========================================="
echo "  多方标定工具模型加载路径配置"
echo "=========================================="
echo ""
echo "项目根目录: ${CALIB_ROOT}"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

success() { echo -e "${GREEN}✓${NC} $1"; }
warning() { echo -e "${YELLOW}⚠${NC} $1"; }
error() { echo -e "${RED}✗${NC} $1"; }

# 1. Transformer-IMU-Calibrator
echo "[1/4] 配置 Transformer-IMU-Calibrator..."
TIC_ROOT="${CALIB_ROOT}/Transformer-IMU-Calibrator"
if [ -d "${TIC_ROOT}" ]; then
    mkdir -p "${TIC_ROOT}/checkpoint"
    if [ -f "${TIC_ROOT}/model/TIC_13.pth" ]; then
        if [ -L "${TIC_ROOT}/checkpoint/TIC_13.pth" ] || [ -f "${TIC_ROOT}/checkpoint/TIC_13.pth" ]; then
            success "checkpoint/TIC_13.pth 已存在"
        else
            ln -sf ../model/TIC_13.pth "${TIC_ROOT}/checkpoint/TIC_13.pth"
            success "创建软链接 checkpoint/TIC_13.pth -> model/TIC_13.pth"
        fi
    else
        error "模型文件缺失: ${TIC_ROOT}/model/TIC_13.pth"
    fi
else
    warning "Transformer-IMU-Calibrator 目录不存在，跳过"
fi

# 2. MIAS-LCEC
echo ""
echo "[2/4] 检查 MIAS-LCEC..."
MIAS_ROOT="${CALIB_ROOT}/MIAS-LCEC"
if [ -d "${MIAS_ROOT}" ]; then
    if [ -f "${MIAS_ROOT}/model/pretrained_overlap_transformer.pth.tar" ]; then
        SIZE=$(stat -c%s "${MIAS_ROOT}/model/pretrained_overlap_transformer.pth.tar" 2>/dev/null || stat -f%z "${MIAS_ROOT}/model/pretrained_overlap_transformer.pth.tar")
        SIZE_MB=$((SIZE / 1024 / 1024))
        success "模型文件存在: pretrained_overlap_transformer.pth.tar (${SIZE_MB} MB)"
    else
        error "模型文件缺失: ${MIAS_ROOT}/model/pretrained_overlap_transformer.pth.tar"
    fi
    
    # 检查 C++ 可执行文件
    if [ -x "${MIAS_ROOT}/bin/mias_lcec" ]; then
        success "可执行文件存在: bin/mias_lcec"
    else
        warning "C++ 可执行文件未编译，需要运行编译"
    fi
else
    warning "MIAS-LCEC 目录不存在，跳过"
fi

# 3. learn-to-calibrate
echo ""
echo "[3/4] 检查 learn-to-calibrate..."
L2C_ROOT="${CALIB_ROOT}/learn-to-calibrate"
if [ -d "${L2C_ROOT}" ]; then
    PRECOMP="${L2C_ROOT}/rl_solver/bingham/precomputed"
    if [ -f "${PRECOMP}/norm_constant_model_-500_0_200.model" ]; then
        success "Bingham 模型文件存在: norm_constant_model_-500_0_200.model"
    else
        error "Bingham 模型文件缺失"
    fi
    if [ -f "${PRECOMP}/b_model_-500_0_200.model" ]; then
        success "Bingham 模型文件存在: b_model_-500_0_200.model"
    else
        error "Bingham 模型文件缺失"
    fi
else
    warning "learn-to-calibrate 目录不存在，跳过"
fi

# 4. DM-Calib
echo ""
echo "[4/4] 检查 DM-Calib..."
DM_ROOT="${CALIB_ROOT}/DM-Calib"
if [ -d "${DM_ROOT}" ]; then
    CALIB_MODEL="${DM_ROOT}/model/calib/diffusion_pytorch_model.safetensors"
    DEPTH_MODEL="${DM_ROOT}/model/depth_safe/diffusion_pytorch_model.safetensors"
    VAE_MODEL="${DM_ROOT}/model/depth_safe/vae_decoder.safetensors"
    
    if [ -f "${CALIB_MODEL}" ]; then
        SIZE=$(stat -c%s "${CALIB_MODEL}" 2>/dev/null || stat -f%z "${CALIB_MODEL}")
        SIZE_GB=$(echo "scale=1; ${SIZE} / 1024 / 1024 / 1024" | bc)
        success "标定模型存在: calib/diffusion_pytorch_model.safetensors (${SIZE_GB} GB)"
    else
        error "标定模型缺失"
    fi
    
    if [ -f "${DEPTH_MODEL}" ]; then
        SIZE=$(stat -c%s "${DEPTH_MODEL}" 2>/dev/null || stat -f%z "${DEPTH_MODEL}")
        SIZE_GB=$(echo "scale=1; ${SIZE} / 1024 / 1024 / 1024" | bc)
        success "深度模型存在: depth_safe/diffusion_pytorch_model.safetensors (${SIZE_GB} GB)"
    else
        warning "深度模型缺失 (可选)"
    fi
    
    if [ -f "${VAE_MODEL}" ]; then
        success "VAE 解码器存在: vae_decoder.safetensors"
    else
        warning "VAE 解码器缺失 (可选)"
    fi
else
    warning "DM-Calib 目录不存在，跳过"
fi

echo ""
echo "=========================================="
echo "  模型配置完成"
echo "=========================================="
echo ""
echo "下一步操作："
echo "  1. 运行验证脚本确认模型加载正常:"
echo "     ./verify_models.sh"
echo ""
echo "  2. 配置 sensors.yaml 中的第三方工具路径:"
echo "     编辑 unicalib_C_plus_plus/config/sensors.yaml"
echo ""
