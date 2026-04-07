#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true

EXE="/root/calib_ws/calib_unified/build/bin/unicalib_cam_cam"
CONFIG="/root/calib_ws/calib_unified/config/unicalib_example.yaml"
TASK="cam-cam"

# cam-intrin/joint：从挂载的预下载 wheel 安装 PyTorch>=2.4 + diffusers/transformers（DM-Calib 需 transformers 与 PyTorch>=2.4）
WHEELS_DIR="/root/calib_ws/calib_unified/thirdparty/wheels"
if [[ "$TASK" == "cam-intrin" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]]; then
  if [[ -d "$WHEELS_DIR" ]]; then
    set +e
    HAS_TORCH_WHL=$(find "$WHEELS_DIR" -maxdepth 1 -name 'torch-2.*.whl' 2>/dev/null | head -1)
    # 1) 若 wheels 中有 PyTorch 2.x，先安装/升级为 >=2.4（避免 transformers 报 "PyTorch >= 2.4 is required"）
    if [[ -n "${HAS_TORCH_WHL}" ]]; then
      TORCH_VER=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
      NEED_TORCH=0
      [[ "$TORCH_VER" == "0" ]] && NEED_TORCH=1
      [[ "$TORCH_VER" == 2.0* ]] && NEED_TORCH=1
      [[ "$TORCH_VER" == 2.1* ]] && NEED_TORCH=1
      [[ "$TORCH_VER" == 2.2* ]] && NEED_TORCH=1
      [[ "$TORCH_VER" == 2.3* ]] && NEED_TORCH=1
      if [[ "${NEED_TORCH}" -eq 1 ]]; then
        echo "[INFO] 从预下载 wheel 安装 PyTorch>=2.4（${WHEELS_DIR}）..."
        if pip3 install --no-index --find-links "$WHEELS_DIR" --upgrade torch torchvision torchaudio 2>&1; then
          TORCH_VER=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "?")
          echo "[INFO] 安装后 PyTorch 版本: ${TORCH_VER}"
        else
          echo "[WARN] 从 wheel 安装 PyTorch 失败，DM-Calib 可能仍报 PyTorch 版本问题；可尝试在容器内执行: pip3 install --no-index --find-links ${WHEELS_DIR} --upgrade torch torchvision torchaudio"
        fi
      fi
    else
      TORCH_VER=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
      if [[ "$TORCH_VER" == 2.0* ]] || [[ "$TORCH_VER" == 2.1* ]] || [[ "$TORCH_VER" == 2.2* ]] || [[ "$TORCH_VER" == 2.3* ]] || [[ "$TORCH_VER" == "0" ]]; then
        echo "[WARN] 未找到预下载的 PyTorch 2.x wheel（${WHEELS_DIR}），DM-Calib 需要 PyTorch>=2.4；请先执行: ./calib_unified_run.sh --download-diffusers"
      fi
    fi
    # 2) 未装 diffusers 时安装 diffusers/transformers
    if ! python3 -c "import diffusers" 2>/dev/null; then
      if [[ -n "$(find "$WHEELS_DIR" -maxdepth 1 -name 'diffusers-*.whl' 2>/dev/null)" ]]; then
        echo "[INFO] 从预下载 wheel 为系统 Python 安装 diffusers/transformers（DM-Calib 用）..."
        pip3 install --no-index --find-links "$WHEELS_DIR" 'numpy<2' diffusers transformers 2>/dev/null ||           pip3 install --no-index --find-links "$WHEELS_DIR" diffusers transformers
      fi
    fi
    # 3) PyTorch<2.4 时强制安装与 2.1 兼容且满足 DM-Calib 的 diffusers（需 StableDiffusionMixin）+ transformers + huggingface_hub
    #    diffusers>=0.27 才有 StableDiffusionMixin；huggingface_hub<0.26 提供 cached_download（部分旧依赖用）
    TORCH_VER=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
    if [[ "$TORCH_VER" == 2.0* ]] || [[ "$TORCH_VER" == 2.1* ]] || [[ "$TORCH_VER" == 2.2* ]] || [[ "$TORCH_VER" == 2.3* ]]; then
      echo "[INFO] PyTorch=${TORCH_VER} <2.4，强制安装兼容组合: diffusers 0.27~0.29（含 StableDiffusionMixin）+ transformers 4.31.x + huggingface_hub<0.26（DM-Calib 用）..."
      pip3 install --no-index --find-links "$WHEELS_DIR" 'numpy<2' 'huggingface_hub>=0.17,<0.26' 'diffusers>=0.27,<0.30' 'transformers>=4.30,<4.36' 2>/dev/null ||         pip3 install --no-index --find-links "$WHEELS_DIR" 'huggingface_hub>=0.17,<0.26' 'diffusers>=0.27,<0.30' 'transformers>=4.30,<4.36' 2>/dev/null ||         pip3 install 'numpy<2' 'huggingface_hub>=0.17,<0.26' 'diffusers>=0.27,<0.30' 'transformers>=4.30,<4.36' || true
      TR_VER=$(python3 -c "import transformers; print(transformers.__version__)" 2>/dev/null || echo "?")
      DF_VER=$(python3 -c "import diffusers; print(diffusers.__version__)" 2>/dev/null || echo "?")
      HFH_VER=$(python3 -c "import huggingface_hub; print(huggingface_hub.__version__)" 2>/dev/null || echo "?")
      echo "[INFO] 当前 huggingface_hub=${HFH_VER}  diffusers=${DF_VER}  transformers=${TR_VER}"
    fi
    set -e
  fi
fi

# NumPy 1.x 覆盖层：cam-intrin（DM-Calib）与 lidar-cam（粗标定 cv2/PnP）在 NumPy 2.x 下均需 NumPy 1.x，与相机内参标定环境配置一致
NEED_NUMPY1_TASK=0
[[ "$TASK" == "cam-intrin" ]] || [[ "$TASK" == "lidar-cam" ]] || [[ "$TASK" == "cam-cam" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]] && NEED_NUMPY1_TASK=1
if [[ "${NEED_NUMPY1_TASK}" -eq 1 ]]; then
  NUMPY_VER=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "0")
  if [[ "${NUMPY_VER}" == 2.* ]]; then
    # 系统 NumPy 2.x：创建 NumPy 1.x 覆盖层 + 统一 wrapper（DM-Calib 与粗标定脚本共用，避免 cv2/OpenCV ABI 崩溃）
    DMCALIB_LIB="/root/calib_ws/results/.dmcalib_numpy1"
    if [[ ! -f "${DMCALIB_LIB}/numpy/__init__.py" ]]; then
      echo "[INFO] 为标定 Python 子进程准备 NumPy 1.x 覆盖层（系统 NumPy=${NUMPY_VER}），路径: ${DMCALIB_LIB}"
      mkdir -p "${DMCALIB_LIB}"
      NUMPY_WHL=""
      for w in "/root/calib_ws/calib_unified/thirdparty/wheels"/numpy-*.whl; do
        [[ -f "$w" ]] && NUMPY_WHL="$w" && break
      done
      if [[ -n "${NUMPY_WHL}" ]]; then
        echo "[INFO] 从本地 wheel 安装 NumPy 1.x: ${NUMPY_WHL}"
        pip3 install --quiet --no-cache-dir --target="${DMCALIB_LIB}" "${NUMPY_WHL}" || true
      else
        pip3 install --quiet --no-cache-dir --target="${DMCALIB_LIB}" 'numpy<2' || true
      fi
    fi
    if [[ -f "${DMCALIB_LIB}/numpy/__init__.py" ]]; then
      DMCALIB_WRAPPER="/root/calib_ws/.calib_scripts/dmcalib_python_wrapper.sh"
      mkdir -p "$(dirname "${DMCALIB_WRAPPER}")"
      cat > "${DMCALIB_WRAPPER}" << 'WRAPEOF'
#!/bin/bash
# Python 包装：传递 NumPy 1.x 覆盖层（DM-Calib / 粗标定 run_lidar_cam_coarse 共用），参数原样传给 python3
if [[ -n "" ]]; then export PYTHONPATH=":"; fi
exec python3 ""
WRAPEOF
      chmod +x "${DMCALIB_WRAPPER}"
      export DMCALIB_LIB="${DMCALIB_LIB}"
      if [[ "$TASK" == "cam-intrin" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]]; then
        export UNICALIB_DM_CALIB_PYTHON="${DMCALIB_WRAPPER}"
        echo "[INFO] DM-Calib 将使用 NumPy 1.x 覆盖 (UNICALIB_DM_CALIB_PYTHON)"
      fi
      if [[ "$TASK" == "lidar-cam" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]]; then
        export UNICALIB_MIAS_LCEC_PYTHON="${DMCALIB_WRAPPER}"
        echo "[INFO] 粗标定 (MIAS-LCEC) 将使用 NumPy 1.x 覆盖 (UNICALIB_MIAS_LCEC_PYTHON)，cv2/PnP 可用"
      fi
    fi
  else
    # 系统 NumPy 1.x：DM-Calib 可选使用自带 venv
    if [[ -z "${UNICALIB_DM_CALIB_PYTHON:-}" ]] && { [[ "$TASK" == "cam-intrin" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]]; }; then
      if [[ -x "/root/calib_ws/DM-Calib/.venv/bin/python" ]]; then
        export UNICALIB_DM_CALIB_PYTHON="/root/calib_ws/DM-Calib/.venv/bin/python"
        echo "[INFO] DM-Calib 使用 venv: ${UNICALIB_DM_CALIB_PYTHON}"
      fi
    fi
    fi
fi

# lidar-cam/joint/all：若未安装 mobile_sam，从预下载目录或 MIAS-LCEC 内安装（与 --download-mobile-sam 配合）
NEED_MOBILESAM_TASK=0
[[ "$TASK" == "lidar-cam" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]] && NEED_MOBILESAM_TASK=1
if [[ "${NEED_MOBILESAM_TASK}" -eq 1 ]]; then
  if ! python3 -c "import mobile_sam" 2>/dev/null; then
    MOBILESAM_SRC=""
    if [[ -d "/root/calib_ws/calib_unified/thirdparty/MobileSAM" ]] && [[ -f "/root/calib_ws/calib_unified/thirdparty/MobileSAM/setup.py" ]]; then
      MOBILESAM_SRC="/root/calib_ws/calib_unified/thirdparty/MobileSAM"
    elif [[ -d "/root/calib_ws/MIAS-LCEC/bin/MobileSAM" ]] && [[ -f "/root/calib_ws/MIAS-LCEC/bin/MobileSAM/setup.py" ]]; then
      MOBILESAM_SRC="/root/calib_ws/MIAS-LCEC/bin/MobileSAM"
    fi
    if [[ -n "${MOBILESAM_SRC}" ]]; then
      echo "[INFO] 粗标定需要 MobileSAM，从本地安装: pip install -e ${MOBILESAM_SRC}"
      pip3 install -e "${MOBILESAM_SRC}" 2>/dev/null || true
      if python3 -c "import mobile_sam" 2>/dev/null; then
        echo "[INFO] MobileSAM 安装成功（来自 ${MOBILESAM_SRC}）"
      else
        echo "[WARN] MobileSAM 安装失败，粗标定将使用简化分割；可先执行: ./calib_unified_run.sh --download-mobile-sam"
      fi
    else
      echo "[WARN] 未找到 MobileSAM 源码目录（thirdparty/MobileSAM 或 MIAS-LCEC/bin/MobileSAM），粗标定将使用简化分割；可先执行: ./calib_unified_run.sh --download-mobile-sam"
    fi
  fi
fi

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║  UniCalib 标定任务: cam-cam"
echo "║  可执行: unicalib_cam_cam"
echo "╚════════════════════════════════════════════════════════╝"

if [[ ! -f "${EXE}" ]]; then
    echo "[ERROR] 未找到: ${EXE}"
    echo "[ERROR] 请先编译: ./calib_unified_run.sh --build-only"
    exit 1
fi
if [[ ! -f "${CONFIG}" ]]; then
    echo "[WARN] 配置文件不存在: ${CONFIG}"
    echo "[WARN] 使用默认唯一配置..."
    CONFIG="/root/calib_ws/calib_unified/config/unicalib_example.yaml"
fi

echo ""
echo "┌─────────────────────────────────────────────────────────────────────────┐"
echo "│  运行前检查 — 请确认以下项                                                │"
echo "├─────────────────────────────────────────────────────────────────────────┤"
echo "│  当前任务: cam-cam"
echo "│  说明:     相机与相机外参 (BA + 特征匹配)"
echo "│  配置文件: ${CONFIG}  (全工程唯一配置)"
echo "│  数据目录: /root/calib_ws/data  (宿主机: 请将数据放入 CALIB_DATA_DIR)"
echo "│  结果目录: /root/calib_ws/results"
echo "│  可执行:   ${EXE}"
echo "│  选项:     coarse=false  manual=true"
echo "│  IMU 备选: UNICALIB_TRANSFORMER_IMU=${UNICALIB_TRANSFORMER_IMU:-未设置}"
echo "└─────────────────────────────────────────────────────────────────────────┘"
echo "  详细任务说明与数据要求: 宿主机执行 ./calib_unified_run.sh --task-help"
echo ""

# 启用手动校准时，在终端打印操作说明，便于用户随时查看
if [[ "true" == "true" ]]; then
echo "══════════════════════════════════════════════════════════════════════════════"
echo "  手动标定操作说明 (--manual 已启用)"
echo "══════════════════════════════════════════════════════════════════════════════"
echo "  精标定完成后将弹出 6-DOF 调整窗口，请用以下按键微调外参："
echo ""
echo "  旋转:   Q/A  Roll   W/S  Pitch   E/D  Yaw"
echo "  平移:   R/F  Tx     T/G  Ty      Y/H  Tz"
echo "  其他:   Shift+键 = 10 倍步长   U = 撤销"
echo "  结束:   Enter = 接受当前外参   Esc = 取消(保留自动结果)"
echo ""
echo "  详细说明与配置见: calib_unified/docs/MANUAL_EXTRINSIC_ADJUSTMENT.md"
echo "══════════════════════════════════════════════════════════════════════════════"
echo ""
fi

LOG_FILE="/root/calib_ws/logs/run_cam-cam_$(date +%Y%m%d_%H%M%S).log"
echo "[INFO] 配置: ${CONFIG}"
echo "[INFO] 数据: /root/calib_ws/data"
echo "[INFO] 结果: /root/calib_ws/results"
echo "[INFO] 日志: ${LOG_FILE}"
echo "[INFO] 启动: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# cam-intrin 离线：若已预下载 SD2.1 到 results/sd2.1，则导出供 DM-Calib 使用（避免连 huggingface.co）
if [[ "$TASK" == "cam-intrin" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]]; then
  if [[ -d "/root/calib_ws/results/sd2.1" ]] && [[ -d "/root/calib_ws/results/sd2.1/text_encoder" ]]; then
    export UNICALIB_DM_CALIB_SD21_LOCAL="/root/calib_ws/results/sd2.1"
    echo "[INFO] DM-Calib 使用本地 SD2.1: ${UNICALIB_DM_CALIB_SD21_LOCAL}"
  fi
fi

# cam-intrin/joint 时预检 torch 及版本，避免 DM-Calib 不可用或 transformers 报 PyTorch>=2.4
if [[ "$TASK" == "cam-intrin" ]] || [[ "$TASK" == "joint" ]] || [[ "$TASK" == "all" ]]; then
  PY_FOR_TORCH="${UNICALIB_DM_CALIB_PYTHON:-python3}"
  if ! ${PY_FOR_TORCH} -c "import torch" 2>/dev/null; then
    if ! python3 -c "import torch" 2>/dev/null; then
      echo "[WARN] 当前环境无法 import torch，DM-Calib 将不可用，相机内参将使用先验或棋盘格结果（若需 DM-Calib 请安装: pip install torch）"
    fi
  else
    TORCH_V=$(${PY_FOR_TORCH} -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
    if [[ "$TORCH_V" == 2.0* ]] || [[ "$TORCH_V" == 2.1* ]] || [[ "$TORCH_V" == 2.2* ]] || [[ "$TORCH_V" == 2.3* ]]; then
      echo "[WARN] DM-Calib 使用的 Python 当前 PyTorch=${TORCH_V}；transformers 要求 PyTorch>=2.4，否则会报 'PyTorch was not found'。请先执行: ./calib_unified_run.sh --download-diffusers 或在当前环境: pip install 'torch>=2.4'"
    fi
  fi
fi

"${EXE}"     --config "${CONFIG}"     --data-dir "/root/calib_ws/data"      --manual     --log-level info     --output-dir "/root/calib_ws/results"     --ai-root "/root/calib_ws"     2>&1 | tee "${LOG_FILE}"

RC=${PIPESTATUS[0]}
echo ""
if [[ $RC -eq 0 ]]; then
    echo "╔══════════════════════════════════╗"
    echo "║  标定完成 ✓                      ║"
    echo "╚══════════════════════════════════╝"
    echo "结果目录: /root/calib_ws/results"
    ls -lh "/root/calib_ws/results/" 2>/dev/null || true
else
    echo "[ERROR] 标定失败 (exit=${RC})"
    echo "日志: ${LOG_FILE}"
fi
exit $RC