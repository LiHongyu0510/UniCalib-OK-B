#!/bin/bash
# =============================================================================
# 在容器内构建并运行 C++ 主框架标定（unicalib_example）
# 用于 make run，与 README 主框架一致
# =============================================================================
set -e

WORKSPACE="${WORKSPACE:-/root/calib_ws}"
UNICALIB_CPP="${WORKSPACE}/unicalib_C_plus_plus"
CONFIG="${UNICALIB_CPP}/config/sensors.yaml"
DATA_DIR="${WORKSPACE}/data"
RESULTS_DIR="${WORKSPACE}/results"

if [ ! -d "${UNICALIB_CPP}" ]; then
    echo "[ERROR] unicalib_C_plus_plus not found at ${UNICALIB_CPP} (mount volume?)"
    exit 1
fi

# 若未构建则先构建
if [ ! -f "${UNICALIB_CPP}/build/unicalib_example" ]; then
    echo "[INFO] Building unicalib_C_plus_plus..."
    mkdir -p "${UNICALIB_CPP}/build"
    (cd "${UNICALIB_CPP}/build" && cmake .. && make -j$(nproc))
    echo "[INFO] Build done."
fi

if [ ! -d "${DATA_DIR}" ] || [ -z "$(ls -A ${DATA_DIR} 2>/dev/null)" ]; then
    echo "[WARN] Data dir empty or missing: ${DATA_DIR}. Run may fail."
fi

mkdir -p "${RESULTS_DIR}"
cd "${UNICALIB_CPP}/build"

echo "[INFO] Running full calibration pipeline (C++ main framework)..."
RUN_PIPELINE=1 ./unicalib_example "${CONFIG}" "${DATA_DIR}"

# 将输出复制到统一结果目录（config 中 output_dir 默认为 ./calib_results，即 build/calib_results）
if [ -d "calib_results" ]; then
    echo "[INFO] Copying results to ${RESULTS_DIR}"
    rm -rf "${RESULTS_DIR}"/*
    cp -r calib_results/* "${RESULTS_DIR}"/
fi

echo "[INFO] Done. Results in ${RESULTS_DIR}"
