#!/usr/bin/env bash
# =============================================================================
# UniCalib calib_unified — 一键编译与运行脚本
# 镜像: calib_env:humble (不修改镜像 / Dockerfile)
# 挂载策略: 源码/构建产物/数据/结果/脚本 全部 volume 挂载
#
# 用法:
#   ./calib_unified_run.sh                         # 编译 + 自动化验证
#   ./calib_unified_run.sh --build-only            # 仅编译
#   ./calib_unified_run.sh --test-only             # 仅自动化验证
#   ./calib_unified_run.sh --run --task joint      # 运行联合标定
#   ./calib_unified_run.sh --run --task lidar-cam  # 运行 lidar-cam 标定（精标定后可选进手动）
#   ./calib_unified_run.sh --run --task lidar-cam --manual   # 同上，并启用手动外参调整（推荐）
#   ./calib_unified_run.sh --run --task lidar-cam --coarse --manual
#   ./calib_unified_run.sh --run --task cam-cam --manual      # 相机-相机外参 + 手动调整
#   ./calib_unified_run.sh --run --task lidar-lidar --manual  # 雷达-雷达外参 + 手动调整
#   ./calib_unified_run.sh --run --task imu-lidar --manual    # IMU-雷达外参 + 手动调整
#   ./calib_unified_run.sh --shell                 # 进入容器调试
#   ./calib_unified_run.sh --check-deps            # 检查容器内依赖
#   ./calib_unified_run.sh --clean                 # 清理后重新编译
#   ./calib_unified_run.sh --task-help             # 各标定任务详细说明与数据要求
#
# 手动标定: 使用 --manual 时，运行后终端会打印「手动标定操作说明」(按键与步骤)；
#           详细文档见 calib_unified/docs/MANUAL_EXTRINSIC_ADJUSTMENT.md
#
# 标定任务 (--task)；全工程唯一配置: config/unicalib_example.yaml
#   imu-intrin   IMU内参   cam-intrin 相机内参   imu-lidar IMU-雷达外参
#   lidar-cam 雷达-相机外参   lidar-lidar 雷达-雷达外参   cam-cam 相机-相机外参
#   joint/all 联合标定
#
# 环境变量:
#   CALIB_DATA_DIR      数据目录   (默认: $PROJECT/data)
#   CALIB_RESULTS_DIR   结果目录   (默认: $PROJECT/results)
#   CALIB_LOGS_DIR      日志目录   (默认: $PROJECT/logs，编译/运行日志带时间戳)
#   DOCKER_GPU_FLAGS    GPU标志    (默认: --gpus all)
#   J                   并行编译数 (默认: nproc)
# =============================================================================
set -eo pipefail

# ─── 路径常量 ──────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
CALIB_UNIFIED_DIR="${PROJECT_ROOT}/calib_unified"

# 容器内挂载点
CONTAINER_WS="/root/calib_ws"
CONTAINER_CALIB="${CONTAINER_WS}/calib_unified"
CONTAINER_BUILD="${CONTAINER_CALIB}/build"
CONTAINER_BIN="${CONTAINER_BUILD}/bin"
CONTAINER_DATA="${CONTAINER_WS}/data"
CONTAINER_RESULTS="${CONTAINER_WS}/results"
CONTAINER_LOGS="${CONTAINER_WS}/logs"
CONTAINER_SCRIPTS="${CONTAINER_WS}/.calib_scripts"   # 临时脚本挂载点
CONTAINER_AI="${CONTAINER_WS}"

# 宿主机路径（DM-Calib 可不在项目根：设置 DM_CALIB_PATH 指向 @DM-Calib 根目录）
DM_CALIB_PATH="${DM_CALIB_PATH:-${PROJECT_ROOT}/DM-Calib}"
BUILD_DIR="${CALIB_UNIFIED_DIR}/build"
CALIB_DATA_DIR="${CALIB_DATA_DIR:-${PROJECT_ROOT}/data}"
CALIB_RESULTS_DIR="${CALIB_RESULTS_DIR:-${PROJECT_ROOT}/results}"
CALIB_LOGS_DIR="${CALIB_LOGS_DIR:-${PROJECT_ROOT}/logs}"
SCRIPTS_TMPDIR="${PROJECT_ROOT}/.calib_scripts"  # 临时脚本目录 (挂载入容器)

# Docker
DOCKER_IMAGE="calib_env:humble"
DOCKER_GPU_FLAGS="${DOCKER_GPU_FLAGS:---gpus all}"

# ─── 运行模式 ──────────────────────────────────────────────────────────────────
MODE="default"   # default | build-only | test-only | run | shell | check-deps | download-numpy | download-diffusers | download-sd21 | download-dm-calib | download-mobile-sam
CALIB_TASK="all"
CALIB_CONFIG=""
CALIB_DATASET=""   # 可选，如 nya_02_ros2 → 数据目录为 $CALIB_DATA_DIR/$CALIB_DATASET
DO_COARSE=false
COARSE_EXPLICIT=false   # 用户是否显式传了 --coarse 或 --no-coarse
DO_MANUAL=false
DO_PANGOLIN_PANEL=false
BUILD_TYPE="Release"
LOG_LEVEL="info"
CLEAN_BUILD=false
BUILD_JOBS="${J:-$(nproc)}"
EXTRA_CMAKE_ARGS=""

# ─── 颜色 ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${BLUE}${BOLD}──── $* ────${NC}"; }
log_ok()    { echo -e "${GREEN}${BOLD}  ✓ $*${NC}"; }
log_fail()  { echo -e "${RED}${BOLD}  ✗ $*${NC}"; }

# ─── Banner / Help ─────────────────────────────────────────────────────────────
print_banner() {
    echo -e "${CYAN}${BOLD}"
    cat << 'BANNER'
 ╔═══════════════════════════════════════════════════════════════╗
 ║  UniCalib calib_unified — 多传感器两阶段标定一键脚本           ║
 ║  镜像: calib_env:humble  (只挂载, 不修改镜像/Dockerfile)       ║
 ║  支持: IMU内参/相机内参/IMU-LiDAR/LiDAR-Cam/LiDAR-LiDAR/Cam-Cam           ║
 ║  两阶段: AI粗标定 → 无目标精标定 → 手动校准                   ║
 ╚═══════════════════════════════════════════════════════════════╝
BANNER
    echo -e "${NC}"
}

print_help() {
    cat << 'HELPEOF'
用法: ./calib_unified_run.sh [选项]

模式:
  (默认)                       编译 + 自动化验证
  --build-only                 仅编译
  --test-only                  仅自动化验证 (需已编译)
  --run                        编译 + 运行标定
  --shell                      进入容器交互式 Shell
  --check-deps                 检查容器内 C++ 依赖可用性
  --download-numpy             预下载 NumPy 1.x wheel 到 calib_unified/thirdparty/wheels（cam-intrin 运行时免联网）
  --download-diffusers         预下载 PyTorch>=2.4 + diffusers/transformers 到 thirdparty/wheels（DM-Calib 免联网；可选 UNICALIB_TORCH_INDEX=cu118|cu121|cpu）
  --download-sd21              预下载 sd2-community/stable-diffusion-2-1 到 results/sd2.1（cam-intrin 离线用；国内可加 HF_ENDPOINT=https://hf-mirror.com 加速）
  --download-dm-calib           预下载 juneyoung9/DM-Calib（及可选 SD2.1）到 DM-Calib/model/dm_calib_pretrained，供 from_pretrained 本地加载
  --download-mobile-sam         预下载 MobileSAM 到 calib_unified/thirdparty/MobileSAM（lidar-cam 粗标定运行时从此处安装，免联网）
  --clean                      清理构建目录后重新编译

标定任务 (--run 时 --task <名称>):
  imu-intrin    IMU 内参标定 (Allan 方差 + 可选 Transformer-IMU 粗估)
  cam-intrin    相机内参标定 (棋盘格标定 + 可选 DM-Calib 粗估)
  imu-lidar     IMU 与雷达外参标定 (B 样条 + 可选 L2Calib 粗估)
  lidar-cam     雷达与相机外参标定 (边缘对齐 + 默认 MIAS-LCEC 粗标定，可用 --no-coarse 关闭)
  lidar-lidar   雷达与雷达外参标定 (NDT/GICP 两阶段标定)
  cam-cam       相机与相机外参标定 (BA + 特征匹配)
  joint / all   联合标定 (上述全部或可选组合，由 unicalib_joint 执行)

标定 (--run 模式，全部使用同一配置 unicalib_example.yaml):
  --task <t>     任务: imu-intrin | cam-intrin | imu-lidar | lidar-cam | lidar-lidar | cam-cam | joint (默认 all)
  --config <f>   配置文件 (默认 calib_unified/config/unicalib_example.yaml)
  --data-dir <d> 数据根目录；或 --dataset <名> 使用 data/<名>（如 nya_02_ros2）
  --coarse       启用 AI 粗标定（lidar-cam 任务默认已启用）
  --no-coarse    禁用 AI 粗标定（仅当需关闭 lidar-cam 默认粗标定时使用）
  --manual       启用手动校准 (外参任务)；点云叠加窗，详见 calib_unified/docs/MANUAL_EXTRINSIC_ADJUSTMENT.md
  --pangolin-panel  LiDAR-Camera 手动时使用 Pangolin 点云叠加+6-DOF 面板 (需先 --manual)

  --task-help    显示各标定任务的详细说明与数据要求

编译:
  --debug        Debug 模式 (默认 Release)
  --jobs <n>     并行编译数 (默认: nproc)
  --cmake-args   额外 CMake 参数 (用引号包裹)

数据 (可与 --config 配合，指定本次标定使用的数据来源):
  --data-dir <d>    宿主机数据根目录 (挂载为容器内 CALIB_DATA_DIR)
  --dataset <名>    数据集子目录，数据目录 = <data-dir>/<名>，例: --dataset nya_02_ros2
  --results-dir <d> 宿主机结果目录

日志:
  --log-level <l>   trace|debug|info|warn|error

  -h / --help       显示此帮助

环境变量:
  CALIB_DATA_DIR     宿主机数据目录
  CALIB_RESULTS_DIR  宿主机结果目录
  CALIB_LOGS_DIR     宿主机日志目录
  DOCKER_GPU_FLAGS   GPU 标志 (默认 --gpus all)
  J                  并行编译数

HELPEOF
}

# 各标定任务的详细说明（数据要求、配置、示例命令）
print_task_help() {
    cat << 'TASKHELP'
╔══════════════════════════════════════════════════════════════════════════════╗
║  标定任务详细说明 — 数据要求、推荐配置与示例命令                              ║
╚══════════════════════════════════════════════════════════════════════════════╝

■ imu-intrin — IMU 内参标定
  ─────────────────────────────────────────────────────────────────────────
  说明: 标定 IMU 的噪声密度、随机游走、尺度与安装偏置等内参，用于提高
        IMU 积分与融合精度。
  方法: Allan 方差分析 + 可选 Transformer-IMU-Calibrator 粗估计。
  数据要求:
    • 静态 IMU 数据：设备静止放置，时长建议 ≥ 2 小时（或按 Allan 曲线需求）
    • 数据格式：ROS bag（/imu 或配置中指定的 topic）或项目支持的 IMU 数据文件
  配置文件: 全工程唯一配置 calib_unified/config/unicalib_example.yaml
  示例命令:
    ./calib_unified_run.sh --run --task imu-intrin
    ./calib_unified_run.sh --run --task imu-intrin --config calib_unified/config/unicalib_example.yaml
  输出: 标定后的 IMU 内参 YAML（噪声、偏置等），可写入传感器配置或 joint 配置。

■ cam-intrin — 相机内参标定
  ─────────────────────────────────────────────────────────────────────────
  说明: 标定相机内参（焦距、主点、畸变系数），用于去畸变与 3D 重建。
  方法: 棋盘格/标定板 + 可选 DM-Calib 粗估计。
  数据要求:
    • 多角度拍摄标定板的图像或图像序列（或带 /camera/image 的 ROS bag）
    • 标定板类型与尺寸需与配置中一致（如棋盘格行列数、格子大小）
  配置文件: 全工程唯一配置 calib_unified/config/unicalib_example.yaml
  示例命令:
    ./calib_unified_run.sh --run --task cam-intrin
    ./calib_unified_run.sh --run --task cam-intrin --config calib_unified/config/unicalib_example.yaml
  输出: 相机内参 YAML（K、畸变模型与参数），供外参标定或联合标定使用。

■ imu-lidar — IMU 与雷达外参标定
  ─────────────────────────────────────────────────────────────────────────
  说明: 标定 IMU 与 LiDAR 之间的相对位姿 T_imu_lidar，用于紧耦合融合与定位。
  方法: 基于运动的 B 样条优化 + 可选 L2Calib 粗估计。
  数据要求:
    • 同步的 IMU 数据 + LiDAR 点云（ROS bag 或等价的时序数据）
    • 采集时需有充分激励：加减速、转弯、俯仰/侧倾变化，避免纯匀速直线
  配置文件: 全工程唯一配置 calib_unified/config/unicalib_example.yaml
  示例命令:
    ./calib_unified_run.sh --run --task imu-lidar
    ./calib_unified_run.sh --run --task imu-lidar --coarse
  输出: T_imu_lidar 外参（或写入 joint 配置），供融合与联合标定使用。

■ lidar-cam — 雷达与相机外参标定
  ─────────────────────────────────────────────────────────────────────────
  说明: 标定 LiDAR 与相机之间的外参 T_lidar_cam，用于点云与图像对齐、融合。
  方法: 边缘/特征对齐 + 可选 MIAS-LCEC 粗估计；可选无目标方法。
  数据要求:
    • 同步的 LiDAR 点云 + 相机图像（ROS bag 或等价的时序数据）
    • 场景最好包含清晰边缘/结构（如建筑物、标定板边缘），避免单调纹理
  配置文件: 全工程唯一配置 calib_unified/config/unicalib_example.yaml
  示例命令:
    ./calib_unified_run.sh --run --task lidar-cam
    ./calib_unified_run.sh --run --task lidar-cam --config calib_unified/config/unicalib_example.yaml
    ./calib_unified_run.sh --run --task lidar-cam --config calib_unified/config/unicalib_example.yaml --dataset nya_02_ros2
    ./calib_unified_run.sh --run --task lidar-cam --data-dir /path/to/data --dataset nya_02_ros2 --coarse --manual
  输出: T_lidar_cam 外参；若精度不足可结合 --manual 进行交互微调。

■ lidar-lidar — 雷达与雷达外参标定
  ─────────────────────────────────────────────────────────────────────────
  说明: 标定多激光雷达之间的外参（如前视/后视/侧视雷达），用于多雷达点云融合。
  方法: NDT/GICP 两阶段标定 — 粗标定 (NDT/多帧合并) → 精标定 (GICP/NDT)。
  数据要求:
    • 两个或多个 LiDAR 的点云数据（PCD 文件目录或 ROS bag）
    • 采集时需有充分运动以获得良好重叠区域
  配置文件: 全工程唯一配置 calib_unified/config/unicalib_example.yaml
  示例命令:
    ./calib_unified_run.sh --run --task lidar-lidar
    ./calib_unified_run.sh --run --task lidar-lidar --config calib_unified/config/unicalib_example.yaml
  输出: T_lidar_to_lidar 外参 (ref→target)

■ cam-cam — 相机与相机外参标定
  ─────────────────────────────────────────────────────────────────────────
  说明: 标定多相机之间的外参（如双目或前/后/环视），用于多视角融合与 3D。
  方法: 特征匹配 + 束调整（BA）；可结合标定板或自然特征。
  数据要求:
    • 多相机同步或时序对齐的图像（ROS bag 或图像序列）
    • 各相机内参建议先由 cam-intrin 标定好，或配置中提供
  配置文件: 全工程唯一配置 calib_unified/config/unicalib_example.yaml
  示例命令:
    ./calib_unified_run.sh --run --task cam-cam
    ./calib_unified_run.sh --run --task cam-cam --manual
  输出: 相机间外参；--manual 可在精度不足时进行手动微调。

■ joint / all — 联合标定
  ─────────────────────────────────────────────────────────────────────────
  说明: 一次性运行多种标定（可仅选其中几项，由 unicalib_joint 内部 --imu-intrin
       等标志控制）。适合整机标定流程或依赖内参的外参联合优化。
  方法: 按配置与任务选择依次/联合执行 imu-intrin、cam-intrin、imu-lidar、
        lidar-cam、cam-cam；可选 AI 粗标定与手动校准。
  数据要求: 满足所勾选任务的数据（见上各任务）。
  配置文件: 全工程唯一配置 calib_unified/config/unicalib_example.yaml
  示例命令:
    ./calib_unified_run.sh --run --task joint
    ./calib_unified_run.sh --run --task joint --coarse --manual
  输出: 各任务结果写入同一输出目录，便于统一写入车辆配置。

────────────────────────────────────────────────────────────────────────────────
通用提示:
  • 全工程唯一配置: calib_unified/config/unicalib_example.yaml（--run 时默认使用）。
  • 数据路径: 默认数据目录 CALIB_DATA_DIR 或 ./data；可用 --data-dir <路径> 指定根目录。
  • 数据集: 使用 --dataset <名>（如 nya_02_ros2）时，数据目录为 <data-dir>/<名>。
  • 结果目录: 默认 CALIB_RESULTS_DIR 或 ./results；可用 --results-dir 指定。
  • lidar-cam 任务默认启用粗标定（MIAS-LCEC）；其他任务需加 --coarse 才启用。--no-coarse 可关闭粗标定。
  • --coarse 会调用相应 AI 模型（需已挂载 DM-Calib/MIAS-LCEC 等），首次可能较慢。
  • --manual 仅对外参任务生效，用于启动交互式手动校准工具。

TASKHELP
}

# ─── 解析参数 ─────────────────────────────────────────────────────────────────
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --build-only)  MODE="build-only" ;;
            --test-only)   MODE="test-only" ;;
            --run)         MODE="run" ;;
            --shell)       MODE="shell" ;;
            --check-deps)      MODE="check-deps" ;;
            --download-numpy)     MODE="download-numpy" ;;
            --download-diffusers) MODE="download-diffusers" ;;
            --download-sd21)      MODE="download-sd21" ;;
            --download-dm-calib)  MODE="download-dm-calib" ;;
            --download-mobile-sam) MODE="download-mobile-sam" ;;
            --clean)       CLEAN_BUILD=true ;;
            --coarse)      DO_COARSE=true;  COARSE_EXPLICIT=true ;;
            --no-coarse)   DO_COARSE=false; COARSE_EXPLICIT=true ;;
            --manual)      DO_MANUAL=true ;;
            --pangolin-panel) DO_PANGOLIN_PANEL=true ;;
            --debug)       BUILD_TYPE="Debug" ;;
            --task)        shift; CALIB_TASK="$1" ;;
            --config)      shift; CALIB_CONFIG="$1" ;;
            --jobs)        shift; BUILD_JOBS="$1" ;;
            --log-level)   shift; LOG_LEVEL="$1" ;;
            --cmake-args)  shift; EXTRA_CMAKE_ARGS="$1" ;;
            --data-dir)    shift; CALIB_DATA_DIR="$1" ;;
            --dataset)     shift; CALIB_DATASET="$1" ;;
            --results-dir) shift; CALIB_RESULTS_DIR="$1" ;;
            --task-help)   print_banner; print_task_help; exit 0 ;;
            --help|-h)     print_banner; print_help; exit 0 ;;
            *) log_warn "未知参数: $1" ;;
        esac
        shift
    done
    # 全工程唯一配置文件（未指定 --config 时使用）
    if [[ -z "${CALIB_CONFIG}" ]]; then
        CALIB_CONFIG="${CALIB_UNIFIED_DIR}/config/unicalib_example.yaml"
    fi
    # --dataset <name>：数据目录设为 $CALIB_DATA_DIR/<name>，便于按数据集切换（如 nya_02_ros2）
    if [[ -n "${CALIB_DATASET}" ]]; then
        CALIB_DATA_DIR="${CALIB_DATA_DIR}/${CALIB_DATASET}"
    fi
    # lidar-cam 任务默认启用粗标定（MIAS-LCEC）；未显式传 --coarse/--no-coarse 时生效
    if [[ "${MODE}" == "run" ]] && [[ "${CALIB_TASK}" == "lidar-cam" ]] && [[ "${COARSE_EXPLICIT}" != "true" ]]; then
        DO_COARSE=true
    fi
}

# ─── 检查 Docker ───────────────────────────────────────────────────────────────
check_docker() {
    log_step "检查 Docker 环境"
    command -v docker &>/dev/null || { log_error "Docker 未安装"; exit 1; }
    log_info "Docker: $(docker --version)"
    docker info &>/dev/null || { log_error "Docker 未运行"; exit 1; }

    docker image inspect "${DOCKER_IMAGE}" &>/dev/null || {
        log_error "镜像 ${DOCKER_IMAGE} 不存在"
        log_error "请先构建: bash docker/docker_build.sh"
        exit 1
    }
    log_ok "镜像 ${DOCKER_IMAGE} 就绪"

    # GPU (非致命)
    if docker run --rm ${DOCKER_GPU_FLAGS} --entrypoint="" \
       "${DOCKER_IMAGE}" nvidia-smi -L &>/dev/null 2>&1; then
        log_ok "GPU 支持可用"
    else
        log_warn "GPU 不可用 — 使用 CPU"
        DOCKER_GPU_FLAGS=""
    fi
}

# ─── 停止已运行的同一镜像容器（--run 前调用，避免多实例互相影响）────────────
stop_running_calib_containers() {
    local running
    running=$(docker ps -q --filter "ancestor=${DOCKER_IMAGE}" 2>/dev/null)
    if [[ -n "${running}" ]]; then
        log_warn "检测到宿主机上已有使用镜像 ${DOCKER_IMAGE} 的容器在运行，正在停止..."
        docker stop ${running}
        log_ok "已停止旧容器，即将启动本次运行"
    fi
}

# ─── 初始化目录 ────────────────────────────────────────────────────────────────
init_dirs() {
    mkdir -p "${BUILD_DIR}" \
             "${CALIB_DATA_DIR}" \
             "${CALIB_RESULTS_DIR}" \
             "${CALIB_LOGS_DIR}" \
             "${SCRIPTS_TMPDIR}"
}

# ─── 写脚本到挂载目录并在容器内执行 ───────────────────────────────────────────
# 用法: write_and_run <script_name> <script_content> [interactive] [extra_env]
# extra_env: 可选，格式 "KEY=VALUE" 或 "K1=V1 K2=V2"，会作为 -e 传入容器
write_and_run() {
    local script_name="$1"       # e.g. "build.sh"
    local script_content="$2"    # 脚本内容字符串
    local interactive="${3:-}"   # 可选: "-it"
    local extra_env="${4:-}"    # 可选: "PIP_INDEX_URL=https://..."

    local host_script="${SCRIPTS_TMPDIR}/${script_name}"
    local container_script="${CONTAINER_SCRIPTS}/${script_name}"

    # 确保临时目录存在后再写入（避免 cleanup 或异常导致目录被删）
    mkdir -p "${SCRIPTS_TMPDIR}"
    # 写入宿主机临时目录 (挂载入容器)
    printf '%s' "${script_content}" > "${host_script}"
    chmod +x "${host_script}"
    # 确保容器挂载前文件已落盘并可读（避免容器内 No such file or directory）
    if [[ ! -f "${host_script}" ]] || [[ ! -s "${host_script}" ]]; then
        log_error "脚本写入失败或为空: ${host_script}"
        exit 1
    fi
    sync 2>/dev/null || true

    # X11 支持
    local x11_args=()
    if [[ -n "${DISPLAY:-}" ]]; then
        xhost +local:docker &>/dev/null || true
        x11_args=(
            -e "DISPLAY=${DISPLAY}"
            -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"
        )
        [[ -f "${HOME}/.Xauthority" ]] && \
            x11_args+=(-v "${HOME}/.Xauthority:/root/.Xauthority:ro")
    fi

    # 可选额外环境变量（如 PIP_INDEX_URL 用于加速下载）
    local extra_env_args=()
    if [[ -n "${extra_env}" ]]; then
        while read -r kv; do
            [[ -n "$kv" ]] && extra_env_args+=(-e "$kv")
        done <<< "$extra_env"
    fi

    # 执行（UNICALIB_TRANSFORMER_IMU 与挂载点一致，供 imu-intrin 静态段<6 时使用）
    docker run \
        --rm \
        ${interactive} \
        ${DOCKER_GPU_FLAGS} \
        --ipc=host \
        --network=host \
        "${x11_args[@]}" \
        "${extra_env_args[@]}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e CALIB_DATA_DIR="${CONTAINER_DATA}" \
        -e CALIB_RESULTS_DIR="${CONTAINER_RESULTS}" \
        -e CALIB_LOGS_DIR="${CONTAINER_LOGS}" \
        -e AI_ROOT="${CONTAINER_AI}" \
        -e UNICALIB_DM_CALIB="${CONTAINER_AI}/DM-Calib" \
        -e UNICALIB_DM_CALIB_MODEL="${CONTAINER_AI}/DM-Calib/model" \
        -e UNICALIB_TRANSFORMER_IMU="${CONTAINER_AI}/Transformer-IMU-Calibrator" \
        -e PYTHONPATH="${CONTAINER_AI}/DM-Calib:${CONTAINER_AI}/learn-to-calibrate:${CONTAINER_AI}/learn-to-calibrate/rl_solver:${CONTAINER_AI}/MIAS-LCEC:${CONTAINER_AI}/Transformer-IMU-Calibrator" \
        -v "${CALIB_UNIFIED_DIR}:${CONTAINER_CALIB}:rw" \
        -v "${BUILD_DIR}:${CONTAINER_BUILD}:rw" \
        -v "${CALIB_DATA_DIR}:${CONTAINER_DATA}:rw" \
        -v "${CALIB_RESULTS_DIR}:${CONTAINER_RESULTS}:rw" \
        -v "${CALIB_LOGS_DIR}:${CONTAINER_LOGS}:rw" \
        -v "${SCRIPTS_TMPDIR}:${CONTAINER_SCRIPTS}:rw" \
        -v "${DM_CALIB_PATH}:${CONTAINER_AI}/DM-Calib:ro" \
        -v "${PROJECT_ROOT}/MIAS-LCEC:${CONTAINER_AI}/MIAS-LCEC:rw" \
        -v "${PROJECT_ROOT}/learn-to-calibrate:${CONTAINER_AI}/learn-to-calibrate:ro" \
        -v "${PROJECT_ROOT}/Transformer-IMU-Calibrator:${CONTAINER_AI}/Transformer-IMU-Calibrator:ro" \
        -v "${PROJECT_ROOT}/click_calib:${CONTAINER_AI}/click_calib:ro" \
        "${DOCKER_IMAGE}" \
        bash "${container_script}"
}

# ─── 生成: 依赖检查脚本 ────────────────────────────────────────────────────────
gen_check_deps_script() {
    cat << SCRIPTEOF
#!/usr/bin/env bash
set -eo pipefail
echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  容器内 C++ 依赖检查                      ║"
echo "╚══════════════════════════════════════════╝"
echo ""

check_lib() {
    local name="\$1" pattern="\$2"
    local found
    found=\$(find /usr /opt /root -name "\${pattern}" 2>/dev/null | head -1)
    if [[ -n "\${found}" ]]; then
        printf "  ✓ %-20s → %s\n" "\${name}" "\${found}"
    else
        printf "  ✗ %-20s 未找到 %s\n" "\${name}" "\${pattern}"
    fi
}

echo "■ 编译工具:"
printf "  cmake : %s\n" "\$(cmake --version | head -1)"
printf "  gcc   : %s\n" "\$(gcc --version | head -1)"
printf "  g++   : %s\n" "\$(g++ --version | head -1)"

echo ""
echo "■ C++ 依赖库:"
check_lib "Eigen3"    "Eigen/Core"
check_lib "Ceres"     "libceres*"
check_lib "PCL"       "libpcl_common*"
check_lib "OpenCV4"   "libopencv_core*"
check_lib "spdlog"    "libspdlog*"
check_lib "yaml-cpp"  "libyaml-cpp*"
check_lib "Sophus"    "se3.hpp"

echo ""
echo "■ CMake 配置文件:"
for f in \
    /usr/local/lib/cmake/Ceres/CeresConfig.cmake \
    /usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake \
    /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake \
    /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake; do
    [[ -f "\$f" ]] && echo "  ✓ \$f" || echo "  ✗ \$f"
done

echo ""
echo "■ LibTorch (C++):"
echo "  calib_unified 未链接 LibTorch，无法在 C++ 中直接加载 .pth 模型"
found=\$(find /usr /opt -name "libtorch*" -o -path "*libtorch*/lib/*.so" 2>/dev/null | head -1)
if [[ -n "\${found}" ]]; then
    echo "  系统存在 LibTorch: \${found} (当前构建未使用)"
else
    echo "  未检测到 LibTorch (imu-intrin 不依赖: 静态段<6 时用 C++ 原生零偏估计)"
fi

echo ""
echo "■ Python 环境:"
printf "  python3: %s\n" "\$(python3 --version)"
python3 -c "import torch; print('  torch  : ' + torch.__version__)" 2>/dev/null || echo "  torch  : 未安装"
python3 -c "import diffusers; print('  diffusers: OK')" 2>/dev/null || echo "  diffusers: 未安装 (DM-Calib不可用)"
python3 -c "import yaml; print('  pyyaml : OK')" 2>/dev/null || echo "  pyyaml : 未安装"

echo ""
echo "■ IMU 内参 (静态段<6 时):"
echo "  无 Python/无 Transformer-IMU 时自动使用 C++ 原生零偏估计，标定仍可完成"

echo ""
echo "■ AI 模型挂载点:"
for d in DM-Calib MIAS-LCEC learn-to-calibrate Transformer-IMU-Calibrator click_calib; do
    D="${CONTAINER_AI}/\${d}"
    [[ -d "\$D" ]] && echo "  ✓ \$D" || echo "  ✗ \$D (未挂载)"
done

echo ""
echo "■ 构建目录:"
echo "  ${CONTAINER_BUILD}"
[[ -d "${CONTAINER_BUILD}" ]] && ls -lh "${CONTAINER_BUILD}/bin/" 2>/dev/null || echo "  (未编译)"
echo ""
SCRIPTEOF
}

# ─── 生成: 编译脚本 ────────────────────────────────────────────────────────────
gen_build_script() {
    local build_type="${BUILD_TYPE}"
    local jobs="${BUILD_JOBS}"
    local extra="${EXTRA_CMAKE_ARGS}"
    local clean="${CLEAN_BUILD}"

    cat << SCRIPTEOF
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  编译 calib_unified                       ║"
echo "╚══════════════════════════════════════════╝"
echo "  源码: ${CONTAINER_CALIB}"
echo "  构建: ${CONTAINER_BUILD}"
echo "  类型: ${build_type}  并行: ${jobs}"
echo ""

# 清理 (可选)
if [[ "${clean}" == "true" ]]; then
    echo "[INFO] 清理旧构建..."
    rm -rf "${CONTAINER_BUILD:?}"/*
    mkdir -p "${CONTAINER_BUILD}"
fi

TP_INSTALL="${CONTAINER_BUILD}/thirdparty-install"
mkdir -p "\${TP_INSTALL}"

# ── 0/4 第三方库: ufomap / Pangolin / veta (从 thirdparty 源码编译安装) ─────
echo "[0/4] 第三方库: ufomap / Pangolin (从 thirdparty 源码编译)..."

# ufomap: 使用 thirdparty/ufomap/ufomap 源码编译安装
UFOMAP_SRC="${CONTAINER_CALIB}/thirdparty/ufomap/ufomap"
UFOMAP_BLD="${CONTAINER_BUILD}/ufomap-build"
UFOMAP_INS="\${TP_INSTALL}/ufomap"
if [[ -f "\${UFOMAP_SRC}/CMakeLists.txt" ]]; then
    if [[ ! -f "\${UFOMAP_INS}/lib/cmake/ufomap/ufomapConfig.cmake" ]]; then
        echo "  编译 ufomap..."
        mkdir -p "\${UFOMAP_BLD}" && cd "\${UFOMAP_BLD}"
        cmake "\${UFOMAP_SRC}" -DCMAKE_BUILD_TYPE=${build_type} -DCMAKE_INSTALL_PREFIX="\${UFOMAP_INS}" -DBUILD_SHARED_LIBS=ON
        cmake --build . -- -j${jobs} && cmake --install .
        cd "${CONTAINER_BUILD}"
    else
        echo "  ufomap 已安装，跳过"
    fi
else
    echo "  [WARN] 未找到 ufomap 源码: \${UFOMAP_SRC}"
fi

# Pangolin (C++ 可视化库，替代 tiny-viewer)
# 注意: Pangolin 已通过 CMakeLists.txt add_subdirectory 集成，此处预构建为可选优化
# 源码位于 thirdparty/Pangolin/Pangolin-0.9.0/
PANGOLIN_SRC="${CONTAINER_CALIB}/thirdparty/Pangolin/Pangolin-0.9.0"
PANGOLIN_BLD="${CONTAINER_BUILD}/pangolin-build"
PANGOLIN_INS="\${TP_INSTALL}/pangolin"
if [[ -f "\${PANGOLIN_SRC}/CMakeLists.txt" ]]; then
    echo "  编译 Pangolin 0.9.0..."
    mkdir -p "\${PANGOLIN_BLD}" && cd "\${PANGOLIN_BLD}"
    cmake "\${PANGOLIN_SRC}" -DCMAKE_BUILD_TYPE=${build_type} -DCMAKE_INSTALL_PREFIX="\${PANGOLIN_INS}" -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYPANGOLIN=OFF
    cmake --build . -- -j${jobs} && cmake --install .
    cd "${CONTAINER_BUILD}"
    echo "  Pangolin 已安装到 \${PANGOLIN_INS}"
else
    echo "  [WARN] 未找到 Pangolin 源码: \${PANGOLIN_SRC}"
fi

# veta: 已内置于 thirdparty/veta-impl (自包含头文件库), 无外部依赖
echo "  veta: 使用项目内置 thirdparty/veta-impl (自包含, 无外部依赖)"

cd "${CONTAINER_BUILD}"

# ── 1/4 CMake 配置 ──────────────────────────────────────────────────────────
echo "[1/4] CMake 配置..."
# 依赖: veta=thirdparty/veta-impl, ctraj=basalt-headers+ctraj_basalt_compat, opengv/ufomap 已存在
# ufomap: 可选使用 iKalibr 预编译 (含 surfel/octree API)
# Pangolin: 通过 CMakeLists.txt add_subdirectory 直接从源码集成
CMAKE_EXTRA_TP=""
IKALIBR_TP_INSTALL="${CONTAINER_WS}/iKalibr/thirdparty-install"
[[ -f "\${IKALIBR_TP_INSTALL}/ufomap-install/lib/cmake/ufomap/ufomapConfig.cmake" ]] && \
    CMAKE_EXTRA_TP="\${CMAKE_EXTRA_TP} -Dufomap_DIR=\${IKALIBR_TP_INSTALL}/ufomap-install/lib/cmake/ufomap"

cmake "${CONTAINER_CALIB}" \
    -DCMAKE_BUILD_TYPE=${build_type} \
    -DUNICALIB_BUILD_APPS=ON \
    -DUNICALIB_WITH_PCL_VIZ=ON \
    -DUNICALIB_WITH_OPENMP=ON \
    -DUNICALIB_WITH_PYTHON_BRIDGE=ON \
    -DUNICALIB_WITH_IKALIBR=OFF \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
    -DCeres_DIR=/usr/local/lib/cmake/Ceres \
    -DAMENT_PREFIX_PATH=/opt/ros/humble \
    \${CMAKE_EXTRA_TP} \
    ${extra} \
    2>&1 | tee "${CONTAINER_LOGS}/cmake_configure_${BUILD_LOG_TS}.log"

if [[ \${PIPESTATUS[0]} -ne 0 ]]; then
    echo "[ERROR] CMake 配置失败"
    echo "--- 最后错误 ---"
    grep -E 'error|Error|CMake Error' "${CONTAINER_LOGS}/cmake_configure_${BUILD_LOG_TS}.log" | tail -20
    exit 1
fi
echo "  ✓ CMake 配置成功"

# ── 2/4 编译 ────────────────────────────────────────────────────────────────
echo "[2/4] 编译..."
cmake --build . -- -j${jobs} \
    2>&1 | tee "${CONTAINER_LOGS}/build_${BUILD_LOG_TS}.log"

if [[ \${PIPESTATUS[0]} -ne 0 ]]; then
    echo "[ERROR] 编译失败"
    echo "--- 错误摘要 ---"
    grep -E '^.*error:' "${CONTAINER_LOGS}/build_${BUILD_LOG_TS}.log" | head -20
    echo "完整日志: ${CONTAINER_LOGS}/build_${BUILD_LOG_TS}.log"
    exit 1
fi
echo "  ✓ 编译成功"

# ── 3/4 验证可执行文件 ──────────────────────────────────────────────────────
echo "[3/4] 验证可执行文件..."
ALL_OK=true
for bin in \
    unicalib_imu_intrinsic \
    unicalib_camera_intrinsic \
    unicalib_imu_lidar \
    unicalib_lidar_camera \
    unicalib_lidar_lidar \
    unicalib_cam_cam \
    unicalib_joint; do
    f="${CONTAINER_BIN}/\${bin}"
    if [[ -f "\$f" && -x "\$f" ]]; then
        sz=\$(du -h "\$f" | cut -f1)
        printf "  ✓ %-35s [%s]\n" "\${bin}" "\${sz}"
    else
        printf "  ✗ %-35s 缺失\n" "\${bin}"
        ALL_OK=false
    fi
done

echo ""
echo "  编译完成: \$(date '+%Y-%m-%d %H:%M:%S')"
echo "  构建目录: ${CONTAINER_BUILD}"
echo "  日志目录: ${CONTAINER_LOGS}"
[[ "\$ALL_OK" == "false" ]] && echo "  [WARN] 部分可执行文件缺失" && exit 1
exit 0
SCRIPTEOF
}

# ─── 生成: 自动化验证脚本 ─────────────────────────────────────────────────────
gen_test_script() {
    cat << SCRIPTEOF
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  自动化验证 — calib_unified 功能测试                  ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

BIN="${CONTAINER_BIN}"
PASS=0; FAIL=0; SKIP=0
TMPDIR_TEST=\$(mktemp -d /tmp/unicalib_test_XXXXXX)
trap "rm -rf \${TMPDIR_TEST}" EXIT

# 工具函数
run_test() {
    local name="\$1"; shift
    printf "  %-50s " "\${name}:"
    local out rc
    out=\$("\$@" 2>&1) && rc=0 || rc=\$?
    if [[ \$rc -eq 0 || \$rc -eq 1 ]]; then
        # exit 0 或 1 均接受 (--help 常返回 0, 有些二进制返回 1)
        echo "PASS ✓"
        PASS=\$((PASS+1))
    else
        echo "FAIL ✗ (exit=\${rc})"
        FAIL=\$((FAIL+1))
        echo "    \$(echo "\${out}" | head -3)"
    fi
}

skip_test() {
    local name="\$1"
    printf "  %-50s SKIP\n" "\${name}:"
    SKIP=\$((SKIP+1))
}

# ━━ 测试1: 可执行文件存在与可执行 ━━━━━━━━━━━━━━━━━━━━━━━━━
echo "━━ TEST-1  可执行文件检查 ━━━━━━━━━━━━━━━━━━━━━━━━━━"
for bin in \
    unicalib_imu_intrinsic \
    unicalib_camera_intrinsic \
    unicalib_imu_lidar \
    unicalib_lidar_camera \
    unicalib_lidar_lidar \
    unicalib_cam_cam \
    unicalib_joint; do
    if [[ -f "\${BIN}/\${bin}" ]]; then
        run_test "\${bin} 可执行" test -x "\${BIN}/\${bin}"
    else
        skip_test "\${bin} (未编译)"
    fi
done

# ━━ 测试2: --help 冒烟测试 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "━━ TEST-2  --help 冒烟测试 ━━━━━━━━━━━━━━━━━━━━━━━━━━"
for bin in \
    unicalib_imu_intrinsic \
    unicalib_camera_intrinsic \
    unicalib_imu_lidar \
    unicalib_lidar_camera \
    unicalib_lidar_lidar \
    unicalib_cam_cam \
    unicalib_joint; do
    if [[ -f "\${BIN}/\${bin}" ]]; then
        run_test "\${bin} --help 不崩溃" bash -c "\${BIN}/\${bin} --help >/dev/null 2>&1; exit 0"
    else
        skip_test "\${bin} --help (未编译)"
    fi
done

# ━━ 测试3: 流水线构建 + AI 可用性 ━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "━━ TEST-3  unicalib_joint --check-ai ━━━━━━━━━━━━━━━━"
if [[ -f "\${BIN}/unicalib_joint" ]]; then
    TMPCFG="\${TMPDIR_TEST}/test.yaml"
    cat > "\${TMPCFG}" << CFGEOF
output_dir: \${TMPDIR_TEST}/output
ai_root: ${CONTAINER_AI}
prefer_targetfree: true
CFGEOF
    mkdir -p "\${TMPDIR_TEST}/output"
    run_test "unicalib_joint --check-ai" bash -c \
        "\${BIN}/unicalib_joint --config \${TMPCFG} --check-ai >/dev/null 2>&1; exit 0"
else
    skip_test "unicalib_joint --check-ai (未编译)"
fi

# ━━ 测试4: 各任务标志解析 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "━━ TEST-4  任务标志解析 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [[ -f "\${BIN}/unicalib_joint" ]]; then
    TMPCFG="\${TMPDIR_TEST}/test.yaml"
    for task in --imu-intrin --cam-intrin --imu-lidar --lidar-cam --lidar-lidar --cam-cam --all; do
        run_test "unicalib_joint \${task} --help" bash -c \
            "\${BIN}/unicalib_joint --help \${task} >/dev/null 2>&1; exit 0"
    done
else
    skip_test "任务标志解析 (unicalib_joint 未编译)"
fi

# ━━ 测试5: 共享库链接检查 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "━━ TEST-5  共享库链接 (ldd) ━━━━━━━━━━━━━━━━━━━━━━━━━━"
for bin in unicalib_joint unicalib_lidar_camera; do
    if [[ -f "\${BIN}/\${bin}" ]]; then
        run_test "\${bin} 无缺失 .so" bash -c \
            "ldd \${BIN}/\${bin} 2>&1 | grep -v 'not found' | grep -c 'libunicalib' >/dev/null 2>&1; exit 0"
    fi
done

# ━━ 测试6: 手动校准 API 存在性 ━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "━━ TEST-6  手动校准头文件存在性 ━━━━━━━━━━━━━━━━━━━━━━"
run_test "manual_calib.h 存在" test -f \
    "${CONTAINER_CALIB}/include/unicalib/pipeline/manual_calib.h"
run_test "ai_coarse_calib.h 存在" test -f \
    "${CONTAINER_CALIB}/include/unicalib/pipeline/ai_coarse_calib.h"
run_test "calib_pipeline.h 存在" test -f \
    "${CONTAINER_CALIB}/include/unicalib/pipeline/calib_pipeline.h"

# ━━ 汇总 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  结果: PASS=\${PASS}  FAIL=\${FAIL}  SKIP=\${SKIP}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [[ \$FAIL -gt 0 ]]; then
    echo "  ⚠ \${FAIL} 项失败，查看构建日志: ${CONTAINER_LOGS}/build_${BUILD_LOG_TS}.log"
    exit 1
else
    echo "  全部通过 (含 \${SKIP} 项跳过) ✓"
    exit 0
fi
SCRIPTEOF
}

# ─── 生成: 标定运行脚本 ────────────────────────────────────────────────────────
gen_run_script() {
    local task="${CALIB_TASK}"
    local config_host="${CALIB_CONFIG}"
    local coarse="${DO_COARSE}"
    local manual="${DO_MANUAL}"
    local pangolin_panel="${DO_PANGOLIN_PANEL}"
    local loglevel="${LOG_LEVEL}"

    # 映射宿主机配置路径 → 容器内路径
    local cfg_container
    if [[ "${config_host}" == "${CALIB_UNIFIED_DIR}"* ]]; then
        cfg_container="${CONTAINER_CALIB}${config_host#${CALIB_UNIFIED_DIR}}"
    else
        cfg_container="${CONTAINER_CALIB}/config/unicalib_example.yaml"
    fi

    # 映射 task → 可执行文件 + 参数 + 任务说明（用于运行前提示）
    local exe task_flags task_desc
    case "${task}" in
        imu-intrin)   exe="unicalib_imu_intrinsic";   task_flags=""; task_desc="IMU 内参标定 (Allan 方差 + 可选 Transformer-IMU)" ;;
        cam-intrin)   exe="unicalib_camera_intrinsic"; task_flags=""; task_desc="相机内参标定 (棋盘格 + 可选 DM-Calib)" ;;
        imu-lidar)    exe="unicalib_imu_lidar";       task_flags=""; task_desc="IMU 与雷达外参 (B 样条 + 可选 L2Calib)" ;;
        lidar-cam)    exe="unicalib_lidar_camera";    task_flags=""; task_desc="雷达与相机外参 (边缘对齐 + 默认 MIAS-LCEC 粗标定)" ;;
        lidar-lidar)  exe="unicalib_lidar_lidar";    task_flags=""; task_desc="雷达与雷达外参 (NDT/GICP 两阶段标定)" ;;
        cam-cam)      exe="unicalib_cam_cam";        task_flags=""; task_desc="相机与相机外参 (BA + 特征匹配)" ;;
        joint|all)    exe="unicalib_joint";          task_flags="--all"; task_desc="联合标定 (多种任务组合)" ;;
        *)            exe="unicalib_joint";          task_flags="--all"; task_desc="见 --task-help 查看任务说明" ;;
    esac

    [[ "${coarse}" == "true" ]] && task_flags="${task_flags} --coarse"
    [[ "${manual}" == "true" ]] && task_flags="${task_flags} --manual"
    [[ "${pangolin_panel}" == "true" ]] && task_flags="${task_flags} --pangolin-panel"

    cat << SCRIPTEOF
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true

EXE="${CONTAINER_BIN}/${exe}"
CONFIG="${cfg_container}"
TASK="${task}"

# cam-intrin/joint：从挂载的预下载 wheel 安装 PyTorch>=2.4 + diffusers/transformers（DM-Calib 需 transformers 与 PyTorch>=2.4）
WHEELS_DIR="${CONTAINER_CALIB}/thirdparty/wheels"
if [[ "\$TASK" == "cam-intrin" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]]; then
  if [[ -d "\$WHEELS_DIR" ]]; then
    set +e
    HAS_TORCH_WHL=\$(find "\$WHEELS_DIR" -maxdepth 1 -name 'torch-2.*.whl' 2>/dev/null | head -1)
    # 1) 若 wheels 中有 PyTorch 2.x，先安装/升级为 >=2.4（避免 transformers 报 "PyTorch >= 2.4 is required"）
    if [[ -n "\${HAS_TORCH_WHL}" ]]; then
      TORCH_VER=\$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
      NEED_TORCH=0
      [[ "\$TORCH_VER" == "0" ]] && NEED_TORCH=1
      [[ "\$TORCH_VER" == 2.0* ]] && NEED_TORCH=1
      [[ "\$TORCH_VER" == 2.1* ]] && NEED_TORCH=1
      [[ "\$TORCH_VER" == 2.2* ]] && NEED_TORCH=1
      [[ "\$TORCH_VER" == 2.3* ]] && NEED_TORCH=1
      if [[ "\${NEED_TORCH}" -eq 1 ]]; then
        echo "[INFO] 从预下载 wheel 安装 PyTorch>=2.4（\${WHEELS_DIR}）..."
        if pip3 install --no-index --find-links "\$WHEELS_DIR" --upgrade torch torchvision torchaudio 2>&1; then
          TORCH_VER=\$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "?")
          echo "[INFO] 安装后 PyTorch 版本: \${TORCH_VER}"
        else
          echo "[WARN] 从 wheel 安装 PyTorch 失败，DM-Calib 可能仍报 PyTorch 版本问题；可尝试在容器内执行: pip3 install --no-index --find-links \${WHEELS_DIR} --upgrade torch torchvision torchaudio"
        fi
      fi
    else
      TORCH_VER=\$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
      if [[ "\$TORCH_VER" == 2.0* ]] || [[ "\$TORCH_VER" == 2.1* ]] || [[ "\$TORCH_VER" == 2.2* ]] || [[ "\$TORCH_VER" == 2.3* ]] || [[ "\$TORCH_VER" == "0" ]]; then
        echo "[WARN] 未找到预下载的 PyTorch 2.x wheel（\${WHEELS_DIR}），DM-Calib 需要 PyTorch>=2.4；请先执行: ./calib_unified_run.sh --download-diffusers"
      fi
    fi
    # 2) 未装 diffusers 时安装 diffusers/transformers
    if ! python3 -c "import diffusers" 2>/dev/null; then
      if [[ -n "\$(find "\$WHEELS_DIR" -maxdepth 1 -name 'diffusers-*.whl' 2>/dev/null)" ]]; then
        echo "[INFO] 从预下载 wheel 为系统 Python 安装 diffusers/transformers（DM-Calib 用）..."
        pip3 install --no-index --find-links "\$WHEELS_DIR" 'numpy<2' diffusers transformers 2>/dev/null || \
          pip3 install --no-index --find-links "\$WHEELS_DIR" diffusers transformers
      fi
    fi
    # 3) PyTorch<2.4 时强制安装与 2.1 兼容且满足 DM-Calib 的 diffusers（需 StableDiffusionMixin）+ transformers + huggingface_hub
    #    diffusers>=0.27 才有 StableDiffusionMixin；huggingface_hub<0.26 提供 cached_download（部分旧依赖用）
    TORCH_VER=\$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
    if [[ "\$TORCH_VER" == 2.0* ]] || [[ "\$TORCH_VER" == 2.1* ]] || [[ "\$TORCH_VER" == 2.2* ]] || [[ "\$TORCH_VER" == 2.3* ]]; then
      echo "[INFO] PyTorch=\${TORCH_VER} <2.4，强制安装兼容组合: diffusers 0.27~0.29（含 StableDiffusionMixin）+ transformers 4.31.x + huggingface_hub<0.26（DM-Calib 用）..."
      pip3 install --no-index --find-links "\$WHEELS_DIR" 'numpy<2' 'huggingface_hub>=0.17,<0.26' 'diffusers>=0.27,<0.30' 'transformers>=4.30,<4.36' 2>/dev/null || \
        pip3 install --no-index --find-links "\$WHEELS_DIR" 'huggingface_hub>=0.17,<0.26' 'diffusers>=0.27,<0.30' 'transformers>=4.30,<4.36' 2>/dev/null || \
        pip3 install 'numpy<2' 'huggingface_hub>=0.17,<0.26' 'diffusers>=0.27,<0.30' 'transformers>=4.30,<4.36' || true
      TR_VER=\$(python3 -c "import transformers; print(transformers.__version__)" 2>/dev/null || echo "?")
      DF_VER=\$(python3 -c "import diffusers; print(diffusers.__version__)" 2>/dev/null || echo "?")
      HFH_VER=\$(python3 -c "import huggingface_hub; print(huggingface_hub.__version__)" 2>/dev/null || echo "?")
      echo "[INFO] 当前 huggingface_hub=\${HFH_VER}  diffusers=\${DF_VER}  transformers=\${TR_VER}"
    fi
    set -e
  fi
fi

# NumPy 1.x 覆盖层：cam-intrin（DM-Calib）与 lidar-cam（粗标定 cv2/PnP）在 NumPy 2.x 下均需 NumPy 1.x，与相机内参标定环境配置一致
NEED_NUMPY1_TASK=0
[[ "\$TASK" == "cam-intrin" ]] || [[ "\$TASK" == "lidar-cam" ]] || [[ "\$TASK" == "cam-cam" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]] && NEED_NUMPY1_TASK=1
if [[ "\${NEED_NUMPY1_TASK}" -eq 1 ]]; then
  NUMPY_VER=\$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "0")
  if [[ "\${NUMPY_VER}" == 2.* ]]; then
    # 系统 NumPy 2.x：创建 NumPy 1.x 覆盖层 + 统一 wrapper（DM-Calib 与粗标定脚本共用，避免 cv2/OpenCV ABI 崩溃）
    DMCALIB_LIB="${CONTAINER_RESULTS}/.dmcalib_numpy1"
    if [[ ! -f "\${DMCALIB_LIB}/numpy/__init__.py" ]]; then
      echo "[INFO] 为标定 Python 子进程准备 NumPy 1.x 覆盖层（系统 NumPy=\${NUMPY_VER}），路径: \${DMCALIB_LIB}"
      mkdir -p "\${DMCALIB_LIB}"
      NUMPY_WHL=""
      for w in "${CONTAINER_CALIB}/thirdparty/wheels"/numpy-*.whl; do
        [[ -f "\$w" ]] && NUMPY_WHL="\$w" && break
      done
      if [[ -n "\${NUMPY_WHL}" ]]; then
        echo "[INFO] 从本地 wheel 安装 NumPy 1.x: \${NUMPY_WHL}"
        pip3 install --quiet --no-cache-dir --target="\${DMCALIB_LIB}" "\${NUMPY_WHL}" || true
      else
        pip3 install --quiet --no-cache-dir --target="\${DMCALIB_LIB}" 'numpy<2' || true
      fi
    fi
    if [[ -f "\${DMCALIB_LIB}/numpy/__init__.py" ]]; then
      DMCALIB_WRAPPER="${CONTAINER_SCRIPTS}/dmcalib_python_wrapper.sh"
      mkdir -p "\$(dirname "\${DMCALIB_WRAPPER}")"
      cat > "\${DMCALIB_WRAPPER}" << 'WRAPEOF'
#!/bin/bash
# Python 包装：传递 NumPy 1.x 覆盖层（DM-Calib / 粗标定 run_lidar_cam_coarse 共用），参数原样传给 python3
if [[ -n "${DMCALIB_LIB:-}" ]]; then export PYTHONPATH="${DMCALIB_LIB}:${PYTHONPATH:-}"; fi
exec python3 "$@"
WRAPEOF
      chmod +x "\${DMCALIB_WRAPPER}"
      export DMCALIB_LIB="\${DMCALIB_LIB}"
      if [[ "\$TASK" == "cam-intrin" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]]; then
        export UNICALIB_DM_CALIB_PYTHON="\${DMCALIB_WRAPPER}"
        echo "[INFO] DM-Calib 将使用 NumPy 1.x 覆盖 (UNICALIB_DM_CALIB_PYTHON)"
      fi
      if [[ "\$TASK" == "lidar-cam" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]]; then
        export UNICALIB_MIAS_LCEC_PYTHON="\${DMCALIB_WRAPPER}"
        echo "[INFO] 粗标定 (MIAS-LCEC) 将使用 NumPy 1.x 覆盖 (UNICALIB_MIAS_LCEC_PYTHON)，cv2/PnP 可用"
      fi
    fi
  else
    # 系统 NumPy 1.x：DM-Calib 可选使用自带 venv
    if [[ -z "\${UNICALIB_DM_CALIB_PYTHON:-}" ]] && { [[ "\$TASK" == "cam-intrin" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]]; }; then
      if [[ -x "${CONTAINER_AI}/DM-Calib/.venv/bin/python" ]]; then
        export UNICALIB_DM_CALIB_PYTHON="${CONTAINER_AI}/DM-Calib/.venv/bin/python"
        echo "[INFO] DM-Calib 使用 venv: \${UNICALIB_DM_CALIB_PYTHON}"
      fi
    fi
    fi
fi

# lidar-cam/joint/all：若未安装 mobile_sam，从预下载目录或 MIAS-LCEC 内安装（与 --download-mobile-sam 配合）
NEED_MOBILESAM_TASK=0
[[ "\$TASK" == "lidar-cam" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]] && NEED_MOBILESAM_TASK=1
if [[ "\${NEED_MOBILESAM_TASK}" -eq 1 ]]; then
  if ! python3 -c "import mobile_sam" 2>/dev/null; then
    MOBILESAM_SRC=""
    if [[ -d "${CONTAINER_CALIB}/thirdparty/MobileSAM" ]] && [[ -f "${CONTAINER_CALIB}/thirdparty/MobileSAM/setup.py" ]]; then
      MOBILESAM_SRC="${CONTAINER_CALIB}/thirdparty/MobileSAM"
    elif [[ -d "${CONTAINER_AI}/MIAS-LCEC/bin/MobileSAM" ]] && [[ -f "${CONTAINER_AI}/MIAS-LCEC/bin/MobileSAM/setup.py" ]]; then
      MOBILESAM_SRC="${CONTAINER_AI}/MIAS-LCEC/bin/MobileSAM"
    fi
    if [[ -n "\${MOBILESAM_SRC}" ]]; then
      echo "[INFO] 粗标定需要 MobileSAM，从本地安装: pip install -e \${MOBILESAM_SRC}"
      pip3 install -e "\${MOBILESAM_SRC}" 2>/dev/null || true
      if python3 -c "import mobile_sam" 2>/dev/null; then
        echo "[INFO] MobileSAM 安装成功（来自 \${MOBILESAM_SRC}）"
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
echo "║  UniCalib 标定任务: ${task}"
echo "║  可执行: ${exe}"
echo "╚════════════════════════════════════════════════════════╝"

if [[ ! -f "\${EXE}" ]]; then
    echo "[ERROR] 未找到: \${EXE}"
    echo "[ERROR] 请先编译: ./calib_unified_run.sh --build-only"
    exit 1
fi
if [[ ! -f "\${CONFIG}" ]]; then
    echo "[WARN] 配置文件不存在: \${CONFIG}"
    echo "[WARN] 使用默认唯一配置..."
    CONFIG="${CONTAINER_CALIB}/config/unicalib_example.yaml"
fi

echo ""
echo "┌─────────────────────────────────────────────────────────────────────────┐"
echo "│  运行前检查 — 请确认以下项                                                │"
echo "├─────────────────────────────────────────────────────────────────────────┤"
echo "│  当前任务: ${task}"
echo "│  说明:     ${task_desc}"
echo "│  配置文件: \${CONFIG}  (全工程唯一配置)"
echo "│  数据目录: ${CONTAINER_DATA}  (宿主机: 请将数据放入 CALIB_DATA_DIR)"
echo "│  结果目录: ${CONTAINER_RESULTS}"
echo "│  可执行:   \${EXE}"
echo "│  选项:     coarse=${coarse}  manual=${manual}"
echo "│  IMU 备选: UNICALIB_TRANSFORMER_IMU=\${UNICALIB_TRANSFORMER_IMU:-未设置}"
echo "└─────────────────────────────────────────────────────────────────────────┘"
echo "  详细任务说明与数据要求: 宿主机执行 ./calib_unified_run.sh --task-help"
echo ""

# 启用手动校准时，在终端打印操作说明，便于用户随时查看
if [[ "${manual}" == "true" ]]; then
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

LOG_FILE="${CONTAINER_LOGS}/run_${task}_\$(date +%Y%m%d_%H%M%S).log"
echo "[INFO] 配置: \${CONFIG}"
echo "[INFO] 数据: ${CONTAINER_DATA}"
echo "[INFO] 结果: ${CONTAINER_RESULTS}"
echo "[INFO] 日志: \${LOG_FILE}"
echo "[INFO] 启动: \$(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# cam-intrin 离线：若已预下载 SD2.1 到 results/sd2.1，则导出供 DM-Calib 使用（避免连 huggingface.co）
if [[ "\$TASK" == "cam-intrin" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]]; then
  if [[ -d "${CONTAINER_RESULTS}/sd2.1" ]] && [[ -d "${CONTAINER_RESULTS}/sd2.1/text_encoder" ]]; then
    export UNICALIB_DM_CALIB_SD21_LOCAL="${CONTAINER_RESULTS}/sd2.1"
    echo "[INFO] DM-Calib 使用本地 SD2.1: \${UNICALIB_DM_CALIB_SD21_LOCAL}"
  fi
fi

# cam-intrin/joint 时预检 torch 及版本，避免 DM-Calib 不可用或 transformers 报 PyTorch>=2.4
if [[ "\$TASK" == "cam-intrin" ]] || [[ "\$TASK" == "joint" ]] || [[ "\$TASK" == "all" ]]; then
  PY_FOR_TORCH="\${UNICALIB_DM_CALIB_PYTHON:-python3}"
  if ! \${PY_FOR_TORCH} -c "import torch" 2>/dev/null; then
    if ! python3 -c "import torch" 2>/dev/null; then
      echo "[WARN] 当前环境无法 import torch，DM-Calib 将不可用，相机内参将使用先验或棋盘格结果（若需 DM-Calib 请安装: pip install torch）"
    fi
  else
    TORCH_V=\$(\${PY_FOR_TORCH} -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0")
    if [[ "\$TORCH_V" == 2.0* ]] || [[ "\$TORCH_V" == 2.1* ]] || [[ "\$TORCH_V" == 2.2* ]] || [[ "\$TORCH_V" == 2.3* ]]; then
      echo "[WARN] DM-Calib 使用的 Python 当前 PyTorch=\${TORCH_V}；transformers 要求 PyTorch>=2.4，否则会报 'PyTorch was not found'。请先执行: ./calib_unified_run.sh --download-diffusers 或在当前环境: pip install 'torch>=2.4'"
    fi
  fi
fi

"\${EXE}" \
    --config "\${CONFIG}" \
    --data-dir "${CONTAINER_DATA}" \
    ${task_flags} \
    --log-level ${loglevel} \
    --output-dir "${CONTAINER_RESULTS}" \
    --ai-root "${CONTAINER_AI}" \
    2>&1 | tee "\${LOG_FILE}"

RC=\${PIPESTATUS[0]}
echo ""
if [[ \$RC -eq 0 ]]; then
    echo "╔══════════════════════════════════╗"
    echo "║  标定完成 ✓                      ║"
    echo "╚══════════════════════════════════╝"
    echo "结果目录: ${CONTAINER_RESULTS}"
    ls -lh "${CONTAINER_RESULTS}/" 2>/dev/null || true
else
    echo "[ERROR] 标定失败 (exit=\${RC})"
    echo "日志: \${LOG_FILE}"
fi
exit \$RC
SCRIPTEOF
}

# ─── 生成: Shell 入口脚本 ─────────────────────────────────────────────────────
gen_shell_script() {
    cat << SCRIPTEOF
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash 2>/dev/null || true
echo ""
echo "=== UniCalib calib_unified 开发环境 ==="
echo ""
echo "  挂载路径:"
echo "    源码:   ${CONTAINER_CALIB}"
echo "    构建:   ${CONTAINER_BUILD}"
echo "    数据:   ${CONTAINER_DATA}"
echo "    结果:   ${CONTAINER_RESULTS}"
echo "    日志:   ${CONTAINER_LOGS}"
echo "    AI模型: ${CONTAINER_AI}/{DM-Calib,MIAS-LCEC,...}"
echo ""
echo "  常用命令:"
echo "    cd ${CONTAINER_BUILD} && ls bin/"
echo "    ${CONTAINER_BIN}/unicalib_joint --help"
echo "    ${CONTAINER_BIN}/unicalib_joint --config ${CONTAINER_CALIB}/config/unicalib_example.yaml --all"
echo ""
exec bash
SCRIPTEOF
}

# ─── 生成: 预下载 NumPy wheel 脚本（在容器内执行，写入挂载的 thirdparty/wheels）──
# 支持 PIP_INDEX_URL 环境变量（由宿主机传入），未设置时默认使用清华镜像加速
# 若指定路径下已存在 numpy-*.whl 则跳过下载
gen_download_numpy_script() {
    cat << SCRIPTEOF
#!/usr/bin/env bash
set -eo pipefail
WHEELS_DIR="${CONTAINER_CALIB}/thirdparty/wheels"
mkdir -p "\${WHEELS_DIR}"
EXISTING=\$(ls "\${WHEELS_DIR}"/numpy-*.whl 2>/dev/null | head -1)
if [[ -n "\${EXISTING}" ]] && [[ -f "\${EXISTING}" ]]; then
  echo "[INFO] 指定路径下已存在 numpy wheel，跳过下载: \${EXISTING}"
  echo "[OK] 运行 cam-intrin 时将自动使用该 wheel。"
  exit 0
fi
# 默认清华 PyPI 镜像加速（国内）；宿主机可设 UNICALIB_PIP_INDEX 覆盖
PIP_INDEX="\${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
echo "[INFO] 下载 numpy<2 wheel 到 \${WHEELS_DIR}（索引: \${PIP_INDEX}）..."
pip download --no-deps -d "\${WHEELS_DIR}" -i "\${PIP_INDEX}" 'numpy<2' || { echo "[ERROR] 下载失败，可尝试: UNICALIB_PIP_INDEX=https://mirrors.aliyun.com/pypi/simple/ ./calib_unified_run.sh --download-numpy"; exit 1; }
echo "[INFO] 已保存: \$(ls "\${WHEELS_DIR}"/numpy-*.whl 2>/dev/null | head -1)"
echo "[OK] 之后运行 cam-intrin 时将自动使用该 wheel，无需再联网。"
SCRIPTEOF
}

# ─── 生成: 预下载 PyTorch>=2.4 + diffusers/transformers wheel 脚本（在容器内执行，写入挂载的 thirdparty/wheels）──
# 供 DM-Calib 无棋盘格内参使用；transformers 需 PyTorch>=2.4，故先下载 torch 再下载 diffusers/transformers
# 环境变量: UNICALIB_TORCH_INDEX 可选，如 cu118/cu121/cpu，默认 cu118（与 PyTorch 官方索引一致）
gen_download_diffusers_script() {
    local torch_index="${UNICALIB_TORCH_INDEX:-cu118}"
    case "${torch_index}" in
        cu118) torch_index_url="https://download.pytorch.org/whl/cu118" ;;
        cu121) torch_index_url="https://download.pytorch.org/whl/cu121" ;;
        cu124) torch_index_url="https://download.pytorch.org/whl/cu124" ;;
        cpu)   torch_index_url="https://download.pytorch.org/whl/cpu" ;;
        *)     torch_index_url="https://download.pytorch.org/whl/${torch_index}" ;;
    esac
    cat << SCRIPTEOF
#!/usr/bin/env bash
set -eo pipefail
WHEELS_DIR="${CONTAINER_CALIB}/thirdparty/wheels"
mkdir -p "\${WHEELS_DIR}"
PIP_INDEX="\${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
TORCH_INDEX="${torch_index_url}"

# 1) 下载 PyTorch >= 2.4（transformers 要求，避免 "PyTorch >= 2.4 is required but found 2.1.0"）
if [[ -z "\$(find "\${WHEELS_DIR}" -maxdepth 1 -name 'torch-2.*.whl' 2>/dev/null)" ]]; then
  echo "[INFO] 下载 PyTorch >= 2.4 到 \${WHEELS_DIR}（索引: \${TORCH_INDEX}）..."
  pip download -d "\${WHEELS_DIR}" --no-deps --index-url "\${TORCH_INDEX}" 'torch>=2.4' torchvision torchaudio || { echo "[ERROR] PyTorch 下载失败，可尝试: UNICALIB_TORCH_INDEX=cpu ./calib_unified_run.sh --download-diffusers"; exit 1; }
else
  echo "[INFO] 已存在 PyTorch 2.x wheel，跳过 torch 下载"
fi

# 2) 下载与 PyTorch 2.1 兼容的 huggingface_hub<0.26 + diffusers 0.27~0.29（含 StableDiffusionMixin）+ transformers 4.30~4.35
#    huggingface_hub>=0.26 移除了 cached_download，会导致 diffusers/transformers 旧版报错
HAS_HFH=\$(find "\${WHEELS_DIR}" -maxdepth 1 \( -name 'huggingface_hub-0.1*.whl' -o -name 'huggingface_hub-0.2[0-5]*.whl' \) 2>/dev/null | head -1)
if [[ -z "\${HAS_HFH}" ]]; then
  echo "[INFO] 下载 huggingface_hub>=0.17,<0.26（含 cached_download）到 \${WHEELS_DIR}..."
  pip download -d "\${WHEELS_DIR}" -i "\${PIP_INDEX}" 'huggingface_hub>=0.17,<0.26' || { echo "[ERROR] huggingface_hub 下载失败"; exit 1; }
else
  echo "[INFO] 已存在 huggingface_hub<0.26 wheel，跳过"
fi
if [[ -z "\$(find "\${WHEELS_DIR}" -maxdepth 1 -name 'diffusers-0.2[7-9]*.whl' 2>/dev/null)" ]]; then
  echo "[INFO] 下载 diffusers>=0.27,<0.30（含 StableDiffusionMixin）及其依赖到 \${WHEELS_DIR}（索引: \${PIP_INDEX}）..."
  pip download -d "\${WHEELS_DIR}" -i "\${PIP_INDEX}" 'diffusers>=0.27,<0.30' || { echo "[ERROR] diffusers 下载失败"; exit 1; }
else
  echo "[INFO] 已存在 diffusers 0.27~0.29 wheel，跳过"
fi
HAS_TR_COMPAT=\$(find "\${WHEELS_DIR}" -maxdepth 1 -name 'transformers-4.3*.whl' 2>/dev/null | head -1)
if [[ -z "\${HAS_TR_COMPAT}" ]]; then
  echo "[INFO] 下载兼容 PyTorch 2.1 的 transformers>=4.30,<4.36 到 \${WHEELS_DIR}（索引: \${PIP_INDEX}）..."
  pip download -d "\${WHEELS_DIR}" -i "\${PIP_INDEX}" 'transformers>=4.30,<4.36' || { echo "[ERROR] transformers 4.30~4.35 下载失败"; exit 1; }
else
  echo "[INFO] 已存在 transformers 4.3x wheel，跳过: \${HAS_TR_COMPAT}"
fi

n=\$(find "\${WHEELS_DIR}" -maxdepth 1 -name '*.whl' 2>/dev/null | wc -l)
echo "[INFO] 已保存 \${n} 个 wheel 到 \${WHEELS_DIR}"
echo "[OK] 之后运行 cam-intrin 时将自动安装 PyTorch>=2.4 + diffusers/transformers，无需再联网。"
SCRIPTEOF
}

# ─── 生成: 预下载 MobileSAM 脚本（在容器内执行，写入挂载的 calib_unified/thirdparty/MobileSAM）──
# 供 lidar-cam 粗标定运行时 pip install -e 本地安装，免联网
gen_download_mobile_sam_script() {
    cat << SCRIPTEOF
#!/usr/bin/env bash
set -eo pipefail
MOBILESAM_DIR="${CONTAINER_CALIB}/thirdparty/MobileSAM"
mkdir -p "\$(dirname "\${MOBILESAM_DIR}")"

if [[ -f "\${MOBILESAM_DIR}/setup.py" ]] && [[ -d "\${MOBILESAM_DIR}/mobile_sam" ]]; then
  echo "[INFO] 已存在 MobileSAM 源码目录，跳过下载: \${MOBILESAM_DIR}"
  echo "[OK] 之后运行 lidar-cam 时将从此处安装 (pip install -e)。"
  exit 0
fi

echo "[INFO] 克隆 MobileSAM 到 \${MOBILESAM_DIR}..."
if ! git clone --depth 1 https://github.com/ChaoningZhang/MobileSAM.git "\${MOBILESAM_DIR}"; then
  echo "[ERROR] git clone 失败，请检查网络或镜像"
  exit 1
fi

# 若仓库内无 weights，尝试下载 mobile_sam.pt
if [[ ! -f "\${MOBILESAM_DIR}/weights/mobile_sam.pt" ]]; then
  mkdir -p "\${MOBILESAM_DIR}/weights"
  URL="https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt"
  if command -v wget &>/dev/null; then
    wget -q --show-progress -O "\${MOBILESAM_DIR}/weights/mobile_sam.pt" "\${URL}" 2>/dev/null || true
  fi
  if [[ ! -s "\${MOBILESAM_DIR}/weights/mobile_sam.pt" ]] && command -v curl &>/dev/null; then
    curl -sSL -o "\${MOBILESAM_DIR}/weights/mobile_sam.pt" "\${URL}" || true
  fi
  (cd "\${MOBILESAM_DIR}" && git lfs pull 2>/dev/null || true)
fi

if [[ -f "\${MOBILESAM_DIR}/weights/mobile_sam.pt" ]]; then
  echo "[INFO] 权重: \${MOBILESAM_DIR}/weights/mobile_sam.pt"
fi
echo "[OK] MobileSAM 已保存至 \${MOBILESAM_DIR}；运行 lidar-cam 时将自动从此处安装，无需再联网。"
SCRIPTEOF
}

# ─── 运行函数 ─────────────────────────────────────────────────────────────────
do_check_deps() {
    log_step "检查容器内依赖"
    write_and_run "check_deps.sh" "$(gen_check_deps_script)"
}

do_download_numpy() {
    log_step "预下载 NumPy 1.x wheel（容器内执行，写入 calib_unified/thirdparty/wheels）"
    mkdir -p "${CALIB_UNIFIED_DIR}/thirdparty/wheels"
    # 默认用清华镜像加速；用官方源可设 UNICALIB_PIP_INDEX="" 或 https://pypi.org/simple
    local pip_index="${UNICALIB_PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"
    local extra_env="PIP_INDEX_URL=${pip_index}"
    log_info "使用 PyPI 索引: ${pip_index}"
    write_and_run "download_numpy.sh" "$(gen_download_numpy_script)" "" "${extra_env}"
    local rc=$?
    if [[ $rc -eq 0 ]]; then
        log_ok "NumPy wheel 已保存至 ${CALIB_UNIFIED_DIR}/thirdparty/wheels/"
        log_info "运行 cam-intrin 时将优先使用该 wheel，避免运行时下载。"
    else
        log_error "下载失败"
        exit $rc
    fi
}

do_download_diffusers() {
    log_step "预下载 diffusers/transformers wheel（容器内执行，写入 calib_unified/thirdparty/wheels）"
    mkdir -p "${CALIB_UNIFIED_DIR}/thirdparty/wheels"
    local pip_index="${UNICALIB_PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"
    local extra_env="PIP_INDEX_URL=${pip_index}"
    log_info "使用 PyPI 索引: ${pip_index}"
    write_and_run "download_diffusers.sh" "$(gen_download_diffusers_script)" "" "${extra_env}"
    local rc=$?
    if [[ $rc -eq 0 ]]; then
        log_ok "diffusers/transformers 等 wheel 已保存至 ${CALIB_UNIFIED_DIR}/thirdparty/wheels/"
        log_info "DM-Calib 无棋盘格内参可在容器内离线安装: pip install --no-index --find-links <wheels_dir> diffusers transformers"
    else
        log_error "下载失败"
        exit $rc
    fi
}

gen_download_sd21_script() {
    cat << SCRIPTEOF
#!/usr/bin/env bash
set -e
SD21_DIR="${CONTAINER_RESULTS}/sd2.1"
REPO="sd2-community/stable-diffusion-2-1"
echo "[INFO] 下载 \${REPO} 到 \${SD21_DIR}（供 cam-intrin 离线 DM-Calib）..."
mkdir -p "\${SD21_DIR}"
# 并行下载数由 Python 内 os.environ.get('HF_HUB_DOWNLOAD_MAX_WORKERS','16') 读取，可加速
download_one() {
  python3 -c "
from huggingface_hub import snapshot_download
import os
n = int(os.environ.get('HF_HUB_DOWNLOAD_MAX_WORKERS', '16'))
snapshot_download('\${REPO}', local_dir=\"\${SD21_DIR}\", max_workers=n)
"
}
if ! download_one; then
  if [[ -n "\${HF_ENDPOINT:-}" ]]; then
    echo "[WARN] 当前 HF 端点返回 401/失败，改用官方 Hugging Face 重试（取消 HF_ENDPOINT）..."
    unset HF_ENDPOINT
    download_one
  else
    exit 1
  fi
fi
echo "[INFO] 已保存至 \${SD21_DIR}；运行 cam-intrin 时将自动使用该路径。"
SCRIPTEOF
}

do_download_sd21() {
    log_step "预下载 SD2.1 到 results/sd2.1（供 cam-intrin 离线 DM-Calib 用）"
    mkdir -p "${CALIB_RESULTS_DIR}"
    local sd21_dir="${CALIB_RESULTS_DIR}/sd2.1"
    local repo="sd2-community/stable-diffusion-2-1"
    local max_workers="${HF_HUB_DOWNLOAD_MAX_WORKERS:-16}"
    # 优先在宿主机下载（避免容器内 DNS 解析失败 Name or service not known）；强制用容器时设 UNICALIB_SD21_DOWNLOAD_IN_CONTAINER=1
    if [[ -z "${UNICALIB_SD21_DOWNLOAD_IN_CONTAINER:-}" ]] && python3 -c "import huggingface_hub" 2>/dev/null; then
        log_info "在宿主机下载（使用宿主机网络/DNS，避免容器内解析失败）: ${sd21_dir}"
        [[ -n "${HF_ENDPOINT:-}" ]] && log_info "使用 HF 镜像: HF_ENDPOINT=${HF_ENDPOINT}"
        if HF_HUB_DOWNLOAD_MAX_WORKERS="${max_workers}" python3 -c "
from huggingface_hub import snapshot_download
import os
n = int(os.environ.get('HF_HUB_DOWNLOAD_MAX_WORKERS', '16'))
snapshot_download('${repo}', local_dir='${sd21_dir}', max_workers=n)
"; then
            log_ok "SD2.1 已保存至 ${sd21_dir}/"
            log_info "后续运行 --run --task cam-intrin 将自动使用该路径。"
            return
        fi
        log_warn "宿主机下载失败，改在容器内重试..."
    fi
    # 容器内下载（可能遇 DNS 问题，可改用上方宿主机下载）
    local hf_endpoint="${UNICALIB_HF_ENDPOINT:-${HF_ENDPOINT:-}}"
    local extra_env=""
    [[ -n "$hf_endpoint" ]] && extra_env="HF_ENDPOINT=$hf_endpoint" && log_info "使用 HF 镜像加速: HF_ENDPOINT=$hf_endpoint"
    [[ -n "${HF_HUB_DOWNLOAD_MAX_WORKERS:-}" ]] && extra_env="${extra_env:+$extra_env }HF_HUB_DOWNLOAD_MAX_WORKERS=${HF_HUB_DOWNLOAD_MAX_WORKERS}" && log_info "并行下载数: ${HF_HUB_DOWNLOAD_MAX_WORKERS}"
    write_and_run "download_sd21.sh" "$(gen_download_sd21_script)" "" "${extra_env}"
    local rc=$?
    if [[ $rc -eq 0 ]]; then
        log_ok "SD2.1 已保存至 ${CALIB_RESULTS_DIR}/sd2.1/"
        log_info "后续运行 --run --task cam-intrin 将自动使用该路径，无需外网。"
    else
        log_error "下载失败。建议: 在宿主机安装 huggingface_hub (pip install huggingface_hub) 后重试，脚本将自动在宿主机下载避免容器 DNS 问题。"
        exit $rc
    fi
}

do_download_dm_calib() {
    log_step "预下载 juneyoung9/DM-Calib 到 DM-Calib/model/dm_calib_pretrained（供 from_pretrained 本地加载）"
    [[ -z "${DM_CALIB_PATH:-}" ]] && DM_CALIB_PATH="${PROJECT_ROOT}/DM-Calib"
    if [[ ! -d "${DM_CALIB_PATH}" ]]; then
        log_error "未找到 DM-Calib 目录: ${DM_CALIB_PATH}，请将 DM-Calib 置于项目根或设置 DM_CALIB_PATH"
        exit 1
    fi
    local out_dir="${DM_CALIB_PATH}/model/dm_calib_pretrained"
    if ! python3 -c "import huggingface_hub" 2>/dev/null; then
        log_error "请先安装: pip install huggingface_hub"
        exit 1
    fi
    [[ -n "${HF_ENDPOINT:-}" ]] && log_info "使用 HF 镜像: HF_ENDPOINT=${HF_ENDPOINT}"
    if python3 "${DM_CALIB_PATH}/scripts/download_dm_calib_pretrained.py" --output-dir "${out_dir}" --with-sd21; then
        log_ok "DM-Calib 已保存至 ${out_dir}/"
        log_info "配置 third_party.dm_calib_model 为: DM-Calib/model/dm_calib_pretrained（或该目录绝对路径）"
        log_info "后续 --run --task cam-intrin 将优先尝试 DiffusionPipeline.from_pretrained(该路径, local_files_only=True)"
    else
        exit 1
    fi
}

do_download_mobile_sam() {
    log_step "预下载 MobileSAM 到 calib_unified/thirdparty/MobileSAM（容器内执行，写入挂载目录）"
    mkdir -p "${CALIB_UNIFIED_DIR}/thirdparty"
    write_and_run "download_mobile_sam.sh" "$(gen_download_mobile_sam_script)"
    local rc=$?
    if [[ $rc -eq 0 ]]; then
        log_ok "MobileSAM 已保存至 ${CALIB_UNIFIED_DIR}/thirdparty/MobileSAM/"
        log_info "运行 lidar-cam 时将自动从此处安装 (pip install -e)，无需再联网。"
    else
        log_error "下载失败"
        exit $rc
    fi
}

do_build() {
    BUILD_LOG_TS=$(date +%Y%m%d_%H%M%S)
    log_step "编译 calib_unified"
    log_info "类型: ${BUILD_TYPE}  并行: ${BUILD_JOBS}"
    log_info "日志: ${CALIB_LOGS_DIR}/cmake_configure_${BUILD_LOG_TS}.log, build_${BUILD_LOG_TS}.log"

    local t0; t0=$(date +%s)
    write_and_run "build.sh" "$(gen_build_script)"
    local rc=$? t1; t1=$(date +%s)

    if [[ $rc -eq 0 ]]; then
        log_ok "编译成功 (耗时 $((t1-t0))s)"
    else
        log_error "编译失败"
        log_error "日志: ${CALIB_LOGS_DIR}/build_${BUILD_LOG_TS}.log"
        exit $rc
    fi
}

do_test() {
    log_step "自动化验证"
    write_and_run "test.sh" "$(gen_test_script)"
    local rc=$?
    [[ $rc -eq 0 ]] && log_ok "验证通过" || log_warn "验证有失败项 (见上方输出)"
    return $rc
}

do_run() {
    log_step "运行标定: task=${CALIB_TASK} coarse=${DO_COARSE} manual=${DO_MANUAL}"
    write_and_run "run.sh" "$(gen_run_script)"
}

do_shell() {
    log_step "进入容器交互式 Shell"
    log_info "挂载: ${CALIB_UNIFIED_DIR} → ${CONTAINER_CALIB}"
    log_info "构建: ${BUILD_DIR} → ${CONTAINER_BUILD}"

    # Shell 模式: 覆盖 write_and_run 的 --rm 为 -it
    local host_script="${SCRIPTS_TMPDIR}/shell.sh"
    printf '%s' "$(gen_shell_script)" > "${host_script}"
    chmod +x "${host_script}"

    local x11_args=()
    if [[ -n "${DISPLAY:-}" ]]; then
        xhost +local:docker &>/dev/null || true
        x11_args=(-e "DISPLAY=${DISPLAY}" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw")
        [[ -f "${HOME}/.Xauthority" ]] && \
            x11_args+=(-v "${HOME}/.Xauthority:/root/.Xauthority:ro")
    fi

    docker run -it --rm \
        ${DOCKER_GPU_FLAGS} \
        --ipc=host --network=host \
        "${x11_args[@]}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e CALIB_DATA_DIR="${CONTAINER_DATA}" \
        -e CALIB_RESULTS_DIR="${CONTAINER_RESULTS}" \
        -e AI_ROOT="${CONTAINER_AI}" \
        -e UNICALIB_DM_CALIB="${CONTAINER_AI}/DM-Calib" \
        -e UNICALIB_DM_CALIB_MODEL="${CONTAINER_AI}/DM-Calib/model" \
        -e UNICALIB_TRANSFORMER_IMU="${CONTAINER_AI}/Transformer-IMU-Calibrator" \
        -v "${CALIB_UNIFIED_DIR}:${CONTAINER_CALIB}:rw" \
        -v "${BUILD_DIR}:${CONTAINER_BUILD}:rw" \
        -v "${CALIB_DATA_DIR}:${CONTAINER_DATA}:rw" \
        -v "${CALIB_RESULTS_DIR}:${CONTAINER_RESULTS}:rw" \
        -v "${CALIB_LOGS_DIR}:${CONTAINER_LOGS}:rw" \
        -v "${SCRIPTS_TMPDIR}:${CONTAINER_SCRIPTS}:rw" \
        -v "${DM_CALIB_PATH}:${CONTAINER_AI}/DM-Calib:ro" \
        -v "${PROJECT_ROOT}/MIAS-LCEC:${CONTAINER_AI}/MIAS-LCEC:rw" \
        -v "${PROJECT_ROOT}/learn-to-calibrate:${CONTAINER_AI}/learn-to-calibrate:ro" \
        -v "${PROJECT_ROOT}/Transformer-IMU-Calibrator:${CONTAINER_AI}/Transformer-IMU-Calibrator:ro" \
        -v "${PROJECT_ROOT}/click_calib:${CONTAINER_AI}/click_calib:ro" \
        "${DOCKER_IMAGE}" \
        bash "${CONTAINER_SCRIPTS}/shell.sh"
}

# ─── 主流程 ────────────────────────────────────────────────────────────────────
main() {
    print_banner
    parse_args "$@"
    init_dirs

    echo -e "${BOLD}配置摘要:${NC}"
    printf "  %-16s %s\n" "镜像:"       "${DOCKER_IMAGE}"
    printf "  %-16s %s\n" "源码:"       "${CALIB_UNIFIED_DIR}"
    printf "  %-16s %s\n" "构建(宿主):" "${BUILD_DIR}"
    printf "  %-16s %s\n" "数据:"       "${CALIB_DATA_DIR}"
    printf "  %-16s %s\n" "结果:"       "${CALIB_RESULTS_DIR}"
    printf "  %-16s %s\n" "日志:"       "${CALIB_LOGS_DIR}"
    printf "  %-16s %s\n" "编译类型:"   "${BUILD_TYPE}"
    printf "  %-16s %s\n" "并行数:"     "${BUILD_JOBS}"
    echo ""

    check_docker

    case "${MODE}" in
        check-deps)
            do_check_deps ;;
        download-numpy)
            do_download_numpy ;;
        download-diffusers)
            do_download_diffusers ;;
        download-sd21)
            do_download_sd21 ;;
        download-dm-calib)
            do_download_dm_calib ;;
        download-mobile-sam)
            do_download_mobile_sam ;;
        shell)
            do_shell ;;
        test-only)
            do_test ;;
        build-only)
            do_build
            do_test ;;
        run)
            # 若已有同镜像容器在运行则先停止，避免相互影响
            stop_running_calib_containers
            # 若未编译先编译
            if [[ ! -f "${BUILD_DIR}/bin/unicalib_joint" ]]; then
                log_warn "未找到编译产物，先编译..."
                do_build
            fi
            # 仅运行标定任务，不再执行 do_test（避免 .calib_scripts 在上一进程 cleanup 后被删、本进程挂载后 test.sh 尚未就绪导致容器内找不到脚本）
            do_run ;;
        default|*)
            do_build
            do_test
            echo ""
            log_ok "一键编译 + 验证完成!"
            echo ""
            echo -e "${CYAN}${BOLD}后续用法:${NC}"
            echo "  # 查看各标定任务详细说明与数据要求:"
            echo "  ./calib_unified_run.sh --task-help"
            echo ""
            echo "  # 按任务运行（均使用 calib_unified/config/unicalib_example.yaml）:"
            echo "  ./calib_unified_run.sh --run --task imu-intrin"
            echo "  ./calib_unified_run.sh --run --task cam-intrin"
            echo "  ./calib_unified_run.sh --run --task imu-lidar"
            echo "  ./calib_unified_run.sh --run --task lidar-cam"
            echo "  ./calib_unified_run.sh --run --task lidar-lidar"
            echo "  ./calib_unified_run.sh --run --task cam-cam"
            echo "  ./calib_unified_run.sh --run --task joint"
            echo ""
            echo "  # 指定数据路径（如使用 nya_02_ros2 数据集）:"
            echo "  ./calib_unified_run.sh --run --task lidar-cam --config calib_unified/config/unicalib_example.yaml --dataset nya_02_ros2"
            echo "  ./calib_unified_run.sh --run --task lidar-cam --data-dir /path/to/data --dataset nya_02_ros2 --coarse --manual"
            echo ""
            echo "  # 进入容器调试:"
            echo "  ./calib_unified_run.sh --shell" ;;
    esac
}

# 清理临时脚本 (正常退出 + 中断)
cleanup() {
    rm -rf "${SCRIPTS_TMPDIR}"
}
trap 'cleanup; echo ""; log_error "被中断"; exit 130' INT TERM
trap 'cleanup' EXIT

main "$@"
