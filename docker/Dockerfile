# =============================================================================
# 多标定项目联合开发环境 - ROS2 Humble + CUDA + PyTorch
# 支持项目: click_calib, MIAS-LCEC, iKalibr, learn-to-calib, 
#          WoodScape, DM-Calib, L2Calib
# =============================================================================

# 基础镜像：NVIDIA CUDA + Ubuntu 22.04
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# 环境变量设置
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=Asia/Shanghai

# CUDA 环境变量
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# ROS2 环境变量
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# 工作空间路径
ENV WORKSPACE=/root/calib_ws
ENV THIRDPARTY_WS=/root/thirdparty

# =============================================================================
# 阶段1：基础系统依赖
# =============================================================================

# 设置时区和基础工具
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update && apt-get install -y --no-install-recommends \
    # 基础开发工具
    build-essential \
    cmake \
    cmake-curses-gui \
    git \
    wget \
    curl \
    unzip \
    pkg-config \
    software-properties-common \
    gnupg2 \
    lsb-release \
    ca-certificates \
    # 编译工具链
    gcc-11 \
    g++-11 \
    gdb \
    ninja-build \
    ccache \
    # 编辑器和终端工具
    vim \
    nano \
    tmux \
    htop \
    tree \
    # 网络工具
    net-tools \
    iputils-ping \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# 设置默认编译器
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100

# 使用本地预装 CMake 3.24（Sophus 等依赖要求，避免构建时拉取外网）
COPY ./deps/cmake /opt/cmake
ENV PATH="/opt/cmake/bin:${PATH}"

# =============================================================================
# 阶段2：安装 ROS2 Humble
# =============================================================================

# 添加 ROS2 仓库
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble 桌面完整版
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-ros-core \
    # ROS2 开发工具
    ros-dev-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    # ROS2 额外包
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-stereo-image-proc \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-camera-calibration-parsers \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-ros2bag \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init || true && \
    rosdep update --rosdistro ${ROS_DISTRO}

# =============================================================================
# 阶段3：核心数学库和依赖
# =============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Eigen3
    libeigen3-dev \
    # Boost
    libboost-all-dev \
    # BLAS/LAPACK
    libatlas-base-dev \
    liblapack-dev \
    libblas-dev \
    libsuitesparse-dev \
    # 日志库
    libgoogle-glog-dev \
    libgflags-dev \
    # 格式库
    libfmt-dev \
    # YAML处理
    libyaml-cpp-dev \
    # HDF5
    libhdf5-dev \
    # 其他数学库
    libmetis-dev \
    libcholmod3 \
    # JSON库
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# 阶段4：OpenCV 4.x（带 CUDA 支持）
# =============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    # 图像I/O
    libpng-dev \
    libjpeg-dev \
    libopenexr-dev \
    libtiff-dev \
    libwebp-dev \
    # 视频I/O
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    # GUI
    libgtk-3-dev \
    libcanberra-gtk3-module \
    # 多线程
    libtbb-dev \
    # 其他
    libdc1394-dev \
    && rm -rf /var/lib/apt/lists/*

# 从源码编译 OpenCV 4.8.0（支持 CUDA）
ARG OPENCV_VERSION=4.8.0
ARG IPPICV_HASH="1224f78da6684df04397ac0f40c961ed37f79ccb"
ARG IPPICV_FILENAME="ippicv_2021.8_lnx_intel64_20230330_general.tgz"
COPY ./deps/opencv/opencv ${THIRDPARTY_WS}/opencv
COPY ./deps/opencv/opencv_contrib ${THIRDPARTY_WS}/opencv_contrib
# 预下载 IPPICV 等依赖，避免构建时网络请求
COPY ./deps/opencv_deps ${THIRDPARTY_WS}/opencv_deps
RUN mkdir -p ${THIRDPARTY_WS}/opencv/build && cd ${THIRDPARTY_WS}/opencv/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOPENCV_EXTRA_MODULES_PATH=${THIRDPARTY_WS}/opencv_contrib/modules \
        # IPPICV: 使用本地预下载文件，避免网络请求
        -DOPENCV_IPPICV_URL="file://${THIRDPARTY_WS}/opencv_deps/${IPPICV_FILENAME}" \
        # WeChat QR Code: 使用本地预下载文件，避免网络请求
        # 通过 OPENCV_DOWNLOAD_PATH 指定本地缓存目录
        -DOPENCV_DOWNLOAD_PATH="${THIRDPARTY_WS}/opencv_deps" \
        # 额外指定 WeChat QR Code 模型的哈希值（与下载脚本一致）
        -DOPENCV_WECHAT_QRCODE_DOWNLOAD_HASH="a8b69ccc738421293254aec5ddb38bd523503252" \
        # CUDA 配置
        -DWITH_CUDA=ON \
        -DCUDA_ARCH_BIN="7.0;7.5;8.0;8.6;8.9" \
        -DCUDA_ARCH_PTX="" \
        -DWITH_CUDNN=ON \
        -DOPENCV_DNN_CUDA=ON \
        -DENABLE_FAST_MATH=ON \
        -DCUDA_FAST_MATH=ON \
        -DWITH_CUBLAS=ON \
        # 其他功能模块
        -DWITH_TBB=ON \
        -DWITH_OPENMP=ON \
        -DWITH_V4L=ON \
        -DWITH_GTK=ON \
        -DWITH_OPENGL=ON \
        -DWITH_FFMPEG=ON \
        # Python 绑定
        -DBUILD_opencv_python3=ON \
        -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
        # 构建选项
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DINSTALL_PYTHON_EXAMPLES=OFF \
        -DINSTALL_C_EXAMPLES=OFF \
        -DOPENCV_ENABLE_NONFREE=ON \
        -GNinja && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    # 保留 opencv_deps 供构建脚本提取（减小镜像体积可通过重新构建实现）
    cd ${THIRDPARTY_WS} && rm -rf opencv opencv_contrib && \
    echo "OpenCV 构建完成，依赖文件保留在 ${THIRDPARTY_WS}/opencv_deps"

# =============================================================================
# 阶段5：PCL（Point Cloud Library）
# =============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcl-dev \
    pcl-tools \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# 阶段6：Ceres Solver 2.1（最新稳定版）
# =============================================================================

ARG CERES_VERSION=2.1.0
COPY ./deps/ceres-solver ${THIRDPARTY_WS}/ceres-solver
RUN mkdir -p ${THIRDPARTY_WS}/ceres-solver/build && cd ${THIRDPARTY_WS}/ceres-solver/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DUSE_CUDA=ON \
        -GNinja && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd ${THIRDPARTY_WS} && rm -rf ceres-solver

# =============================================================================
# 阶段7：GTSAM 4.2（用于 iKalibr）
# =============================================================================

ARG GTSAM_VERSION=4.2
COPY ./deps/gtsam ${THIRDPARTY_WS}/gtsam
RUN mkdir -p ${THIRDPARTY_WS}/gtsam/build && cd ${THIRDPARTY_WS}/gtsam/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=ON \
        -DGTSAM_USE_QUATERNIONS=ON \
        -DGTSAM_WITH_TBB=ON \
        -DGTSAM_BUILD_PYTHON=OFF \
        -GNinja && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd ${THIRDPARTY_WS} && rm -rf gtsam

# =============================================================================
# 阶段8：Sophus（李群/李代数库）
# =============================================================================
COPY ./deps/Sophus ${THIRDPARTY_WS}/Sophus
RUN mkdir -p ${THIRDPARTY_WS}/Sophus/build && cd ${THIRDPARTY_WS}/Sophus/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_SOPHUS_TESTS=OFF \
        -DBUILD_SOPHUS_EXAMPLES=OFF \
        -GNinja && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd ${THIRDPARTY_WS} && rm -rf Sophus

# =============================================================================
# 阶段9：magic_enum（用于 iKalibr，本地 deps 避免构建时拉取网络）
# =============================================================================

COPY ./deps/magic_enum ${THIRDPARTY_WS}/magic_enum
RUN mkdir -p ${THIRDPARTY_WS}/magic_enum/build && cd ${THIRDPARTY_WS}/magic_enum/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DMAGIC_ENUM_OPT_BUILD_TESTS=OFF \
        -DMAGIC_ENUM_OPT_BUILD_EXAMPLES=OFF \
        -GNinja && \
    ninja -j$(nproc) && \
    ninja install && \
    ldconfig && \
    cd ${THIRDPARTY_WS} && rm -rf magic_enum

# =============================================================================
# 阶段10：Python 环境和深度学习框架
# =============================================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-dev \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-pandas \
    python3-sklearn \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 升级 pip
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

# 安装 PyTorch（CUDA 11.8）
RUN pip3 install --no-cache-dir \
    torch==2.1.0+cu118 \
    torchvision==0.16.0+cu118 \
    torchaudio==2.1.0+cu118 \
    --extra-index-url https://download.pytorch.org/whl/cu118

# 安装深度学习和计算机视觉相关包（先移除系统 distutils 安装的 blinker，避免 pip 卸载报错）
RUN rm -rf /usr/lib/python3*/dist-packages/blinker* && \
    pip3 install --no-cache-dir \
    # 深度学习
    pytorch-lightning==2.1.0 \
    tensorboard==2.15.0 \
    wandb \
    # 计算机视觉
    albumentations \
    kornia \
    timm \
    segmentation-models-pytorch \
    # 点云处理
    open3d \
    # 数据处理
    h5py \
    lmdb \
    pyarrow \
    # 配置管理
    hydra-core \
    omegaconf \
    # 可视化
    tqdm \
    rich \
    plotly \
    # 标定相关
    transforms3d \
    pyquaternion \
    # Jupyter
    jupyter \
    jupyterlab \
    ipywidgets \
    # 其他工具
    pyyaml \
    einops \
    natsort

# =============================================================================
# 阶段11：额外依赖（各项目特定需求）
# =============================================================================

# 用于 WoodScape 的鱼眼相机处理
RUN pip3 install --no-cache-dir \
    imageio \
    imageio-ffmpeg \
    scikit-image

# 用于各项目的ROS2 Python绑定
RUN pip3 install --no-cache-dir \
    rospkg \
    catkin_pkg \
    empy==3.3.4

# =============================================================================
# 阶段12：创建工作空间结构
# =============================================================================

# 创建工作空间目录结构
RUN mkdir -p ${WORKSPACE}/src && \
    mkdir -p ${WORKSPACE}/data && \
    mkdir -p ${WORKSPACE}/logs && \
    mkdir -p ${WORKSPACE}/models && \
    mkdir -p ${WORKSPACE}/results

# 创建项目挂载点（空目录，运行时挂载）
RUN mkdir -p ${WORKSPACE}/src/click_calib && \
    mkdir -p ${WORKSPACE}/src/MIAS-LCEC && \
    mkdir -p ${WORKSPACE}/src/iKalibr && \
    mkdir -p ${WORKSPACE}/src/learn-to-calib && \
    mkdir -p ${WORKSPACE}/src/WoodScape && \
    mkdir -p ${WORKSPACE}/src/DM-Calib && \
    mkdir -p ${WORKSPACE}/src/L2Calib

# =============================================================================
# 阶段13：配置 Shell 环境
# =============================================================================

# 配置 bashrc
RUN echo "" >> /root/.bashrc && \
    echo "# ===== ROS2 Humble Setup =====" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# ===== Workspace Setup =====" >> /root/.bashrc && \
    echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then" >> /root/.bashrc && \
    echo "    source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# ===== Aliases =====" >> /root/.bashrc && \
    echo "alias cb='cd ${WORKSPACE} && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbs='cd ${WORKSPACE} && colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias cbd='cd ${WORKSPACE} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug'" >> /root/.bashrc && \
    echo "alias cbr='cd ${WORKSPACE} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> /root/.bashrc && \
    echo "alias ct='cd ${WORKSPACE} && colcon test'" >> /root/.bashrc && \
    echo "alias ctr='cd ${WORKSPACE} && colcon test-result --verbose'" >> /root/.bashrc && \
    echo "alias cc='cd ${WORKSPACE} && rm -rf build install log'" >> /root/.bashrc && \
    echo "alias ws='cd ${WORKSPACE}'" >> /root/.bashrc && \
    echo "alias src='cd ${WORKSPACE}/src'" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# ===== Environment Variables =====" >> /root/.bashrc && \
    echo "export WORKSPACE=${WORKSPACE}" >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{name}]: {message}'" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# ===== Welcome Message =====" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo '================================================'" >> /root/.bashrc && \
    echo "echo '  Multi-Calibration Development Environment'" >> /root/.bashrc && \
    echo "echo '  ROS2: Humble | CUDA: 11.8 | PyTorch: 2.1'" >> /root/.bashrc && \
    echo "echo '================================================'" >> /root/.bashrc && \
    echo "echo 'Supported Projects:'" >> /root/.bashrc && \
    echo "echo '  - click_calib, MIAS-LCEC, iKalibr'" >> /root/.bashrc && \
    echo "echo '  - learn-to-calib, WoodScape, DM-Calib, L2Calib'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo 'Quick Commands:'" >> /root/.bashrc && \
    echo "echo '  cb  - Build all packages'" >> /root/.bashrc && \
    echo "echo '  cbs <pkg> - Build specific package'" >> /root/.bashrc && \
    echo "echo '  cc  - Clean workspace'" >> /root/.bashrc && \
    echo "echo '  ws  - Go to workspace'" >> /root/.bashrc && \
    echo "echo '================================================'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc

# =============================================================================
# 阶段14：拷贝入口与辅助脚本（参考 reference 结构，便于离线/可观测）
# =============================================================================

# 拷贝 scripts 到 /opt/scripts（entrypoint、验证、ROS2 驱动编译等）
COPY ./scripts /opt/scripts
RUN chmod +x /opt/scripts/*.sh 2>/dev/null || true

# 创建编译脚本（保留兼容）
RUN mkdir -p /root/scripts && \
    echo '#!/bin/bash' > /root/scripts/build_all.sh && \
    echo 'set -e' >> /root/scripts/build_all.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/scripts/build_all.sh && \
    echo 'cd ${WORKSPACE}' >> /root/scripts/build_all.sh && \
    echo 'colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release' >> /root/scripts/build_all.sh && \
    echo 'echo "Build completed successfully!"' >> /root/scripts/build_all.sh && \
    chmod +x /root/scripts/build_all.sh

# 创建测试环境脚本
RUN echo '#!/bin/bash' > /root/scripts/test_env.sh && \
    echo 'echo "Testing environment..."' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "=== ROS2 ===" ' >> /root/scripts/test_env.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/scripts/test_env.sh && \
    echo 'ros2 --version' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "=== Python ===" ' >> /root/scripts/test_env.sh && \
    echo 'python3 --version' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "=== PyTorch ===" ' >> /root/scripts/test_env.sh && \
    echo 'python3 -c "import torch; print(f\"PyTorch: {torch.__version__}\"); print(f\"CUDA available: {torch.cuda.is_available()}\"); print(f\"CUDA version: {torch.version.cuda}\")"' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "=== OpenCV ===" ' >> /root/scripts/test_env.sh && \
    echo 'python3 -c "import cv2; print(f\"OpenCV: {cv2.__version__}\")"' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "=== PCL ===" ' >> /root/scripts/test_env.sh && \
    echo 'pkg-config --modversion pcl_common' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "=== Eigen ===" ' >> /root/scripts/test_env.sh && \
    echo 'pkg-config --modversion eigen3' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "=== Ceres ===" ' >> /root/scripts/test_env.sh && \
    echo 'pkg-config --modversion ceres 2>/dev/null || echo "Ceres installed (no pkg-config)"' >> /root/scripts/test_env.sh && \
    echo 'echo ""' >> /root/scripts/test_env.sh && \
    echo 'echo "Environment test completed!"' >> /root/scripts/test_env.sh && \
    chmod +x /root/scripts/test_env.sh

# 创建项目初始化脚本
RUN echo '#!/bin/bash' > /root/scripts/init_workspace.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/scripts/init_workspace.sh && \
    echo 'cd ${WORKSPACE}' >> /root/scripts/init_workspace.sh && \
    echo '' >> /root/scripts/init_workspace.sh && \
    echo 'echo "Initializing ROS2 workspace..."' >> /root/scripts/init_workspace.sh && \
    echo '' >> /root/scripts/init_workspace.sh && \
    echo '# 更新 rosdep' >> /root/scripts/init_workspace.sh && \
    echo 'rosdep update' >> /root/scripts/init_workspace.sh && \
    echo '' >> /root/scripts/init_workspace.sh && \
    echo '# 安装依赖' >> /root/scripts/init_workspace.sh && \
    echo 'if [ -d "${WORKSPACE}/src" ] && [ "$(ls -A ${WORKSPACE}/src)" ]; then' >> /root/scripts/init_workspace.sh && \
    echo '    rosdep install --from-paths src --ignore-src -r -y' >> /root/scripts/init_workspace.sh && \
    echo 'fi' >> /root/scripts/init_workspace.sh && \
    echo '' >> /root/scripts/init_workspace.sh && \
    echo 'echo "Workspace initialized!"' >> /root/scripts/init_workspace.sh && \
    chmod +x /root/scripts/init_workspace.sh

# =============================================================================
# 阶段15：清理和最终配置
# =============================================================================

# 清理缓存
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/* && \
    rm -rf /root/.cache/pip/*

# 设置工作目录
WORKDIR ${WORKSPACE}

# 暴露常用端口
# RViz2
EXPOSE 11311
# Jupyter
EXPOSE 8888
# TensorBoard
EXPOSE 6006

# 使用统一入口脚本（自动 source 环境、可选自动编译驱动、支持传入命令）
ENTRYPOINT ["/opt/scripts/entrypoint.sh"]

# =============================================================================
# 构建和运行说明
# =============================================================================
# 
# 构建镜像:
#   docker build -t calib_env:humble . 2>&1 | tee build.log
#
# 运行容器 (基础):
#   docker run -it --rm \
#     --gpus all \
#     --privileged \
#     -v /path/to/click_calib:/root/calib_ws/src/click_calib \
#     -v /path/to/MIAS-LCEC:/root/calib_ws/src/MIAS-LCEC \
#     -v /path/to/iKalibr:/root/calib_ws/src/iKalibr \
#     -v /path/to/learn-to-calib:/root/calib_ws/src/learn-to-calib \
#     -v /path/to/WoodScape:/root/calib_ws/src/WoodScape \
#     -v /path/to/DM-Calib:/root/calib_ws/src/DM-Calib \
#     -v /path/to/L2Calib:/root/calib_ws/src/L2Calib \
#     -v /path/to/data:/root/calib_ws/data \
#     calib_env:humble
#
# 运行容器 (完整，带GUI支持):
#   xhost +local:docker && \
#   docker run -it --rm \
#     --gpus all \
#     --privileged \
#     --network host \
#     --ipc host \
#     -e DISPLAY=$DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v $HOME/.Xauthority:/root/.Xauthority \
#     -v /path/to/projects:/root/calib_ws/src \
#     -v /path/to/data:/root/calib_ws/data \
#     calib_env:humble
#
# =============================================================================