# =============================================================================
# UniCalib Unified — 多传感器统一标定系统
# 五种标定: IMU内参 / 相机内参 / IMU-LiDAR外参 / LiDAR-Camera外参 / Camera-Camera外参
# 独立标定 + 联合标定
# 无 tiny-viewer 依赖 / 无 ROS 强制依赖 (可选 ROS2 bag 读取)
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(UniCalib VERSION 2.0.0 LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# C++ 标准与编译选项
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_FLAGS_RELEASE   "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG   "-g -O1")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O1")

# 打开额外优化标志（仅 Release）
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-march=native -funroll-loops)
endif()

# 输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 模块路径
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus/cmake_modules
)

# ---------------------------------------------------------------------------
# 选项：B-样条与可视化独立控制
# ---------------------------------------------------------------------------
option(UNICALIB_BUILD_APPS      "Build application executables"        ON)
option(UNICALIB_WITH_ROS2       "Enable optional ROS2 bag reading"     OFF)
option(UNICALIB_WITH_PCL_VIZ    "Enable PCL visualizer"                ON)
option(UNICALIB_WITH_OPENMP     "Enable OpenMP parallelism"            ON)
option(UNICALIB_WITH_PYTHON_BRIDGE "Enable AI coarse calib (Python)"   ON)
option(UNICALIB_BUILD_TESTS      "Build unit tests"                     OFF)

# B-样条求解器（基于 basalt-headers）
# - 无 ROS 强制依赖，仅需 Ceres
# - 可与任何可视化后端组合
option(UNICALIB_WITH_IKALIBR
    "Build B-spline solver core (requires Ceres, Eigen, Sophus)"
    ON)

# Pangolin 3D 可视化（本地库，推荐）
# - 纯 C++，依赖 OpenGL/X11，无 ROS
# - 支持轨迹、点云、优化曲线交互显示
# 注意: Pangolin 0.9.0 API 与新版不兼容，若编译失败可临时关闭此选项
option(UNICALIB_WITH_PANGOLIN
    "Use Pangolin for 3D visualization (OpenGL backend)"
    OFF)

# ROS RViz 可视化（可选）
# - 仅在 ROS2 环境有效
# - 与 Pangolin 互斥（Pangolin 优先）
option(UNICALIB_WITH_ROS_RVIZ
    "Enable ROS RViz visualization adapter (requires ROS2)"
    OFF)

option(BUILD_TESTING            "Build tests"                          OFF)

# ---------------------------------------------------------------------------
# 第三方库: Eigen3 (header-only, from system or bundled)
# ---------------------------------------------------------------------------
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  # 尝试从系统 /usr/include/eigen3
  find_path(EIGEN3_INCLUDE_DIR NAMES Eigen/Core
    HINTS /usr/include/eigen3 /usr/local/include/eigen3 /opt/ros/humble/include/eigen3)
  if(EIGEN3_INCLUDE_DIR)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
    message(STATUS "Eigen3 found at: ${EIGEN3_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "Eigen3 not found. Install: sudo apt install libeigen3-dev")
  endif()
else()
  message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
endif()

# ---------------------------------------------------------------------------
# 第三方库: Sophus (header-only, from thirdparty/Sophus)
# ---------------------------------------------------------------------------
set(SOPHUS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus)
if(EXISTS ${SOPHUS_SOURCE_DIR}/CMakeLists.txt)
  set(SOPHUS_INSTALL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${SOPHUS_SOURCE_DIR} thirdparty_build/Sophus EXCLUDE_FROM_ALL)
  message(STATUS "Sophus: using bundled source")
else()
  find_package(Sophus REQUIRED)
  message(STATUS "Sophus: found in system")
endif()

# ---------------------------------------------------------------------------
# 第三方库: magic_enum (header-only)
# ---------------------------------------------------------------------------
add_library(magic_enum INTERFACE)
target_include_directories(magic_enum INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/magic_enum/include)
add_library(magic_enum::magic_enum ALIAS magic_enum)

# ---------------------------------------------------------------------------
# 第三方库: cereal (header-only)
# ---------------------------------------------------------------------------
add_library(cereal_unicalib INTERFACE)
target_include_directories(cereal_unicalib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal_eigen_include)
add_library(cereal::cereal ALIAS cereal_unicalib)

# ---------------------------------------------------------------------------
# 第三方库: veta-impl (自包含 SfM 数据结构, 无外部依赖)
# 完全替代外部 veta 库; 头文件位于 thirdparty/veta-impl/include/
# ---------------------------------------------------------------------------
set(VETA_IMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/veta-impl/include)
add_library(veta_impl INTERFACE)
target_include_directories(veta_impl INTERFACE ${VETA_IMPL_DIR})
target_link_libraries(veta_impl INTERFACE Eigen3::Eigen)
add_library(veta::veta   ALIAS veta_impl)
add_library(ns_veta::veta ALIAS veta_impl)
message(STATUS "veta: using self-contained veta-impl at ${VETA_IMPL_DIR}")

# ---------------------------------------------------------------------------
# 第三方库: tiny-viewer stubs (no-op, 仅提供 ns_viewer:: 骨架类型)
# ---------------------------------------------------------------------------
add_library(tiny_viewer_stub INTERFACE)
target_include_directories(tiny_viewer_stub INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ikalibr  # viewer_types.h is in viewer/ subdir
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/tiny-viewer-stub
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj_full/tiny-viewer_stub)
add_library(tiny_viewer::stub ALIAS tiny_viewer_stub)

# ===========================================================================
# 第三方库：basalt-headers（B-样条核心，header-only）
# ===========================================================================
# 来源：https://gitlab.com/VladyslavUsenko/basalt-headers.git
# 功能：
#   - Se3Spline<N, Scalar>: SE(3) 均匀 B-样条（分割表示法）
#   - So3Spline<N, Scalar>: SO(3) 累积 B-样条
#   - RdSpline<DIM, N, Scalar>: R^d 欧氏 B-样条
#   - CeresSplineHelper: Ceres 自动求导集成
#   - CeresLocalParamSE3: SE(3) 李群参数化

set(BASALT_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers)

if(NOT EXISTS "${BASALT_HEADERS_DIR}/include/basalt/spline/se3_spline.h")
    message(FATAL_ERROR 
        "basalt-headers not found at ${BASALT_HEADERS_DIR}. "
        "Expected: ${BASALT_HEADERS_DIR}/include/basalt/spline/se3_spline.h")
endif()

# 创建 INTERFACE 库
add_library(basalt_headers INTERFACE)
target_include_directories(basalt_headers INTERFACE
    ${BASALT_HEADERS_DIR}/include)

# basalt-headers 本身需要 Eigen3 和 Sophus
target_link_libraries(basalt_headers INTERFACE
    Eigen3::Eigen
    Sophus::Sophus)

# 别名
add_library(basalt::headers ALIAS basalt_headers)

message(STATUS "basalt-headers: ✓ Found at ${BASALT_HEADERS_DIR}/include")
message(STATUS "  • Features: Se3Spline, So3Spline, RdSpline, CeresHelper")

# ---------------------------------------------------------------------------
# 第三方库: Pangolin 0.9.0 (C++ 可视化, 替代 tiny-viewer)
# 仅使用本地路径通过 add_subdirectory 参与编译，不使用 find_package(Pangolin)
# 默认: thirdparty/Pangolin/Pangolin-0.9.0；可覆盖: -DPANGOLIN_SOURCE_DIR=/path/to/Pangolin
# ---------------------------------------------------------------------------
if(NOT DEFINED PANGOLIN_SOURCE_DIR)
  set(PANGOLIN_SOURCE_DIR
      ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Pangolin/Pangolin-0.9.0
      CACHE PATH "Pangolin 源码目录，用于 add_subdirectory 编译")
endif()
if(EXISTS ${PANGOLIN_SOURCE_DIR}/CMakeLists.txt)
  set(BUILD_EXAMPLES    OFF CACHE BOOL "Pangolin: disable examples"    FORCE)
  set(BUILD_TOOLS       OFF CACHE BOOL "Pangolin: disable tools"       FORCE)
  set(BUILD_PANGOLIN_PYPANGOLIN OFF CACHE BOOL "Pangolin: no Python"   FORCE)
  set(BUILD_ASAN        OFF CACHE BOOL "Pangolin: no ASan"             FORCE)
  add_subdirectory(${PANGOLIN_SOURCE_DIR}
                   ${CMAKE_BINARY_DIR}/thirdparty_build/Pangolin
                   EXCLUDE_FROM_ALL)
  set(UNICALIB_WITH_PANGOLIN ON)
  add_compile_definitions(UNICALIB_WITH_PANGOLIN=1)
  message(STATUS "Pangolin: building from ${PANGOLIN_SOURCE_DIR}")

  # 手动添加 Pangolin 组件的 include 路径，确保编译器能找到头文件
  # 注意: pango_display 的 target_include_directories 使用 BUILD_INTERFACE，
  # 但 CMake 可能没有正确传递给我们需要手动添加
  set(PANGOLIN_INCLUDE_DIRS
      ${PANGOLIN_SOURCE_DIR}/components/pango_core/include
      ${PANGOLIN_SOURCE_DIR}/components/pango_display/include
      ${PANGOLIN_SOURCE_DIR}/components/pango_opengl/include
      ${PANGOLIN_SOURCE_DIR}/components/pango_windowing/include
      ${PANGOLIN_SOURCE_DIR}/components/pango_vars/include
      ${PANGOLIN_SOURCE_DIR}/components/pango_image/include
      ${PANGOLIN_SOURCE_DIR}/components/pango_plot/include
      ${PANGOLIN_SOURCE_DIR}/components/pango_video/include
  )
  message(STATUS "Pangolin include paths: ${PANGOLIN_INCLUDE_DIRS}")
else()
  set(UNICALIB_WITH_PANGOLIN OFF)
  add_compile_definitions(UNICALIB_WITH_PANGOLIN=0)
  message(WARNING "Pangolin source not found at ${PANGOLIN_SOURCE_DIR}, using no-op stub")
  # include/ikalibr/pangolin/display/display.h 提供无操作占位
  # (include/ikalibr 已在 target_include_directories 中, 不需要额外添加路径)
endif()

# ---------------------------------------------------------------------------
# 第三方库: basalt-headers + ctraj_basalt_compat (替代 ctraj，B-Spline 轨迹)
# ---------------------------------------------------------------------------
set(BASALT_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers/include)
if(EXISTS "${BASALT_HEADERS_DIR}/basalt/spline/se3_spline.h")
  message(STATUS "basalt-headers: using ${BASALT_HEADERS_DIR}")
  set(BASALT_HEADERS_FOUND TRUE)
else()
  message(FATAL_ERROR "basalt-headers not found at ${BASALT_HEADERS_DIR}")
  set(BASALT_HEADERS_FOUND FALSE)
endif()
set(CTRAJ_BASALT_COMPAT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj_basalt_compat/include)
if(NOT EXISTS "${CTRAJ_BASALT_COMPAT_DIR}/ctraj/core/spline_bundle.h")
  message(FATAL_ERROR "ctraj_basalt_compat not found at ${CTRAJ_BASALT_COMPAT_DIR}")
endif()
message(STATUS "ctraj: using basalt-headers + ctraj_basalt_compat (replacing ctraj_full)")

# ---------------------------------------------------------------------------
# 系统依赖: Ceres Solver
# ---------------------------------------------------------------------------
find_package(Ceres REQUIRED)
message(STATUS "Ceres: ${CERES_VERSION}")

# ---------------------------------------------------------------------------
# 系统依赖: OpenCV (4.x preferred)
# ---------------------------------------------------------------------------
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d highgui features2d video)
message(STATUS "OpenCV: ${OpenCV_VERSION}")

# ---------------------------------------------------------------------------
# 系统依赖: PCL
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS
    common io filters kdtree search segmentation registration features visualization)
message(STATUS "PCL: ${PCL_VERSION}")
# PCL 不能与 -march=native 同时使用时需要禁用
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

# ---------------------------------------------------------------------------
# 系统依赖: spdlog
# ---------------------------------------------------------------------------
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
  message(STATUS "spdlog not found via find_package, using FetchContent")
  include(FetchContent)
  FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.13.0
    GIT_SHALLOW    TRUE)
  FetchContent_MakeAvailable(spdlog)
else()
  message(STATUS "spdlog: ${spdlog_VERSION}")
endif()

# ---------------------------------------------------------------------------
# 系统依赖: fmt (用于格式化）
# ---------------------------------------------------------------------------
find_package(fmt QUIET)
if(NOT fmt_FOUND)
  message(STATUS "fmt not found via find_package, using FetchContent")
  include(FetchContent)
  FetchContent_Declare(fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG        10.2.1
    GIT_SHALLOW    TRUE)
  FetchContent_MakeAvailable(fmt)
  set(FMT_LIBRARIES fmt::fmt)
else()
  message(STATUS "fmt: ${fmt_VERSION}")
endif()

# ---------------------------------------------------------------------------
# 系统依赖: yaml-cpp
# ---------------------------------------------------------------------------
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(YAML_CPP yaml-cpp)
  endif()
  if(NOT YAML_CPP_FOUND)
    message(STATUS "yaml-cpp not found via find_package, using FetchContent")
    include(FetchContent)
    FetchContent_Declare(yaml-cpp
      GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
      GIT_TAG        yaml-cpp-0.8.0
      GIT_SHALLOW    TRUE)
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(yaml-cpp)
  endif()
else()
  message(STATUS "yaml-cpp: ${yaml-cpp_VERSION}")
endif()

# ---------------------------------------------------------------------------
# OpenMP (可选)
# ---------------------------------------------------------------------------
if(UNICALIB_WITH_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP: found, version ${OpenMP_CXX_VERSION}")
  else()
    message(STATUS "OpenMP: not found, parallel processing disabled")
  endif()
endif()

# ---------------------------------------------------------------------------
# ROS2 (自动探测 — iKalibr 头文件通过 ros.h shim 封装)
# ---------------------------------------------------------------------------
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(rosbag2_cpp QUIET)
find_package(sensor_msgs QUIET)
find_package(cv_bridge QUIET)
if(rclcpp_FOUND)
  message(STATUS "ROS2: found rclcpp, enabling full ROS2 integration")
  add_compile_definitions(UNICALIB_WITH_ROS2=1)
  set(UNICALIB_WITH_ROS2 ON)
else()
  message(STATUS "ROS2: rclcpp not found, using lightweight stub (ros.h stub)")
  add_compile_definitions(UNICALIB_WITH_ROS2=0)
  set(UNICALIB_WITH_ROS2 OFF)
endif()

# ---------------------------------------------------------------------------
# ufomap (必须使用包含 surfel/octree 新 API 的版本, 优先 iKalibr 预编译)
# 旧版 thirdparty/ufomap 无 surfel_map.h / octree/node.h, 不满足要求
# ---------------------------------------------------------------------------
set(UFOMAP_INSTALL_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../iKalibr/thirdparty-install/ufomap-install)

# 仅接受含 surfel_map.h 的新版 ufomap; 先从 iKalibr 预编译找, 再从外部 ufomap_DIR
set(UFOMAP_SEARCH_HINTS
    ${UFOMAP_INSTALL_DIR}/lib/cmake/ufomap)
if(DEFINED ufomap_DIR AND NOT ufomap_DIR STREQUAL "")
  list(PREPEND UFOMAP_SEARCH_HINTS ${ufomap_DIR})
endif()

# 清除旧版缓存条目, 防止本地旧 thirdparty ufomap 被复用
unset(ufomap_DIR CACHE)
find_package(ufomap QUIET HINTS ${UFOMAP_SEARCH_HINTS} NO_DEFAULT_PATH)

# 验证找到的 ufomap 是否含新 API (surfel_map.h)
if(ufomap_FOUND)
  get_target_property(_ufo_inc UFO::Map INTERFACE_INCLUDE_DIRECTORIES)
  if(EXISTS "${_ufo_inc}/ufo/map/surfel_map.h")
    message(STATUS "ufomap: found new API (surfel_map) at ${_ufo_inc}")
  else()
    message(STATUS "ufomap: found but OLD API (no surfel_map.h), disabling")
    set(ufomap_FOUND FALSE)
  endif()
endif()

if(NOT ufomap_FOUND)
  message(STATUS "ufomap: not found or outdated, LiDAR surfel map disabled")
  add_compile_definitions(UNICALIB_NO_UFOMAP=1)
endif()

# ---------------------------------------------------------------------------
# opengv (来自 iKalibr 预编译安装)
# 使用 iKalibr 预编译头文件; find_package 仅用于链接, 失败时直接用 include 路径
# ---------------------------------------------------------------------------
set(OPENGV_INSTALL_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../iKalibr/thirdparty-install/opengv-install)
unset(opengv_DIR CACHE)
find_package(opengv QUIET
    HINTS ${OPENGV_INSTALL_DIR}/lib/cmake/opengv-1.0
    NO_DEFAULT_PATH)
if(opengv_FOUND)
  message(STATUS "opengv: found prebuilt via cmake config")
  add_compile_definitions(UNICALIB_WITH_OPENGV=1)
elseif(EXISTS "${OPENGV_INSTALL_DIR}/include/opengv/types.hpp")
  # cmake config not usable, but headers exist → add include path + import lib manually
  message(STATUS "opengv: using prebuilt headers at ${OPENGV_INSTALL_DIR}/include")
  add_compile_definitions(UNICALIB_WITH_OPENGV=1)
  include_directories(BEFORE ${OPENGV_INSTALL_DIR}/include)
  # Import the opengv library target so we can link it later
  find_library(OPENGV_LIB NAMES opengv
      PATHS ${OPENGV_INSTALL_DIR}/lib NO_DEFAULT_PATH)
  if(OPENGV_LIB)
    add_library(opengv STATIC IMPORTED GLOBAL)
    set_target_properties(opengv PROPERTIES IMPORTED_LOCATION "${OPENGV_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${OPENGV_INSTALL_DIR}/include")
    message(STATUS "opengv library: ${OPENGV_LIB}")
  else()
    message(WARNING "opengv: headers found but library not found; link may fail")
  endif()
else()
  # Fall back to bundled opengv headers: opengv-stub 或 thirdparty/opengv
  set(_OPENGV_STUB ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opengv-stub)
  set(_OPENGV_BUNDLED ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opengv/include)
  if(EXISTS "${_OPENGV_STUB}/opengv/types.hpp")
    message(STATUS "opengv: using bundled headers at ${_OPENGV_STUB}")
    add_compile_definitions(UNICALIB_WITH_OPENGV=1)
    include_directories(BEFORE ${_OPENGV_STUB})
    add_library(opengv INTERFACE IMPORTED GLOBAL)
    set_target_properties(opengv PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_OPENGV_STUB}")
  elseif(EXISTS "${_OPENGV_BUNDLED}/opengv/types.hpp")
    message(STATUS "opengv: using thirdparty/opengv at ${_OPENGV_BUNDLED}")
    add_compile_definitions(UNICALIB_WITH_OPENGV=1)
    include_directories(BEFORE ${_OPENGV_BUNDLED})
    add_library(opengv INTERFACE IMPORTED GLOBAL)
    set_target_properties(opengv PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_OPENGV_BUNDLED}")
  else()
    message(STATUS "opengv: not found, disabling opengv-dependent features")
    add_compile_definitions(UNICALIB_NO_OPENGV=1)
  endif()
endif()

# ===========================================================================
# 选项: 是否构建 Python 桥接 (AI 粗标定依赖)
# ===========================================================================
option(UNICALIB_WITH_PYTHON_BRIDGE  "Enable Python subprocess bridge for AI coarse calib" ON)
if(UNICALIB_WITH_PYTHON_BRIDGE)
  message(STATUS "Python bridge: enabled (DM-Calib/MIAS-LCEC/Transformer-IMU/L2Calib)")
  add_compile_definitions(UNICALIB_WITH_PYTHON_BRIDGE=1)
endif()

# ===========================================================================
# 核心标定库: libunicalib
# ===========================================================================

# 收集 iKalibr 核心源文件 (非 ROS-强依赖模块)
set(IKALIBR_CORE_SOURCES
    # 工具函数
    src/ikalibr/util/utils.cpp
    src/ikalibr/util/circlesgrid.cpp
    # 传感器数据结构
    src/ikalibr/sensor/imu.cpp
    src/ikalibr/sensor/lidar.cpp
    src/ikalibr/sensor/camera.cpp
    src/ikalibr/sensor/radar.cpp
    src/ikalibr/sensor/rgbd.cpp
    src/ikalibr/sensor/event.cpp
    src/ikalibr/sensor/sensor_model.cpp
    # 配置解析
    src/ikalibr/config/configor.cpp
    # 标定参数管理
    src/ikalibr/calib/calib_param_manager.cpp
    src/ikalibr/calib/calib_data_manager.cpp
    src/ikalibr/calib/estimator.cpp
    src/ikalibr/calib/ceres_callback.cpp
    src/ikalibr/calib/spat_temp_priori.cpp
    # 核心算法
    src/ikalibr/core/rotation_estimator.cpp
    src/ikalibr/core/lidar_odometer.cpp
    src/ikalibr/core/scan_undistortion.cpp
    src/ikalibr/core/pts_association.cpp
    src/ikalibr/core/feature_tracking.cpp
    src/ikalibr/core/optical_flow_trace.cpp
    src/ikalibr/core/visual_distortion.cpp
    src/ikalibr/core/visual_reproj_association.cpp
    src/ikalibr/core/visual_velocity_estimator.cpp
    src/ikalibr/core/visual_velocity_sac.cpp
    src/ikalibr/core/rot_only_vo.cpp
    src/ikalibr/core/colmap_data_io.cpp
    src/ikalibr/core/radar_velocity_sac.cpp
    src/ikalibr/core/event_preprocessing.cpp
    src/ikalibr/core/event_trace_sac.cpp
    src/ikalibr/core/ev_circle_tracking.cpp
    src/ikalibr/core/haste_data_io.cpp
    # SfM (条件编译, 依赖 veta/opengv)
    src/ikalibr/core/vision_only_sfm.cpp
    # 因子
    src/ikalibr/factor/data_correspondence.cpp
    src/ikalibr/factor/factor_instantiation.cpp
    # 求解器 (分块实现)
    src/ikalibr/solver/calib_solver_common_impl.cpp
    src/ikalibr/solver/calib_solver_proc_impl.cpp
    src/ikalibr/solver/calib_solver_da_impl.cpp
    src/ikalibr/solver/calib_solver_bo_impl.cpp
    src/ikalibr/solver/calib_solver_init_so3_spline_impl.cpp
    src/ikalibr/solver/calib_solver_init_scale_spline_impl.cpp
    src/ikalibr/solver/calib_solver_init_sen_inertial_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_li_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ri_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_pci_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_vci_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_di_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_circle_based_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_line_based_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ii_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_bo_impl.cpp
    src/ikalibr/solver/calib_solver_io.cpp
    # 可视化 (viewer.cpp + viewer_pangolin.cpp 仅在 UNICALIB_WITH_PANGOLIN 时加入，见下方)
    src/ikalibr/viewer/visual_ang_vel_drawer.cpp
    src/ikalibr/viewer/visual_colorized_cloud_map.cpp
    src/ikalibr/viewer/visual_gravity.cpp
    src/ikalibr/viewer/visual_lidar_covisibility.cpp
    src/ikalibr/viewer/visual_lin_vel_drawer.cpp
)

# 数据加载器 (含 ROS2 可选支持)
set(IKALIBR_LOADER_SOURCES
    src/ikalibr/sensor/imu_data_loader.cpp
    src/ikalibr/sensor/lidar_data_loader.cpp
    src/ikalibr/sensor/camera_data_loader.cpp
    src/ikalibr/sensor/radar_data_loader.cpp
    src/ikalibr/sensor/depth_data_loader.cpp
    src/ikalibr/sensor/event_data_loader.cpp
)

# UniCalib 新增统一接口源文件
set(UNICALIB_NEW_SOURCES
    # 公共数据结构与异常报告
    src/common/sensor_types.cpp
    src/common/calib_param.cpp
    src/common/exception_report.cpp
    src/common/accuracy_logger.cpp
    src/common/calib_stage.cpp
    # 内参标定
    src/intrinsic/allan_variance.cpp
    src/intrinsic/camera_pinhole_calib.cpp
    # 外参标定
    src/extrinsic/imu_lidar_calib.cpp
    src/extrinsic/lidar_scan_pair.cpp
    src/extrinsic/lidar_lidar_calib.cpp
    src/extrinsic/lidar_camera_calib.cpp
    src/extrinsic/cam_cam_calib.cpp
    # 联合求解
    src/solver/joint_calib_solver.cpp
    # ─── 两阶段流水线 (新增) ─────────────────────────────────────────
    src/pipeline/calib_pipeline.cpp
    src/pipeline/manual_calib.cpp
    src/pipeline/ai_coarse_calib.cpp
    # ─────────────────────────────────────────────────────────────────
    # 可视化 (PCL + OpenCV, 替换 tiny-viewer)
    src/viz/cloud_viewer.cpp
    src/viz/report_gen.cpp
    src/viz/calib_visualizer.cpp
    # Pangolin 渲染 (仅在启用 Pangolin 时编译)
    src/viz/pangolin_render.cpp
    # I/O
    src/io/yaml_io.cpp
    src/io/unified_data_loader.cpp
    src/io/new_format_data_source.cpp
    src/io/lidar_packet_reader.cpp
    src/io/oem7_imu_reader.cpp
    # 工程化基础设施（header-only, 部分cpp实现）
    src/common/calib_param.cpp  # 确保包含
)

# ROS2 bag/话题数据源（仅 UNICALIB_WITH_ROS2）；UnifiedDataLoader 主体见 unified_data_loader.cpp（始终编译）
if(UNICALIB_WITH_ROS2)
    list(APPEND UNICALIB_NEW_SOURCES src/io/ros2_data_source.cpp)
endif()

# 最小 iKalibr 依赖：供 IMU-LiDAR 外参标定使用 (LiDAROdometer / LiDARFrame / _1_)
set(IKALIBR_MINIMAL_FOR_IMU_LIDAR
    src/ikalibr/util/utils.cpp
    src/ikalibr/sensor/lidar.cpp
    src/ikalibr/core/lidar_odometer.cpp
)

# ===========================================================================
# 核心库构造：三层解耦
# ===========================================================================

# ─────────────────────────────────────────────────────────────────────────
# 第 1 层检查：B-样条求解器的前置条件
# ─────────────────────────────────────────────────────────────────────────
if(UNICALIB_WITH_IKALIBR)
    message(STATUS "iKalibr: checking dependencies...")
    
    # B-样条求解需要 Ceres
    if(NOT Ceres_FOUND)
        message(FATAL_ERROR
            "iKalibr requires Ceres but it was not found. "
            "Installation: sudo apt install libceres-dev")
    endif()
    
    # B-样条求解需要 Sophus（李群库）
    if(NOT Sophus_FOUND)
        message(FATAL_ERROR
            "iKalibr requires Sophus but it was not found. "
            "Installation: sudo apt install libsophus-dev")
    endif()
    
    message(STATUS "iKalibr: dependencies ✓ (Ceres, Sophus, Eigen3)")
endif()

# ─────────────────────────────────────────────────────────────────────────
# 第 2 层选择：可视化后端（互斥：Pangolin > RViz > Stub）
# ─────────────────────────────────────────────────────────────────────────

set(_VIEWER_SOURCES)
set(_VIEWER_BACKEND "NONE")

if(UNICALIB_WITH_PANGOLIN)
    # Pangolin 优先级最高（本地库，无额外依赖）
    
    # Pangolin 0.9 为组件化：无 pangolin 单 target，使用 pango_display（display/OpenGL 入口）
    if(NOT TARGET pango_display)
        message(FATAL_ERROR "Pangolin target pango_display not found. Check CMakeLists.txt around line 155")
    endif()
    
    # 添加 viewer 实现
    list(APPEND _VIEWER_SOURCES
        src/ikalibr/viewer/viewer.cpp
        src/ikalibr/viewer/viewer_pangolin.cpp
    )
    set(_VIEWER_BACKEND "PANGOLIN")
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=PANGOLIN)
    list(APPEND UNICALIB_NEW_SOURCES src/viz/manual_lidar_cam_window.cpp)
    list(APPEND UNICALIB_NEW_SOURCES src/viz/manual_lidar_lidar_window.cpp)
    
    message(STATUS "Visualization: ✓ Pangolin 3D backend (interactive OpenGL)")
    
elseif(UNICALIB_WITH_ROS_RVIZ AND UNICALIB_WITH_ROS2)
    # RViz 次优先级（需要 ROS2）
    
    find_package(rclcpp QUIET)
    find_package(visualization_msgs QUIET)
    
    if(rclcpp_FOUND AND visualization_msgs_FOUND)
        list(APPEND _VIEWER_SOURCES
            src/viz/ros_rviz_visualizer.cpp
        )
        set(_VIEWER_BACKEND "RVIZ")
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=RVIZ)
        
        message(STATUS "Visualization: ✓ ROS RViz backend (via markers)")
    else()
        message(WARNING 
            "ROS2 visualization libraries not found. "
            "Fall back to headless mode. "
            "Installation: sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-visualization-msgs")
        
        list(APPEND _VIEWER_SOURCES
            src/viz/viewer_stub.cpp
        )
        set(_VIEWER_BACKEND "STUB")
        add_compile_definitions(IKALIBR_VIEWER_BACKEND=STUB)
        
        message(STATUS "Visualization: ⚠ Headless mode (no GUI)")
    endif()
    
else()
    # 默认：Stub（无 GUI，纯计算）
    list(APPEND _VIEWER_SOURCES
        src/viz/viewer_stub.cpp
    )
    set(_VIEWER_BACKEND "STUB")
    add_compile_definitions(IKALIBR_VIEWER_BACKEND=STUB)
    
    message(STATUS "Visualization: ⚠ Headless mode (no GUI)")
endif()

# ─────────────────────────────────────────────────────────────────────────
# 第 3 层构造：主库源文件列表
# ─────────────────────────────────────────────────────────────────────────

if(UNICALIB_WITH_IKALIBR)
    add_compile_definitions(UNICALIB_WITH_IKALIBR=1)
    
    # 完整源文件列表 = iKalibr 核心 + 加载器 + UniCalib 新增 + 可视化后端
    set(_UNICALIB_SOURCES
        # iKalibr 核心（行 357-479）
        ${IKALIBR_CORE_SOURCES}
        # 数据加载器
        ${IKALIBR_LOADER_SOURCES}
        # UniCalib 新增（行 492-516）
        ${UNICALIB_NEW_SOURCES}
        # 可视化后端（根据选择）
        ${_VIEWER_SOURCES}
    )
    
    message(STATUS "iKalibr: ✓ Enabled (B-spline solver via basalt-headers)")
    message(STATUS "  • Spline backend: basalt-headers")
    message(STATUS "  • Visualization backend: ${_VIEWER_BACKEND}")
    
else()
    add_compile_definitions(UNICALIB_WITH_IKALIBR=0)
    
    # 精简源文件列表：仅 UniCalib 模块 + IMU-LiDAR 最小依赖
    set(_UNICALIB_SOURCES
        ${UNICALIB_NEW_SOURCES}
        ${IKALIBR_MINIMAL_FOR_IMU_LIDAR}
    )
    
    message(STATUS "iKalibr: ⚠ Disabled (lean mode)")
    message(STATUS "  • Use -DUNICALIB_WITH_IKALIBR=ON to enable B-spline solver")
endif()

# ─────────────────────────────────────────────────────────────────────────
# 第 4 步：创建主库目标
# ─────────────────────────────────────────────────────────────────────────

add_library(unicalib SHARED ${_UNICALIB_SOURCES})

# ─────────────────────────────────────────────────────────────────────────
# 第 5 步：链接依赖
# ─────────────────────────────────────────────────────────────────────────

# 包含目录（启用 Pangolin 时其 include 必须优先于 include/ikalibr，否则会用到占位头 display.h）
if(UNICALIB_WITH_PANGOLIN)
  target_include_directories(unicalib PUBLIC
    ${PANGOLIN_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ikalibr>
    $<INSTALL_INTERFACE:include>)
else()
  target_include_directories(unicalib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ikalibr>
    $<INSTALL_INTERFACE:include>)
endif()

# 通用依赖（所有模式）
target_link_libraries(unicalib PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
    basalt::headers
    magic_enum::magic_enum
    cereal::cereal
    veta::veta
    tiny_viewer::stub
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
)

# B-样条求解器依赖（仅 IKALIBR 时）
if(UNICALIB_WITH_IKALIBR)
    target_link_libraries(unicalib PUBLIC Ceres::ceres)
endif()

# Pangolin 可视化依赖（0.9 使用组件 target pango_display）
if(UNICALIB_WITH_PANGOLIN AND _VIEWER_BACKEND STREQUAL "PANGOLIN")
    target_link_libraries(unicalib PUBLIC pango_display pango_windowing)
endif()

# RViz 可视化依赖
if(UNICALIB_WITH_ROS_RVIZ AND _VIEWER_BACKEND STREQUAL "RVIZ")
    target_link_libraries(unicalib PUBLIC
        rclcpp::rclcpp
        visualization_msgs::visualization_msgs)
endif()

# 可选依赖（ROS2 bag 读取与反序列化需 sensor_msgs 类型支持与 cv_bridge）
if(UNICALIB_WITH_ROS2)
    target_link_libraries(unicalib PUBLIC
        rclcpp::rclcpp
        rosbag2_cpp::rosbag2_cpp)
    # sensor_msgs typesupport：rclcpp::Serialization 需要 get_message_type_support_handle（在 rosidl_typesupport_cpp 中）
    target_link_libraries(unicalib PUBLIC
        "/opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so"
        "/opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so")
    # cv_bridge：toCvShare 等
    if(TARGET cv_bridge)
        target_link_libraries(unicalib PUBLIC cv_bridge)
    elseif(cv_bridge_FOUND)
        find_library(CV_BRIDGE_LIB NAMES cv_bridge PATHS "/opt/ros/humble/lib" "$ENV{AMENT_PREFIX_PATH}/lib" NO_DEFAULT_PATH)
        if(CV_BRIDGE_LIB)
            target_link_libraries(unicalib PUBLIC ${CV_BRIDGE_LIB})
        endif()
    endif()
endif()

if(UNICALIB_WITH_OPENMP)
    target_link_libraries(unicalib PUBLIC OpenMP::OpenMP_CXX)
endif()

if(UNICALIB_WITH_PCL_VIZ)
    target_link_libraries(unicalib PUBLIC pcl_common pcl_io)
endif()

message(STATUS "unicalib: ✓ Library configured (${_VIEWER_BACKEND} backend)")

# ---------------------------------------------------------------------------
# 额外 Include 路径（补充上面已有的）
# ---------------------------------------------------------------------------
target_include_directories(unicalib PUBLIC
    # 第三方头文件（补充）
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/unicalib/pipeline>
    # ctraj compat (basalt-headers backend)
    $<BUILD_INTERFACE:${CTRAJ_BASALT_COMPAT_DIR}>
    # PCL
    ${PCL_INCLUDE_DIRS}
)

# ufomap: UFO::Map INTERFACE_INCLUDE_DIRECTORIES 自动传递; 额外显式添加保证鲁棒性
if(ufomap_FOUND)
  target_include_directories(unicalib PUBLIC
      $<BUILD_INTERFACE:${UFOMAP_INSTALL_DIR}/include>)
endif()

# ROS2 include paths (rclcpp, sensor_msgs, pcl_conversions, etc.)
# In ROS2 ament, each package has headers at /opt/ros/<distro>/include/<pkg>/<pkg>/
# We add the package-level dirs so that '#include <pkg/header.h>' works.
if(UNICALIB_WITH_ROS2 AND rclcpp_FOUND)
    set(_ROS2_INCLUDE_BASE "/opt/ros/humble/include")
    if(EXISTS "${_ROS2_INCLUDE_BASE}")
        # Collect all package-level include dirs (ament convention)
        file(GLOB _ROS2_PKG_INCLUDES LIST_DIRECTORIES true "${_ROS2_INCLUDE_BASE}/*")
        target_include_directories(unicalib PUBLIC
            ${_ROS2_INCLUDE_BASE}        # top-level (for some old-style includes)
            ${_ROS2_PKG_INCLUDES}        # per-package dirs (new ament convention)
        )
    endif()
    if(rclcpp_INCLUDE_DIRS)
        target_include_directories(unicalib PUBLIC ${rclcpp_INCLUDE_DIRS})
    endif()
endif()

# ---------------------------------------------------------------------------
# 链接库
# ---------------------------------------------------------------------------
target_link_libraries(unicalib PUBLIC
    # 数学/几何
    Eigen3::Eigen
    Sophus::Sophus
    # 优化
    Ceres::ceres
    # B-Spline: basalt-headers + ctraj_basalt_compat (header-only, no ctraj lib)
    # 图像
    ${OpenCV_LIBS}
    # 点云
    ${PCL_LIBRARIES}
    # 序列化 & 配置
    yaml-cpp
    # 日志 & 格式化
    spdlog::spdlog
    ${FMT_LIBRARIES}
    # 枚举
    magic_enum::magic_enum
    # filesystem (GCC < 9 需要显式链接)
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
)

# opengv: link if target exists (found via cmake config, prebuilt lib, or bundled headers)
if(TARGET opengv)
  target_link_libraries(unicalib PUBLIC opengv)
endif()

# ufomap (可选): 使用正确的 CMake target 名 UFO::Map (而非 ufomap)
if(ufomap_FOUND)
  # UFO::Map INTERFACE_LINK_LIBRARIES 要求 lz4 和 tbb 作为 cmake target 或 linker flag
  # 若系统只有运行时库 (liblz4.so.1 无 .so 符号链接), 手动创建 IMPORTED target
  if(NOT TARGET lz4)
    find_library(LZ4_LIBRARY NAMES lz4
        PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
        PATH_SUFFIXES "" NO_DEFAULT_PATH)
    if(NOT LZ4_LIBRARY)
      # 回退: 搜索带版本号的 .so
      find_library(LZ4_LIBRARY NAMES liblz4.so.1 lz4
          PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
    endif()
    if(LZ4_LIBRARY)
      add_library(lz4 SHARED IMPORTED)
      set_target_properties(lz4 PROPERTIES IMPORTED_LOCATION "${LZ4_LIBRARY}")
      message(STATUS "lz4 library: ${LZ4_LIBRARY}")
    else()
      message(WARNING "liblz4 not found; UFO::Map may fail to link")
    endif()
  endif()
  target_link_libraries(unicalib PUBLIC UFO::Map)
endif()

# Pangolin (可视化): pango_display 包含 pangolin/display/display.h 所需的所有 headers
# pango_plot 包含 pangolin/plot/plotter.h (pangolin.h 中也包含了它)
# pango_video 包含 pangolin/video/video.h (pangolin.h 中也包含了它)
# pango_image 包含 pangolin/image/image_io.h (pangolin.h 中也包含了它)
if(UNICALIB_WITH_PANGOLIN)
  target_include_directories(unicalib PUBLIC ${PANGOLIN_INCLUDE_DIRS})
  target_link_libraries(unicalib PUBLIC pango_display pango_plot pango_video pango_image)
endif()

# OpenMP (可选)
if(OpenMP_CXX_FOUND AND UNICALIB_WITH_OPENMP)
  target_link_libraries(unicalib PUBLIC OpenMP::OpenMP_CXX)
endif()

# ROS2 (可选)
if(UNICALIB_WITH_ROS2)
  target_link_libraries(unicalib PUBLIC
      rclcpp::rclcpp
      rosbag2_cpp::rosbag2_cpp)
endif()

# ===========================================================================
# 可执行程序
# ===========================================================================
if(UNICALIB_BUILD_APPS)
  # 辅助函数: 添加标定应用
  function(add_calib_app target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE unicalib)
    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  endfunction()

  # IMU 内参标定
  add_calib_app(unicalib_imu_intrinsic
      apps/imu_intrinsic/main.cpp)

  # 相机内参标定 (针孔 + 鱼眼)
  add_calib_app(unicalib_camera_intrinsic
      apps/camera_intrinsic/main.cpp)

  # IMU-LiDAR 外参标定
  add_calib_app(unicalib_imu_lidar
      apps/imu_lidar_extrin/main.cpp)

  # LiDAR-Camera 外参标定
  add_calib_app(unicalib_lidar_camera
      apps/lidar_camera_extrin/main.cpp)

  # LiDAR-LiDAR 外参标定
  add_calib_app(unicalib_lidar_lidar
      apps/lidar_lidar_extrin/main.cpp)

  # Camera-Camera 外参标定
  add_calib_app(unicalib_cam_cam
      apps/cam_cam_extrin/main.cpp)

  # 联合标定 (全传感器)
  add_calib_app(unicalib_joint
      apps/joint_calib/main.cpp)

  # iKalibr 主程序和 IMU 内参工具仅在 UNICALIB_WITH_IKALIBR=ON 时构建
  if(UNICALIB_WITH_IKALIBR)
    add_executable(unicalib_ikalibr_prog
        src/ikalibr/exe/solver/main.cpp)
    target_link_libraries(unicalib_ikalibr_prog PRIVATE unicalib)

    add_executable(unicalib_imu_intri_tool
        src/ikalibr/nofree/imu_intri_calib.cpp)
    target_link_libraries(unicalib_imu_intri_tool PRIVATE unicalib)
  endif()
endif()

# ===========================================================================
# 测试 (可选)
# ===========================================================================
if(UNICALIB_BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    include(GoogleTest)
    add_subdirectory(tests)
endif()

# ===========================================================================
# 安装规则
# ===========================================================================
include(GNUInstallDirs)
install(TARGETS unicalib
    EXPORT UniCalibTargets
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY config/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/unicalib/config)

# ===========================================================================
# 构建摘要
# ===========================================================================
message(STATUS "")
message(STATUS "========= UniCalib v${PROJECT_VERSION} 构建配置 =========")
message(STATUS "  Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  Eigen3:        ${EIGEN3_VERSION_STRING}")
message(STATUS "  OpenCV:        ${OpenCV_VERSION}")
message(STATUS "  PCL:           ${PCL_VERSION}")
message(STATUS "  Ceres:         ${CERES_VERSION}")
message(STATUS "  spdlog:        ${spdlog_VERSION}")
message(STATUS "  fmt:           ${fmt_VERSION}")
message(STATUS "  OpenMP:        ${OpenMP_CXX_FOUND}")
message(STATUS "  opengv:        bundled headers (${_OPENGV_STUB})")
message(STATUS "  ufomap:        ${ufomap_FOUND}")
message(STATUS "  veta:          self-contained (veta-impl)")
message(STATUS "  Pangolin:      ${UNICALIB_WITH_PANGOLIN}")
message(STATUS "  ROS2:          ${UNICALIB_WITH_ROS2}")
message(STATUS "  Build apps:    ${UNICALIB_BUILD_APPS}")
message(STATUS "  PCL viz:       ${UNICALIB_WITH_PCL_VIZ}")
message(STATUS "=======================================================")
