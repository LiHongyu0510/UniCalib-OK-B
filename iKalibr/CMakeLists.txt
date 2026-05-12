# iKalibr: Unified Targetless Spatiotemporal Calibration Framework
# Copyright 2024, School of Geodesy and Geomatics (SGG), Wuhan University, China
# https://github.com/Unsigned-Long/iKalibr.git
#
# Author: Shuolong Chen (shlchen@whu.edu.cn)
# GitHub: https://github.com/Unsigned-Long
#  ORCID: 0000-0002-5283-9057
#
# Purpose: See .h/.hpp file.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and this disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * The names of its contributors can not be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.14)
project(ikalibr)

# Disable unity build to avoid rosidl_typesupport_introspection_cpp "specialization after instantiation" errors
set(USE_CMAKE_UNITY_BUILD OFF CACHE STRINGS "whether use 'CMAKE_UNITY_BUILD' in building")

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.16" AND USE_CMAKE_UNITY_BUILD)
    set(CMAKE_UNITY_BUILD ON)
    message(STATUS "use 'CMAKE_UNITY_BUILD' in building!")
else ()
    set(CMAKE_UNITY_BUILD OFF)
    message(STATUS "do not use 'CMAKE_UNITY_BUILD' in building!")
endif ()

## Compile as C++17
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "RELEASE")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

# ctraj compat (basalt-headers) and veta-stub include their own deps; preinclude no longer required
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include ...")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

## Find ament_cmake and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# 显式添加 OpenCV 头文件路径，避免 colcon/unity 构建下 opencv2/core.hpp 找不到
if(OpenCV_INCLUDE_DIRS)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()
# 兼容：部分环境（如旧版/自定义 OpenCV）不提供 OpenCV:: 目标，回退到 OpenCV_LIBS
if(TARGET OpenCV::opencv_core)
    set(OpenCV_UTIL_LIBS OpenCV::opencv_core OpenCV::opencv_imgproc)
else()
    set(OpenCV_UTIL_LIBS ${OpenCV_LIBS})
endif()
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# PCL 头文件路径：全局添加，供 cloud_define.hpp、calib_solver.h、lidar_map_viewer 等使用
if(PCL_INCLUDE_DIRS)
    include_directories(${PCL_INCLUDE_DIRS})
endif()
find_package(spdlog REQUIRED)
find_package(fmt QUIET)
get_target_property(_spdlog_inc spdlog::spdlog INTERFACE_INCLUDE_DIRECTORIES)
if(_spdlog_inc)
    include_directories("${_spdlog_inc}")
endif()

# Optional third-party libraries (for ROS2 migration)
# These libraries are in iKalibr/thirdparty/ and need to be compiled separately
# If the libraries are not available, we'll skip linking them (limited functionality)
set(USE_THIRDPARTY_LIBS ON CACHE BOOL "Use third-party libraries (basalt-headers, veta-stub, ufomap, opengv, aslam_cv2)")

if (USE_THIRDPARTY_LIBS)
    message(STATUS "Trying to find third-party libraries...")

    # ── basalt-headers (replaces ctraj): header-only ──────────────────────────
    # Search order:
    #  1. In-tree thirdparty/basalt-headers (always available after build_thirdparty_ros2.sh)
    #  2. Sibling calib_unified checkout (host-only convenience, not in Docker)
    set(_basalt_headers "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers/include")
    if(NOT EXISTS "${_basalt_headers}/basalt/spline/se3_spline.h")
        # Allow override via cmake variable -DBASALT_HEADERS_DIR=...
        if(DEFINED BASALT_HEADERS_DIR AND EXISTS "${BASALT_HEADERS_DIR}/basalt/spline/se3_spline.h")
            set(_basalt_headers "${BASALT_HEADERS_DIR}")
        else()
            set(_basalt_headers "${CMAKE_CURRENT_SOURCE_DIR}/../calib_unified/thirdparty/basalt-headers/include")
        endif()
    endif()
    if(EXISTS "${_basalt_headers}/basalt/spline/se3_spline.h")
        include_directories("${_basalt_headers}")
        message(STATUS "  Found basalt-headers at ${_basalt_headers}")
        set(basalt_headers_FOUND TRUE)
    else()
        message(FATAL_ERROR
            "  basalt-headers NOT found!\n"
            "  Expected at: ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers/include\n"
            "  Run build_thirdparty_ros2.sh first, or set -DBASALT_HEADERS_DIR=<path>")
    endif()

    # ── ctraj compat layer (wraps basalt-headers with ctraj API) ─────────────
    set(_ctraj_compat "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj_basalt_compat/include")
    if(EXISTS "${_ctraj_compat}/ctraj/core/spline_bundle.h")
        include_directories("${_ctraj_compat}")
        message(STATUS "  Using ctraj-compat (basalt-headers backend) at ${_ctraj_compat}")
        set(ctraj_compat_FOUND TRUE)
    else()
        message(FATAL_ERROR "  ctraj_basalt_compat NOT found at ${_ctraj_compat}")
    endif()

    # ── Cereal (header-only serialization) ────────────────────────────────────
    set(_cereal_include "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/include")
    if(EXISTS "${_cereal_include}/cereal/cereal.hpp")
        include_directories("${_cereal_include}")
        message(STATUS "  Using cereal from thirdparty/cereal/include")
    elseif(EXISTS "/usr/include/cereal/cereal.hpp")
        message(STATUS "  Using system cereal")
    else()
        message(WARNING "  cereal NOT found - serialization may fail")
    endif()
    # cereal Eigen serialization
    foreach(_cereal_eigen_path
            "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/include"
            "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj/cereal_eigen_include")
        if(EXISTS "${_cereal_eigen_path}/cereal/types/eigen.hpp")
            include_directories("${_cereal_eigen_path}")
            break()
        endif()
    endforeach()

    # ── Sophus (required by basalt-headers / ctraj-compat) ────────────────────
    find_package(Sophus QUIET)
    if(Sophus_FOUND AND TARGET Sophus::Sophus)
        message(STATUS "  Found Sophus (cmake)")
    elseif(EXISTS "/usr/local/include/sophus/se3.hpp")
        include_directories("/usr/local/include")
        message(STATUS "  Using Sophus from /usr/local/include")
    elseif(EXISTS "/usr/include/sophus/se3.hpp")
        message(STATUS "  Using Sophus from /usr/include")
    else()
        message(WARNING "  Sophus NOT found - spline code may not compile")
    endif()

    # ── aslam_cv2 (replaces veta; provides camera models + distortion math) ──
    # The veta-stub already embeds the aslam distortion algorithms inline.
    # aslam_cv2 headers are added to include path for reference/optional direct use.
    # Search order:
    #  1. In-tree thirdparty/aslam_cv2 (from build_thirdparty_ros2.sh copy)
    #  2. Override via -DASLAM_CV2_DIR=<path>
    #  3. Sibling calib_unified checkout (host convenience)
    set(_aslam_cv2 "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/aslam_cv2")
    if(NOT EXISTS "${_aslam_cv2}/aslam_cv_cameras/include/aslam/cameras/camera.h")
        if(DEFINED ASLAM_CV2_DIR AND EXISTS "${ASLAM_CV2_DIR}/aslam_cv_cameras/include")
            set(_aslam_cv2 "${ASLAM_CV2_DIR}")
        else()
            set(_aslam_cv2 "${CMAKE_CURRENT_SOURCE_DIR}/../calib_unified/thirdparty/aslam_cv2")
        endif()
    endif()
    if(EXISTS "${_aslam_cv2}/aslam_cv_cameras/include/aslam/cameras/camera.h")
        # Add include paths (aslam_cv2 math is embedded inline in veta-stub,
        # so linking the library is optional; only headers needed for type visibility)
        include_directories("${_aslam_cv2}/aslam_cv_cameras/include"
                            "${_aslam_cv2}/aslam_cv_common/include")
        # Optionally compile aslam_cv_cameras as a static lib
        # (needed only if iKalibr sources include aslam headers directly)
        if(NOT TARGET aslam_cv_cameras)
            add_subdirectory(
                "${_aslam_cv2}"
                "${CMAKE_BINARY_DIR}/aslam_cv2-build"
                EXCLUDE_FROM_ALL
            )
        endif()
        set(aslam_cv2_FOUND TRUE)
        message(STATUS "  Found aslam_cv2 at ${_aslam_cv2} (distortion math inlined in veta-stub)")
    else()
        set(aslam_cv2_FOUND FALSE)
        message(STATUS "  aslam_cv2 not found - veta-stub uses self-contained distortion math (OK)")
    endif()

    # ── veta-stub (provides ns_veta API; distortion backed by aslam_cv2 math) ─
    # This is the in-tree replacement for external veta library.
    set(veta_FOUND FALSE)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/veta-stub/CMakeLists.txt")
        add_subdirectory(thirdparty/veta-stub ${CMAKE_BINARY_DIR}/veta-stub-build)
        add_library(veta::veta ALIAS veta)
        set(veta_FOUND TRUE)
        # Always include veta-stub headers (aslam_cv2 backed implementations)
        include_directories("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/veta-stub/include")
        message(STATUS "  Using veta-stub (aslam_cv2 math backend)")
    else()
        message(FATAL_ERROR "  veta-stub NOT found at thirdparty/veta-stub/")
    endif()

    # ── opengv (geometric vision) ─────────────────────────────────────────────
    set(opengv_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty-install/opengv-install/lib/cmake/opengv-1.0)
    find_package(opengv QUIET)

    # ── ufomap (voxel/surfel map; devel_surfel branch) ────────────────────────
    # Use the pre-built install from build_thirdparty_ros2.sh
    set(ufomap_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty-install/ufomap-install/lib/cmake/ufomap")
    set(ufomap_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty-install/ufomap-install/include")
    find_package(ufomap QUIET)

    # ── Status messages ───────────────────────────────────────────────────────
    if(ufomap_FOUND)
        message(STATUS "  Found ufomap (surfel)")
        include_directories(${ufomap_INCLUDE_DIR})
    else()
        message(WARNING "  ufomap NOT found - voxel/surfel features disabled")
    endif()

    if(opengv_FOUND)
        message(STATUS "  Found opengv")
    else()
        message(WARNING "  opengv NOT found - geometric vision features disabled")
    endif()

else()
    message(STATUS "Third-party libraries are disabled - functionality will be limited")
    set(basalt_headers_FOUND FALSE)
    set(ctraj_compat_FOUND   FALSE)
    set(veta_FOUND           FALSE)
    set(opengv_FOUND         FALSE)
    set(ufomap_FOUND         FALSE)
    set(aslam_cv2_FOUND      FALSE)
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(magic_enum QUIET)
# magic_enum 头文件路径：显式添加，避免 colcon 下 INTERFACE 未传递（Docker 中安装于 /usr/local）
set(_magic_enum_inc_done OFF)
if(magic_enum_FOUND AND TARGET magic_enum::magic_enum)
    get_target_property(_magic_enum_inc magic_enum::magic_enum INTERFACE_INCLUDE_DIRECTORIES)
    if(_magic_enum_inc)
        include_directories(${_magic_enum_inc})
        set(_magic_enum_inc_done ON)
    endif()
endif()
if(NOT _magic_enum_inc_done)
    set(_magic_enum_deps "${CMAKE_CURRENT_SOURCE_DIR}/../docker/deps/magic_enum/include")
    if(EXISTS "${_magic_enum_deps}/magic_enum")
        include_directories("${_magic_enum_deps}")
        message(STATUS "  Using magic_enum from docker/deps (fallback)")
    else()
        # 系统安装的 magic_enum（如 /usr/include 或 /usr/local/include）
        set(_magic_enum_sys "/usr/local/include;/usr/include")
        foreach(_d ${_magic_enum_sys})
            if(EXISTS "${_d}/magic_enum/magic_enum.hpp")
                include_directories("${_d}")
                set(_magic_enum_inc_done ON)
                message(STATUS "  Using magic_enum from ${_d}")
                break()
            endif()
        endforeach()
    endif()
endif()
# magic_enum works in range of [MAGIC_ENUM_RANGE_MIN, MAGIC_ENUM_RANGE_MAX]
add_definitions(-DMAGIC_ENUM_RANGE_MIN=0)
# 1024 = 2^10
add_definitions(-DMAGIC_ENUM_RANGE_MAX=1023)
# 使用内置 viewer stub（无 tiny-viewer），实时 GUI 关闭；可视化可后续用文件导出/PCL 等替代
set(IKALIBR_VIEWER_DISABLED TRUE)
find_package(yaml-cpp)

#define PCL_NO_PRECOMPILE [it's very important for Custom pcl Point type]
add_definitions(-DPCL_NO_PRECOMPILE)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/calib CALIB_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/solver SOLVER_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/core CORE_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/sensor SENSOR_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/config CONFIG_SRC_FILES)

if(IKALIBR_VIEWER_DISABLED)
    list(REMOVE_ITEM CORE_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/core/vision_only_sfm.cpp)
    # Add stub implementation instead (avoids tiny-viewer dependency)
    list(APPEND CORE_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/core/vision_only_sfm_stub.cpp)
    add_definitions(-DIKALIBR_VIEWER_DISABLED)
endif()
# viewer.cpp depends on tiny-viewer (removed), but the visual analysis files are still needed
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/viewer VIEWER_SRC_FILES)
list(REMOVE_ITEM VIEWER_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/viewer/viewer.cpp)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/util UTIL_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/factor FACTOR_SRC_FILES)


## Specify additional locations of header files
###################
# libikalibr_util #
###################
add_library(
        ${PROJECT_NAME}_util SHARED
        # source files
        ${UTIL_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_util PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_util
        ${OpenCV_UTIL_LIBS}
)
if (USE_THIRDPARTY_LIBS AND veta_FOUND)
    target_link_libraries(${PROJECT_NAME}_util veta)
endif()
ament_target_dependencies(${PROJECT_NAME}_util rclcpp ament_index_cpp)

#####################
# libikalibr_sensor #
#####################
add_library(
        ${PROJECT_NAME}_sensor SHARED
        # source files
        ${SENSOR_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_sensor PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_sensor
        ${PROJECT_NAME}_util
)
if (USE_THIRDPARTY_LIBS AND veta_FOUND)
    target_link_libraries(${PROJECT_NAME}_sensor veta)
endif()
# 注意：ament_target_dependencies 仅用于外部 ament 包，不能传入当前包名 ${PROJECT_NAME}
ament_target_dependencies(${PROJECT_NAME}_sensor rclcpp ament_index_cpp sensor_msgs cv_bridge pcl_conversions)

#####################
# libikalibr_config #
#####################
add_library(
        ${PROJECT_NAME}_config SHARED
        # source files
        ${CONFIG_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_config PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_config)
if (USE_THIRDPARTY_LIBS AND veta_FOUND)
    target_link_libraries(${PROJECT_NAME}_config veta)
endif()
ament_target_dependencies(${PROJECT_NAME}_config rclcpp ament_index_cpp)

#####################
# libikalibr_factor #
#####################
add_library(
        ${PROJECT_NAME}_factor SHARED
        # source files
        ${FACTOR_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_factor PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_factor)
if (USE_THIRDPARTY_LIBS AND veta_FOUND)
    target_link_libraries(${PROJECT_NAME}_factor veta)
endif()
if (USE_THIRDPARTY_LIBS AND opengv_FOUND)
    target_link_libraries(${PROJECT_NAME}_factor opengv)
endif()
ament_target_dependencies(${PROJECT_NAME}_factor rclcpp ament_index_cpp)

######################
# precompile headers #
######################
#file(GLOB FACTOR_HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/factor/*.h"
#        "${CMAKE_CURRENT_SOURCE_DIR}/include/factor/*.hpp")
#target_precompile_headers(
#        ${PROJECT_NAME}_factor PUBLIC
#        ${FACTOR_HEADER_FILES}
#)

# viewer 模块已禁用（使用 stub），但不影响标定核心功能
# #####################
# # libikalibr_viewer #
# #####################
# add_library(
#         ${PROJECT_NAME}_viewer SHARED
#         # source files
#         ${VIEWER_SRC_FILES}
# )
# 
# target_include_directories(
#         ${PROJECT_NAME}_viewer PUBLIC
#         # include
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>
# )
# 
# target_link_libraries(
#         ${PROJECT_NAME}_viewer PUBLIC
#         # thirdparty libraries (conditional)
# )
# if (USE_THIRDPARTY_LIBS AND ctraj_FOUND AND veta_FOUND AND opengv_FOUND AND ufomap_FOUND)
#     target_link_libraries(${PROJECT_NAME}_viewer PUBLIC ctraj veta opengv)
# endif()
# 
# #target_precompile_headers(
# #        ${PROJECT_NAME}_viewer REUSE_FROM
# #        ${PROJECT_NAME}_factor
# #)


###################
# libikalibr_core #
###################
add_library(
        ${PROJECT_NAME}_core SHARED
        # source files
        ${CORE_SRC_FILES}
        # visual analysis helpers (viewer.cpp excluded - uses only stub viewer)
        ${VIEWER_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_core PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_core)
if (USE_THIRDPARTY_LIBS AND veta_FOUND)
    target_link_libraries(${PROJECT_NAME}_core veta)
endif()
if (USE_THIRDPARTY_LIBS AND opengv_FOUND)
    target_link_libraries(${PROJECT_NAME}_core opengv)
endif()
if (OPENMP_FOUND)
    target_link_libraries(${PROJECT_NAME}_core OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(${PROJECT_NAME}_core rclcpp ament_index_cpp sensor_msgs cv_bridge)

#target_precompile_headers(
#        ${PROJECT_NAME}_core REUSE_FROM
#        ${PROJECT_NAME}_factor
#)

####################
# libikalibr_calib #
####################
add_library(
        ${PROJECT_NAME}_calib SHARED
        # source files
        ${CALIB_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_calib PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_calib)
if (USE_THIRDPARTY_LIBS AND veta_FOUND)
    target_link_libraries(${PROJECT_NAME}_calib veta)
endif()
if (USE_THIRDPARTY_LIBS AND opengv_FOUND)
    target_link_libraries(${PROJECT_NAME}_calib opengv)
endif()
if (OPENMP_FOUND)
    target_link_libraries(${PROJECT_NAME}_calib OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(${PROJECT_NAME}_calib rclcpp ament_index_cpp sensor_msgs pcl_conversions)

#target_precompile_headers(
#        ${PROJECT_NAME}_calib REUSE_FROM
#        ${PROJECT_NAME}_factor
#)

#####################
# libikalibr_solver #
#####################
add_library(
        ${PROJECT_NAME}_solver SHARED
        # source files
        ${SOLVER_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME}_solver PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 镜像/系统安装的 PCL 可能不导出 PCL::pcl，使用 PCL_LIBRARIES 兼容
if(TARGET PCL::pcl)
    target_link_libraries(${PROJECT_NAME}_solver PCL::pcl)
else()
    target_link_libraries(${PROJECT_NAME}_solver ${PCL_LIBRARIES})
endif()
if (USE_THIRDPARTY_LIBS AND veta_FOUND)
    target_link_libraries(${PROJECT_NAME}_solver veta)
endif()
if (USE_THIRDPARTY_LIBS AND opengv_FOUND)
    target_link_libraries(${PROJECT_NAME}_solver opengv)
endif()
if (OPENMP_FOUND)
    target_link_libraries(${PROJECT_NAME}_solver OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(${PROJECT_NAME}_solver rclcpp ament_index_cpp sensor_msgs cv_bridge pcl_conversions)

#target_precompile_headers(
#        ${PROJECT_NAME}_solver REUSE_FROM
#        ${PROJECT_NAME}_factor
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#ament_package()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
## add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_ikalibr.cpp)

add_executable(
        ${PROJECT_NAME}_prog
        exe/solver/main.cpp
)
if(NOT IKALIBR_VIEWER_DISABLED)
add_executable(
        ${PROJECT_NAME}_learn
        exe/nofree/learn.cpp
        src/nofree/logo_svg.cpp
        src/nofree/data_collect_demo.cpp
)
add_executable(
        ${PROJECT_NAME}_lidar_map_viewer
        exe/tool/lidar_map_viewer.cpp
)
endif()
add_executable(
        ${PROJECT_NAME}_data_format_transformer
        exe/tool/data_format_transformer.cpp
)
# 以下 4 个工具依赖 ROS1 (rosbag、ros::init)，ROS2 环境下默认不构建；若需构建请安装 ros-humble-ros1-bridge 或单独提供 ROS1 环境并设置 BUILD_ROS1_BAG_TOOLS=ON
set(BUILD_ROS1_BAG_TOOLS OFF CACHE BOOL "Build ROS1 bag tools (imgs_to_bag, raw_inertial_to_bag, bag_merge, bag_topic_downsample)")
if(BUILD_ROS1_BAG_TOOLS)
add_executable(
        ${PROJECT_NAME}_bag_merge
        exe/tool/bag_merge.cpp
        src/nofree/bag_merge.cpp
)
add_executable(
        ${PROJECT_NAME}_bag_topic_downsample
        exe/tool/bag_topic_downsample.cpp
)
add_executable(
        ${PROJECT_NAME}_imgs_to_bag
        exe/tool/imgs_to_bag.cpp
)
add_executable(
        ${PROJECT_NAME}_raw_inertial_to_bag
        exe/tool/raw_inertial_to_bag.cpp
)
target_include_directories(${PROJECT_NAME}_bag_merge PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_bag_merge PRIVATE ${PROJECT_NAME}_util ${YAML_CPP_LIBRARIES})
target_include_directories(${PROJECT_NAME}_bag_topic_downsample PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_bag_topic_downsample PRIVATE ${PROJECT_NAME}_util ${YAML_CPP_LIBRARIES})
target_include_directories(${PROJECT_NAME}_imgs_to_bag PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_imgs_to_bag PRIVATE ${PROJECT_NAME}_util)
target_include_directories(${PROJECT_NAME}_raw_inertial_to_bag PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_raw_inertial_to_bag PRIVATE ${PROJECT_NAME}_util)
endif()
add_executable(
        ${PROJECT_NAME}_imu_intri_calib
        exe/tool/imu_intri_calib.cpp
        src/nofree/imu_intri_calib.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, following renames
## target back to shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
## set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

###################
# libikalibr_prog #
###################
target_include_directories(
        ${PROJECT_NAME}_prog PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
        ${PROJECT_NAME}_prog PRIVATE

        # dependent library is placed after library that depends on it.
        ${PROJECT_NAME}_solver
        ${PROJECT_NAME}_calib
        ${PROJECT_NAME}_factor
        ${PROJECT_NAME}_core
        # ${PROJECT_NAME}_viewer  # 已禁用
        ${PROJECT_NAME}_sensor
        ${PROJECT_NAME}_config
        ${PROJECT_NAME}_util

        # thirdparty
        ${YAML_CPP_LIBRARIES}
        $<$<BOOL:${OPENMP_FOUND}>:OpenMP::OpenMP_CXX>
        $<$<BOOL:${fmt_FOUND}>:fmt::fmt>
)
####################
# libikalibr_learn #
####################
if(NOT IKALIBR_VIEWER_DISABLED)
target_include_directories(
        ${PROJECT_NAME}_learn PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
        ${PROJECT_NAME}_learn PRIVATE

        # iKalibr libraries
        ${PROJECT_NAME}_util

        # thirdparty
        ${YAML_CPP_LIBRARIES}
)
endif()

###############################
# libikalibr_lidar_map_viewer #
###############################
if(NOT IKALIBR_VIEWER_DISABLED)
target_include_directories(
        ${PROJECT_NAME}_lidar_map_viewer PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
        ${PROJECT_NAME}_lidar_map_viewer PRIVATE

        # thirdparty
        ${PROJECT_NAME}_util
)
endif()

######################################
# libikalibr_data_format_transformer #
######################################
target_include_directories(
        ${PROJECT_NAME}_data_format_transformer PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
        ${PROJECT_NAME}_data_format_transformer PRIVATE

        # dependent library is placed after library that depends on it.
        ${PROJECT_NAME}_calib
        ${PROJECT_NAME}_factor
        ${PROJECT_NAME}_core
        # ${PROJECT_NAME}_viewer  # 已禁用
        ${PROJECT_NAME}_sensor
        ${PROJECT_NAME}_config
        ${PROJECT_NAME}_util

        # thirdparty
        ${YAML_CPP_LIBRARIES}
        $<$<BOOL:${OPENMP_FOUND}>:OpenMP::OpenMP_CXX>
        $<$<BOOL:${fmt_FOUND}>:fmt::fmt>
)
##############################
# libikalibr_imu_intri_calib #
##############################
target_include_directories(
        ${PROJECT_NAME}_imu_intri_calib PUBLIC
        # include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
        ${PROJECT_NAME}_imu_intri_calib PRIVATE

        # dependent library is placed after library that depends on it.
        ${PROJECT_NAME}_calib
        ${PROJECT_NAME}_factor
        ${PROJECT_NAME}_core
        # ${PROJECT_NAME}_viewer  # 已禁用
        ${PROJECT_NAME}_sensor
        ${PROJECT_NAME}_config
        ${PROJECT_NAME}_util

        # thirdparty
        ${YAML_CPP_LIBRARIES}
        $<$<BOOL:${OPENMP_FOUND}>:OpenMP::OpenMP_CXX>
        $<$<BOOL:${fmt_FOUND}>:fmt::fmt>
)

#############
## Install ##
#############

# Install libraries
install(TARGETS
        ${PROJECT_NAME}_util
        ${PROJECT_NAME}_sensor
        ${PROJECT_NAME}_config
        ${PROJECT_NAME}_factor
        # ${PROJECT_NAME}_viewer  # 已禁用
        ${PROJECT_NAME}_core
        ${PROJECT_NAME}_calib
        ${PROJECT_NAME}_solver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install executables
set(_ikalibr_install_execs ${PROJECT_NAME}_prog ${PROJECT_NAME}_data_format_transformer ${PROJECT_NAME}_imu_intri_calib)
if(NOT IKALIBR_VIEWER_DISABLED)
    list(APPEND _ikalibr_install_execs ${PROJECT_NAME}_learn ${PROJECT_NAME}_lidar_map_viewer)
endif()
if(BUILD_ROS1_BAG_TOOLS)
    list(APPEND _ikalibr_install_execs ${PROJECT_NAME}_bag_merge ${PROJECT_NAME}_bag_topic_downsample
            ${PROJECT_NAME}_imgs_to_bag ${PROJECT_NAME}_raw_inertial_to_bag)
endif()
install(TARGETS ${_ikalibr_install_execs}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
        DESTINATION include
        PATTERN "*.h")
install(DIRECTORY include/
        DESTINATION include
        PATTERN "*.hpp")
install(DIRECTORY include/
        DESTINATION include
        PATTERN ".svn" EXCLUDE
)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        PATTERN "*.launch")
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        PATTERN "*.py")

# Install Python scripts
install(DIRECTORY script/
        DESTINATION share/${PROJECT_NAME}/script
        PATTERN "*.py")
install(DIRECTORY script/
        DESTINATION share/${PROJECT_NAME}/script
        PATTERN "__pycache__" EXCLUDE
)

# Install model files
install(DIRECTORY model/
        DESTINATION share/${PROJECT_NAME}/model
        PATTERN "*.obj"
)

# Install config files
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        PATTERN "*.yaml")
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        PATTERN "*.yml")
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        PATTERN "*.json"
)

## ament package

## ROS2 Message Generation
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate ROS2 interfaces (messages, services, actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PropheseeEvent.msg"
  "msg/PropheseeEventArray.msg"
  "msg/DVSEvent.msg"
  "msg/DVSEventArray.msg"
  "msg/LivoxCustomMsg.msg"
  "msg/LivoxCustomPoint.msg"
  "msg/SbgImuData.msg"
  "msg/SbgImuStatus.msg"
  "msg/AinsteinRadarTarget.msg"
  "msg/AinsteinRadarTargetArray.msg"
  "msg/AWR1843RadarScan.msg"
  "msg/AWR1843RadarScanCustom.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Link generated message typesupport to all libraries that include ikalibr messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
foreach(lib_target
    ${PROJECT_NAME}_sensor
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_calib
    ${PROJECT_NAME}_solver)
  if(TARGET ${lib_target})
    target_link_libraries(${lib_target} "${cpp_typesupport_target}")
    target_include_directories(${lib_target} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)
  endif()
endforeach()

ament_package()
