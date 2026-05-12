cmake_minimum_required(VERSION 3.14)
project(aslam_cv2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp QUIET)

# ── glog: try cmake, then pkg-config, then manual ───────────────────────────
find_package(glog QUIET)
if(NOT glog_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(GLOG libglog QUIET)
    endif()
endif()
if(NOT glog_FOUND AND NOT GLOG_FOUND)
    find_path(GLOG_INCLUDE_DIR glog/logging.h PATHS /usr/include /usr/local/include)
    find_library(GLOG_LIBRARY  glog           PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
    if(GLOG_INCLUDE_DIR AND GLOG_LIBRARY)
        add_library(glog::glog UNKNOWN IMPORTED)
        set_target_properties(glog::glog PROPERTIES
            IMPORTED_LOCATION "${GLOG_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}")
        set(glog_FOUND TRUE)
    endif()
endif()

# ── aslam_cv_common (header-only) ───────────────────────────────────────────
add_library(aslam_cv_common INTERFACE)
target_include_directories(aslam_cv_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/aslam_cv_common/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(aslam_cv_common INTERFACE Eigen3::Eigen)

# ── aslam_cv_cameras sources ─────────────────────────────────────────────────
# Note: camera.cc is excluded because it includes yaml-serialization.h which
# requires yaml-cpp for YAML-based camera loading (not used by iKalibr).
# iKalibr uses the distortion models and camera math only.
set(ASLAM_CAMERAS_SRCS
    aslam_cv_cameras/src/camera-pinhole.cc
    aslam_cv_cameras/src/camera-unified-projection.cc
    aslam_cv_cameras/src/distortion.cc
    aslam_cv_cameras/src/distortion-equidistant.cc
    aslam_cv_cameras/src/distortion-fisheye.cc
    aslam_cv_cameras/src/distortion-radtan.cc
)

# Include camera.cc only if yaml-cpp is available
if(yaml-cpp_FOUND OR YAML_CPP_LIBRARIES)
    list(APPEND ASLAM_CAMERAS_SRCS aslam_cv_cameras/src/camera.cc)
endif()

add_library(aslam_cv_cameras STATIC ${ASLAM_CAMERAS_SRCS})

target_include_directories(aslam_cv_cameras PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/aslam_cv_cameras/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/aslam_cv_common/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(aslam_cv_cameras PUBLIC
    Eigen3::Eigen
    aslam_cv_common
)

# glog
if(TARGET glog::glog)
    target_link_libraries(aslam_cv_cameras PUBLIC glog::glog)
elseif(GLOG_FOUND)
    target_include_directories(aslam_cv_cameras PUBLIC ${GLOG_INCLUDE_DIRS})
    target_link_libraries(aslam_cv_cameras PUBLIC ${GLOG_LIBRARIES})
endif()

# yaml-cpp (only needed if camera.cc was included)
if(yaml-cpp_FOUND)
    target_link_libraries(aslam_cv_cameras PUBLIC yaml-cpp)
elseif(YAML_CPP_LIBRARIES)
    target_link_libraries(aslam_cv_cameras PUBLIC ${YAML_CPP_LIBRARIES})
    if(YAML_CPP_INCLUDE_DIRS)
        target_include_directories(aslam_cv_cameras PUBLIC ${YAML_CPP_INCLUDE_DIRS})
    endif()
endif()

# Suppress warnings from external code
target_compile_options(aslam_cv_cameras PRIVATE -w)
