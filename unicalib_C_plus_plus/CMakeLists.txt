cmake_minimum_required(VERSION 3.14)
project(unicalib_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# 依赖: Eigen3 (必需), yaml-cpp (可选), OpenCV (可选，用于鱼眼内参标定)
# -----------------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

option(UNICALIB_USE_YAML "Use yaml-cpp for config" ON)
if(UNICALIB_USE_YAML)
  find_package(yaml-cpp QUIET)
endif()

option(UNICALIB_USE_OPENCV "Use OpenCV for fisheye intrinsic calibration" ON)
if(UNICALIB_USE_OPENCV)
  find_package(OpenCV QUIET COMPONENTS core imgproc calib3d imgcodecs)
endif()

option(UNICALIB_USE_OPEN3D "Use Open3D for realtime visualization" OFF)
if(UNICALIB_USE_OPEN3D)
  find_package(Open3D QUIET)
endif()

# -----------------------------------------------------------------------------
# 库: unicalib_cpp
# -----------------------------------------------------------------------------
set(UNICALIB_HEADERS
  include/unicalib/sensor_config.hpp
  include/unicalib/calib_result.hpp
  include/unicalib/transforms.hpp
  include/unicalib/allan_variance.hpp
  include/unicalib/data_manager.hpp
  include/unicalib/imu_intrinsic.hpp
  include/unicalib/camera_fisheye.hpp
  include/unicalib/process_runner.hpp
  include/unicalib/external_tools_config.hpp
  include/unicalib/dm_calib_wrapper.hpp
  include/unicalib/learn_to_calib_wrapper.hpp
  include/unicalib/mias_lcec_wrapper.hpp
  include/unicalib/feature_matching.hpp
  include/unicalib/ikalibr_wrapper.hpp
  include/unicalib/click_calib_wrapper.hpp
  include/unicalib/reprojection.hpp
  include/unicalib/report_generator.hpp
  include/unicalib/visualization_v2.hpp
  include/unicalib/interactive_report.hpp
  include/unicalib/realtime_visualizer.hpp
  include/unicalib/system.hpp
  include/unicalib/exceptions.hpp
  include/unicalib/logger.hpp
  include/unicalib/data_validator.hpp
  include/unicalib/calibration_quality.hpp
  include/unicalib/iterative_refinement.hpp
  include/unicalib/temporal_calibration.hpp
)
set(UNICALIB_SOURCES
  src/sensor_config.cpp
  src/calib_result.cpp
  src/transforms.cpp
  src/allan_variance.cpp
  src/data_manager.cpp
  src/imu_intrinsic.cpp
  src/camera_fisheye.cpp
  src/process_runner.cpp
  src/external_tools_config.cpp
  src/dm_calib_wrapper.cpp
  src/learn_to_calib_wrapper.cpp
  src/mias_lcec_wrapper.cpp
  src/feature_matching.cpp
  src/ikalibr_wrapper.cpp
  src/click_calib_wrapper.cpp
  src/reprojection.cpp
  src/report_generator.cpp
  src/visualization_v2.cpp
  src/interactive_report.cpp
  src/realtime_visualizer.cpp
  src/system.cpp
  src/exceptions.cpp
  src/logger.cpp
  src/data_validator.cpp
  src/calibration_quality.cpp
  src/iterative_refinement.cpp
  src/temporal_calibration.cpp
)

add_library(unicalib_cpp ${UNICALIB_SOURCES} ${UNICALIB_HEADERS})
target_include_directories(unicalib_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(unicalib_cpp PUBLIC Eigen3::Eigen)
if(yaml-cpp_FOUND)
  target_link_libraries(unicalib_cpp PUBLIC yaml-cpp)
  target_compile_definitions(unicalib_cpp PUBLIC UNICALIB_USE_YAML=1)
endif()
if(OpenCV_FOUND)
  target_include_directories(unicalib_cpp PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(unicalib_cpp PUBLIC ${OpenCV_LIBS})
  target_compile_definitions(unicalib_cpp PUBLIC UNICALIB_USE_OPENCV=1)
endif()
if(Open3D_FOUND)
  target_include_directories(unicalib_cpp PRIVATE ${Open3D_INCLUDE_DIRS})
  target_link_libraries(unicalib_cpp PUBLIC ${Open3D_LIBRARIES})
  target_compile_definitions(unicalib_cpp PUBLIC UNICALIB_USE_OPEN3D=1)
endif()

# -----------------------------------------------------------------------------
# 可执行: 示例 / 测试
# -----------------------------------------------------------------------------
add_executable(unicalib_example examples/example_main.cpp)
target_link_libraries(unicalib_example PRIVATE unicalib_cpp)
target_compile_features(unicalib_example PRIVATE cxx_std_17)

add_executable(unicalib_viz_example examples/visualization_example.cpp)
target_link_libraries(unicalib_viz_example PRIVATE unicalib_cpp)
target_compile_features(unicalib_viz_example PRIVATE cxx_std_17)

add_executable(unicalib_realtime_example examples/realtime_visualization_example.cpp)
target_link_libraries(unicalib_realtime_example PRIVATE unicalib_cpp)
target_compile_features(unicalib_realtime_example PRIVATE cxx_std_17)

# 优化示例
add_executable(unicalib_optimization_example examples/optimization_example.cpp)
target_link_libraries(unicalib_optimization_example PRIVATE unicalib_cpp)
target_compile_features(unicalib_optimization_example PRIVATE cxx_std_17)

# -----------------------------------------------------------------------------
# 安装 (可选)
# -----------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS unicalib_cpp
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/unicalib DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
