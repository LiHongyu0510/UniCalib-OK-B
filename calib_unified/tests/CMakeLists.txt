# UniCalib - 单元测试 CMake 配置

cmake_minimum_required(VERSION 3.14)
project(unicalib_tests)

# 查找依赖
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(GTest REQUIRED)

# 设置包含路径
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../include
)

# 测试可执行文件
add_executable(test_math_safety test_math_safety.cpp)
add_executable(test_lidar_lidar_calib test_lidar_lidar_calib.cpp)

# 链接库
target_link_libraries(test_math_safety
    ${OpenCV_LIBS}
    GTest::gtest
    GTest::gtest_main
)

# LiDAR-LiDAR 标定测试：链接主库 unicalib（由父项目定义）
if(TARGET unicalib)
  target_link_libraries(test_lidar_lidar_calib PRIVATE
    unicalib
    GTest::gtest
    GTest::gtest_main
  )
  target_include_directories(test_lidar_lidar_calib PRIVATE ${CMAKE_SOURCE_DIR}/../include)
endif()

# 添加测试
enable_testing()
include(GoogleTest)
add_test(NAME MathSafety COMMAND test_math_safety)
if(TARGET test_lidar_lidar_calib)
  add_test(NAME LidarLidarCalib COMMAND test_lidar_lidar_calib)
endif()

# 编译选项
target_compile_options(test_math_safety PRIVATE
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
)
target_compile_options(test_lidar_lidar_calib PRIVATE
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
)

# 调试信息
target_compile_options(test_math_safety PRIVATE $<$<CONFIG:Debug>:-g -O0>)
target_compile_options(test_math_safety PRIVATE $<$<CONFIG:Release>:-O3>)

# 定义测试启用宏
add_definitions(-DENABLE_TESTING)
