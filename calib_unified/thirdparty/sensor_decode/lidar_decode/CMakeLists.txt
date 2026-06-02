cmake_minimum_required(VERSION 3.8)
project(lidar_decode)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Compiler options for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies (pcl_ros not used in sources; minimal images often lack it)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# Headers like pcl/conversions.h come from the system PCL dev package (libpcl-dev)
find_package(PCL REQUIRED COMPONENTS common io)

# Define the executable and link libraries
add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
