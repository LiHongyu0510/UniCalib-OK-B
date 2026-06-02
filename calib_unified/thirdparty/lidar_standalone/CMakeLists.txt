cmake_minimum_required(VERSION 3.8)
project(lidar_decode_standalone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED COMPONENTS common io)

add_executable(lidar_decode_standalone src/main.cpp)

target_include_directories(lidar_decode_standalone PRIVATE ${PCL_INCLUDE_DIRS} include)
target_link_libraries(lidar_decode_standalone ${PCL_LIBRARIES})
target_compile_definitions(lidar_decode_standalone PRIVATE ${PCL_DEFINITIONS})

install(TARGETS lidar_decode_standalone
  DESTINATION bin)
