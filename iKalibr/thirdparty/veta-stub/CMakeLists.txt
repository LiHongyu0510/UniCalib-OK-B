cmake_minimum_required(VERSION 3.14)
project(veta-stub LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

add_library(veta INTERFACE)
target_include_directories(veta INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(veta INTERFACE Eigen3::Eigen)

# cereal (header-only serialization used in pinhole.h serialize())
set(_cereal "${CMAKE_CURRENT_SOURCE_DIR}/../cereal/include")
if(EXISTS "${_cereal}/cereal/cereal.hpp")
  target_include_directories(veta INTERFACE "${_cereal}")
endif()

install(TARGETS veta EXPORT vetaTargets)
install(EXPORT vetaTargets FILE vetaTargets.cmake NAMESPACE veta:: DESTINATION lib/cmake/veta)
install(DIRECTORY include/veta DESTINATION include)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/vetaConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/vetaConfig.cmake
  INSTALL_DESTINATION lib/cmake/veta
)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/vetaConfigVersion.cmake VERSION 0.1.0 COMPATIBILITY AnyNewerVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vetaConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/vetaConfigVersion.cmake DESTINATION lib/cmake/veta)
