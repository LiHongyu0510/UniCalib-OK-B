set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3)
find_package(Sophus)
find_package(Ceres)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/core CORE_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/factor FACTOR_SRC_FILES)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/ctraj/core CORE_HEADER_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/ctraj/factor FACTOR_HEADER_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/ctraj/nofree NOFREE_HEADER_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/ctraj/spline SPLINE_HEADER_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/ctraj/utils UTILS_HEADER_FILES)

add_library(
        ${LIBRARY_NAME} SHARED
        ${CORE_SRC_FILES} ${FACTOR_SRC_FILES}
        ${CORE_HEADER_FILES} ${FACTOR_HEADER_FILES} ${NOFREE_HEADER_FILES}
        ${SPLINE_HEADER_FILES} ${UTILS_HEADER_FILES}
)

set(ADDITIONAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

# 优先使用本地 cereal（thirdparty/cereal/include），保证 vector<shared_ptr<IMUFrame>> 等序列化一致
# tiny-viewer 已移除，使用 stub 头文件；cereal Eigen 序列化用 cereal_eigen_include
target_include_directories(
        ${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../cereal/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../tiny-viewer_stub>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../cereal_eigen_include>
        $<INSTALL_INTERFACE:include>
        ${ADDITIONAL_INCLUDE_DIRS}
)

target_link_libraries(
        ${LIBRARY_NAME} PUBLIC
        Sophus::Sophus
        ${CERES_LIBRARIES}
)

add_executable(${PROJECT_NAME}_prog ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(
        ${PROJECT_NAME}_prog PRIVATE
        ${LIBRARY_NAME}
)
