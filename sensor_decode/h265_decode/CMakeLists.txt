cmake_minimum_required(VERSION 3.8)
project(h265_decode)

find_package(ament_cmake REQUIRED)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/h265_decode_perception.py
  ${CMAKE_CURRENT_SOURCE_DIR}/h265_decode.py
  DESTINATION .
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ)  

ament_package()