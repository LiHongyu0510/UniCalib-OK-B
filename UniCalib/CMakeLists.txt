cmake_minimum_required(VERSION 3.8)
project(unicalib)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(
  PROGRAMS
    scripts/run_calibration.py
    scripts/collect_data.py
    scripts/visualize_results.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_intrinsic tests/test_intrinsic.py)
  ament_add_pytest_test(test_extrinsic tests/test_extrinsic.py)
endif()

ament_package()
