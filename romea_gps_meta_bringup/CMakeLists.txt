cmake_minimum_required(VERSION 3.5)
project(romea_gps_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch profile launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(romea_gps_meta_bringup)

install(PROGRAMS
  scripts/generate_launch_file.py
  scripts/generate_urdf_description.py
  scripts/generate_configuration_file.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.flake8")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_gps_meta_bringup.yaml test_gps_meta_bringup.yaml)
  ament_add_pytest_test(test_gps_meta_bringup test/test_gps_meta_bringup.py)
  # ament_add_pytest_test(test_urdf_description test/test_urdf_description.py)
endif()

ament_package()
