cmake_minimum_required(VERSION 3.5)
project(romea_gps_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_gps_meta_bringup.py)

install(PROGRAMS
  scripts/urdf_description.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_gps_meta_bringup.yaml test_gps_meta_bringup.yaml)
  configure_file(test/test_gps_serial_node_parameters.yaml test_gps_serial_node_parameters.yaml)
  configure_file(test/test_gps_tcp_client_node_parameters.yaml test_gps_tcp_client_node_parameters.yaml)
  configure_file(test/test_gps_nmea_navsat_driver_parameters.yaml test_gps_nmea_navsat_driver_parameters.yaml)
  ament_add_pytest_test(test_gps_driver_parameters test/test_gps_driver_parameters.py)
  ament_add_pytest_test(test_gps_meta_bringup test/test_gps_meta_bringup.py)
  ament_add_pytest_test(test_urdf_description test/test_urdf_description.py)
endif()

ament_package()
