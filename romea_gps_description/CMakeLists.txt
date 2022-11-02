cmake_minimum_required(VERSION 3.5)
project(romea_gps_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/gps_description.py)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
