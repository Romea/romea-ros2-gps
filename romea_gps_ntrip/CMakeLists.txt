cmake_minimum_required(VERSION 3.8)
project(romea_gps_ntrip)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/serial_nmea_ntrip
  scripts/tcp_nmea_ntrip
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
