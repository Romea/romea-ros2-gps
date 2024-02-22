cmake_minimum_required(VERSION 3.5)
project(romea_gps_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(romea_gps_utils REQUIRED)
find_package(romea_common_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gps_serial_driver.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "ROMEA_GPS_DRIVER_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  romea_core_common
  romea_core_gps
  romea_common_utils
  romea_gps_utils
  geometry_msgs
  sensor_msgs
  mavros_msgs
  nmea_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::ros2::GpsSerialDriver"
  EXECUTABLE serial_driver_node
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
   find_package(ament_lint_auto REQUIRED)
   list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
   ament_lint_auto_find_test_dependencies()
endif()

ament_package()
