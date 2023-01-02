cmake_minimum_required(VERSION 3.5)
project(romea_gps_utils)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_gps REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(serial REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/gps_data.cpp
  src/gps_data_conversions.cpp
  src/gps_data_diagnostics.cpp
  src/gps_serial_interface.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  nmea_msgs
  romea_core_common
  romea_core_gps
  romea_common_utils
  serial)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nmea_msgs)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_core_gps)
ament_export_dependencies(romea_common_utils)
ament_export_dependencies(serial)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

#if(BUILD_TESTING)

#    find_package(ament_lint_auto REQUIRED)
#    ament_lint_auto_find_test_dependencies()
#    add_subdirectory(test)

#endif()

ament_package()
