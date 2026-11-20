find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(
${PROJECT_NAME}_test_gazebo_ros_gps_sensor
test_gazebo_ros_gps_sensor.cpp
WORKING_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}
# Long timeout because has to run gazebo several times
TIMEOUT
  10
)

target_link_libraries(${PROJECT_NAME}_test_gazebo_ros_gps_sensor PUBLIC gazebo_test_fixture)

target_link_libraries(${PROJECT_NAME}_test_gazebo_ros_gps_sensor PUBLIC
  ${gazebo_dev_LIBRARIES}
  ${gazebo_msgs_TARGETS}
  ${gazebo_ros_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nmea_msgs_TARGETS}
  romea_core_common::romea_core_common
  romea_core_gps::romea_core_gps)

target_include_directories(${PROJECT_NAME}_test_gazebo_ros_gps_sensor SYSTEM PUBLIC
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS})

target_compile_definitions(${PROJECT_NAME}_test_gazebo_ros_gps_sensor PUBLIC
  ${gazebo_dev_DEFINITIONS}
  ${gazebo_ros_DEFINITIONS})
