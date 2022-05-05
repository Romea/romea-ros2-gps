#find_package(ament_cmake_gtest REQUIRED)

#configure_file(test_gazebo_ros_gps_sensor.world test_gazebo_ros_gps_sensor.world COPYONLY)

#ament_add_gtest(
# ${PROJECT_NAME}_test_gazebo_ros_gps_sensor
# test_gazebo_ros_gps_sensor.cpp
# WORKING_DIRECTORY
#   ${CMAKE_CURRENT_BINARY_DIR}
# # Long timeout because has to run gazebo several times
# TIMEOUT
#   120
# )

#target_link_libraries(${PROJECT_NAME}_test_gazebo_ros_gps_sensor gazebo_test_fixture)

#ament_target_dependencies(${PROJECT_NAME}_test_gazebo_ros_gps_sensor
#  "gazebo_dev"
#  "gazebo_msgs"
#  "gazebo_ros"
#  "geometry_msgs"
#  "sensor_msgs"
#  "nmea_msgs"
#  "romea_core_common"
#  "romea_core_gps"
#)
