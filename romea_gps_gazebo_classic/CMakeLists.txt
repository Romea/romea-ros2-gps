cmake_minimum_required(VERSION 3.10)
project(romea_gps_gazebo_classic)

# if($ENV{ROS_DISTRO})
set(ROS_DISTRO $ENV{ROS_DISTRO})
# endif()

if(${ROS_DISTRO} STRLESS_EQUAL "iron")

    set(PLUGIN_LIB_NAME "romea_gazebo_ros_gps_sensor")

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      # we dont use add_compile_options with pedantic in message packages
      # because the Python C extensions dont comply with it
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(romea_core_common REQUIRED)
    find_package(romea_core_gps REQUIRED)
    find_package(gazebo_dev REQUIRED)
    find_package(gazebo_ros REQUIRED)
    find_package(gazebo_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nmea_msgs REQUIRED)
    find_package(sdformat9 REQUIRED)

    link_directories(${gazebo_dev_LIBRARY_DIRS})

    # gazebo_ros_gps_sensor
    add_library(${PLUGIN_LIB_NAME} SHARED
      src/gazebo_ros_gps_sensor.cpp
    )
    target_include_directories(${PLUGIN_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

    target_link_libraries(${PLUGIN_LIB_NAME} PUBLIC
      ${gazebo_ros_LIBRARIES}
      ${sensor_msgs_TARGETS}
      ${geometry_msgs_TARGETS}
      ${nmea_msgs_TARGETS}
      ${gazebo_dev_LIBRARIES}
      romea_core_common::romea_core_common
      romea_core_gps::romea_core_gps
      ${sdformat9_LIBRARIES})

    target_include_directories(${PLUGIN_LIB_NAME} SYSTEM PUBLIC
      ${gazebo_ros_INCLUDE_DIRS}
      ${gazebo_dev_INCLUDE_DIRS}
      ${sdformat9_INCLUDE_DIRS})

    target_compile_definitions(${PLUGIN_LIB_NAME} PUBLIC
      ${gazebo_ros_DEFINITIONS}
      ${gazebo_dev_DEFINITIONS}
      ${sdformat9_DEFINITIONS})
#    ament_export_libraries(${PLUGIN_LIB_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(romea_core_common romea_core_gps)

#    ament_export_include_directories(include)
    ament_export_dependencies(rclcpp)
    ament_export_dependencies(romea_common)
    ament_export_dependencies(romea_gps)
    ament_export_dependencies(gazebo_dev)
    ament_export_dependencies(gazebo_msgs)
    ament_export_dependencies(gazebo_ros)
    ament_export_dependencies(sensor_msgs)
    ament_export_dependencies(geometry_msgs)
    ament_export_dependencies(nmea_msgs)
    ament_export_dependencies(sdformat9)

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
      list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
      ament_lint_auto_find_test_dependencies()
      add_subdirectory(test)
    endif()

    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

    install(DIRECTORY include/
            DESTINATION include)

    install(TARGETS
      ${PLUGIN_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin)

    ament_package()

endif()
