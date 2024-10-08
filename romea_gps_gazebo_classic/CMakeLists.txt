cmake_minimum_required(VERSION 3.5)
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

    link_directories(${gazebo_dev_LIBRARY_DIRS})

    # gazebo_ros_gps_sensor
    add_library(${PLUGIN_LIB_NAME} SHARED
      src/gazebo_ros_gps_sensor.cpp
    )
    target_include_directories(${PLUGIN_LIB_NAME} PUBLIC include)

    ament_target_dependencies(${PLUGIN_LIB_NAME}
      "gazebo_ros"
      "sensor_msgs"
      "geometry_msgs"
      "nmea_msgs"
      "gazebo_dev"
      "romea_core_common"
      "romea_core_gps"
    )
    ament_export_libraries(${PLUGIN_LIB_NAME})

    ament_export_include_directories(include)
    ament_export_dependencies(rclcpp)
    ament_export_dependencies(romea_common)
    ament_export_dependencies(romea_gps)
    ament_export_dependencies(gazebo_dev)
    ament_export_dependencies(gazebo_msgs)
    ament_export_dependencies(gazebo_ros)
    ament_export_dependencies(sensor_msgs)
    ament_export_dependencies(geometry_msgs)
    ament_export_dependencies(nmea_msgs)

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
      ament_lint_auto_find_test_dependencies()
      add_subdirectory(test)
    endif()

    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

    install(DIRECTORY include/
            DESTINATION include)

    install(TARGETS
      ${PLUGIN_LIB_NAME}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin)

    ament_package()

endif()
