cmake_minimum_required(VERSION 3.5)
project(romea_gps_gazebo)

set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STRGREATER "iron")

    # set(PLUGIN_LIB_NAME "romea_gazebo_ros_gps_sensor")

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
    find_package(gz_common_vendor REQUIRED)
    find_package(gz-common REQUIRED)
    find_package(gz_msgs_vendor REQUIRED)
    find_package(gz-msgs REQUIRED)
    find_package(gz_sim_vendor REQUIRED)
    find_package(gz-sim REQUIRED)
    find_package(gz_plugin_vendor REQUIRED)
    find_package(gz-plugin REQUIRED)
    find_package(gz_sensors_vendor REQUIRED)
    find_package(gz-sensors REQUIRED)
    find_package(romea_core_common REQUIRED)
    find_package(romea_core_gps REQUIRED)


    # Define a variable 'MSGS_PROTOS' listing the .proto files
    set(MSGS_PROTOS ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/custom_msgs/nmea_sentence.proto)

    # Call 'gz_msgs_generate_messages()' to process the .proto files
    gz_msgs_generate_messages(
      # The cmake target to be generated for libraries/executables to link
      TARGET msgs
      # The protobuf package to generate (Typically based on the path)
      PROTO_PACKAGE "gz.custom_msgs"
      # The path to the base directory of the proto files
      # All import paths should be relative to this (eg gz/custom_msgs/vector3d.proto)
      MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
      # List of proto files to process
      MSGS_PROTOS ${MSGS_PROTOS}
      # Dependency on gz-msgs
      DEPENDENCIES gz-msgs::gz-msgs
    )

    add_library(${PROJECT_NAME} SHARED src/nmea_gps_sensor.cpp src/nmea_gps_system.cpp)

    target_link_libraries(${PROJECT_NAME}
      PRIVATE
        gz-plugin::register
        gz-sim::gz-sim
        gz-sensors::gz-sensors
        romea_gps_gazebo-msgs
        romea_core_gps::romea_core_gps
    )

    target_include_directories(${PROJECT_NAME} 
      PUBLIC
      include
      ${romea_core_gps_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}/romea_gps_gazebo-msgs_genmsg
    )

    install(DIRECTORY include/
        DESTINATION include)

    install(TARGETS
      ${PROJECT_NAME}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin)


    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

    ament_package()

endif()
