# 1) Overview 

  The romea_gps_gazebo package provides a Gazebo plugin for GNSS (Global Navigation Satellite System) receiver simulation. As gazebo_ros_gps_sensor plugin provided into ROS [gazebo__plugins](https://github.com/ros-simulation/gazebo_ros_pkgs.git), this plugin publish fix and velocity messages  as well as some nmea sentences. For the moment, only GGA and RMC sentences are provided by this plugin but when contellations will be simulated GSV data will be also provided. 

# 2) Published Topics

- fix (sensor_msgs/NavSatFix):

  The simulated GPS position in WGS84 coordinates (latitude, longitude and altitude).

- vel (geometry_msgs/Vector3Stamped):

  The GNSS velocity vector in ENU coordinates.

- nmea(nmea_msgs/Sentence)

  GGA and RMC nmea sentences
