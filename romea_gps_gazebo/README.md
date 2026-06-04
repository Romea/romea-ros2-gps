# romea_gps_gazebo
## Overview

romea_gps_gazebo provides Gazebo simulation plugins and ROS2 bridges for GPS/GNSS sensors.

The package enables the simulation of GPS receivers attached to robotic platforms and publishes simulated GPS measurements and NMEA sentences through standardized ROS2 interfaces.

## Package architecture

The package is composed of three main components:

| Component	| Description |
|:---------:|:-----------:|
| NmeaGpsSensor	| Gazebo sensor implementation generating GPS measurements |
| NmeaGpsSystem	| Gazebo system plugin managing the GPS simulation |
| NmeaGpsBridge	| ROS2 bridge exposing GPS data and NMEA messages |

## Gazebo integration

The GPS sensor can be integrated into the robot URDF using Gazebo custom sensor tag.

Example
```xml
    <sensor type="custom" name="gps_sensor" gz:type="gps"
      xmlns:gz="http://gazebosim.org/schema/gz">
      <pose>0 0 0 0 0 0</pose>
      <always_on>1</always_on>
      <update_rate>10</update_rate>

      <dual_antenna>${dual_antenna}</dual_antenna>

      <gz:gps>
        ...
      </gz:gps>

      <plugin filename="romea_gps_gazebo_plugin" name="romea::gz::NmeaGps" />

    </sensor>
  </xacro:if>
```
The generated GPS measurements can then be bridged to ROS2 topics and used by localization algorithms.

## ROS2 interfaces

The package provides ROS2 bridges exposing standardized interfaces for simulated GPS data.

Typical published topics include:

| Topic | Message type |
|:------------:|:-----:|
| fix	| sensor_msgs/msg/NavSatFix |
|nmea_sentence | nmea_msgs/msg/Sentence |
