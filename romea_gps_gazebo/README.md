#1 Description #

  GazeboRosGps simulates a GNSS (Global Navigation Satellite System) receiver which is attached to a robot. It publishs fix, velocity and GGA and RMC messages. The reference point that corresponds to the origin of the gazebo frame can be configured using the XML parameters. The conversion between gazebo coordinates and WGS84 is done using a local plane projection, which is accurate enough if you do not go far away from the configured reference point.

#2 Published Topics #

- $(fix_topic_name) (sensor_msgs/NavSatFix):

  The simulated GPS position in WGS84 coordinates (latitude, longitude and altitude).

- $(velocity_topic_name) (geometry_msgs/Vector3Stamped):

  The GNSS velocity vector in NWU (north, west, up) coordinates.

- $(nmea_topic_name) nmea_msgs/Sentence

  GGA and RMC nmea sentences

#3 XML Parameters #

- gps_name(string)

  Name of the sensor for debug purpose

- body_name(string)

  Name of the parent link in URDF description

- link_name(string)

  Name of gps link in URDF description

- update_rate

  Publishing rate of nmea sentences and fix and velocity data

- nmea_topic_name(string)

  Nmea sentence ros topic name

- fix_topic_name(string)

  Rover position ros topic name

- velocity_topic_name(string)

  Rover velocity ros topic name

- antenna_body_position(double[3])

  Position of gps antenna with respect of the robot reference frame (base_link)

- wgs84_anchor(double[3], default=[45.78,3.08,365])

  Geodetic coordinates of local tangent plane

- geoid_heihgt(double, default=0)

  Geoid height at wgs84 anchor position

- z_offset(double)

  Elevation offset between robot base_footprint and robot base_link

- status(int, default=2)

  Status published in navsat fix messages (see sensors_msgs/NavSatStatus), by default fix is considered to be computed in using ground based augmentation

- service(int, default=15)

  Service published in navsat fix (see sensors_msgs/NavSatStatus), by default fix is considered to be computed in using GPS, GLONASS, BEIDOU and GALILEO systems
