# ROMEA GPS Bringup #

# 1) Overview #

The romea_gps_bringup package provides  : 

 - **Launch files** for launching ROS 2 GPS receiver drivers according to a user-provided meta-description file (see Section 2 for details). Supported drivers are :

   - [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver)
   - [romea_ublox_driver](https://gitlab.irstea.fr/romea_ros2/interfaces/sensors/romea_ublox)

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_gps_bringup gps_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - A **Python Module** able to load and parse GPS meta-description file as well as to create URDF description of the GPS Receiver according a given meta-description.

 - A **ROS2 python executable** able to create GPS URDF description via command line:

  ```console
  ros2 run romea_gps_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > gps.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF can be combined with other URDFs (e.g., for the mobile base and other sensors) to create a complete robot description.  

   



# 2) GPS meta-description #

The GPS meta-description file is a YAML file with six main items:
- **name**: Name of the GPS sensor (defined by the user).
- **driver**: Configuration for the ROS 2 driver controlling the GPS receiver (see Section 5).
- **ntrip**: Configuration for the ROS 2 NTRIP driver, if needed, to broadcast differential corrections (see Section 4).
- **configuration**: Basic specifications of the GPS receiver.
- **geometry**: Location of the GPS receiver antenna on the robot for URDF generation.
- **records**: Topics to be recorded during experiments or simulation. Remappings ensure the GPS topics have consistent names across drivers and simulation.

Example :
```yaml
  name: gps  # name of the gps given by user
  driver: # gps driver configuration
    package: romea_gps_driver  # ros2 driver package choiced by user and its parameters 
    executable: serial_node
    parameters:
      device:  /dev/ttyACM0
      baudrate: 115200
  ntrip:  # ntrip driver configuration (optional)
    package: "ntrip_client"  # ros2 driver package choiced by user and its parameters
    executable: ntrip_ros.py
    parameters: 
      host: caster.centipede.fr
      port: 2101
      username: centipede # optional
      password: centipede # optional
      mountpoint: MAGC
 configuration: # GPS basic specifications
    type: drotek  #  type of GPS receiver
    model: f9p  # model of GPS receiver
    rate: 10 # frame rate in hz
geometry: # geometry configuration 
  parent_link: "base_link"  # name of parent link where is located the GPS antenna
  xyz: [0.0, 0.0, 1.5]  #and it position in meters
records: # topic to be recorded
  nmea: true # nmea sentences will be recorded into bag
  gps_fix: false # gps_fix topic will not be recorded into bag
  vel: false # vel topic will not be recorded into bag
```

# 4) Supported GPS receiver models

The following GPS receivers are supported:

|  type  |   model    |
| :----: | :--------: |
| drotek |    f9p     |
| astech | proflex800 |
| ublox  |   evk_m8   |
| septentrio  |   AsteriX   |

You can find specifications of each receiver in config directory of romea_gps_description package.

# 5) Supported GPS receiver ROS2 drivers

Supported drivers are [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) and  [romea_gps_driver](https://gitlab.irstea.fr/romea_ros2/interfaces/sensors/romea_gps). In order to used one of them, you can specify driver item in GPS meta-description file like this:

- **Nmea Navsat driver**:

```yaml
  package: nmea_navsat_driver  # ROS2 package name
  executable:  nmea_topic_driver
  parameters: # node parameters
    device:  /dev/ttyUSB0  # serial device
    baudrate: 115200 # serial baudrate
```

- **Romea gps driver using serial connection**:

```yaml
  package: "romea_gps_driver"  # ROS2  package name
  executable: serial_node
  parameters: # node parameters
    device:  "/dev/ttyACM0"  # serial device
    baudrate: 115200 # serial baudrate
```

- **Romea gps driver using tcp connection**:

```yaml
  package: romea_gps_driver  # ROS2  package name
  executable: tcp_client_node
  parameters: # node parameters
    ip: 192.168.0.50
    nmea_port: 1001
    rtcm_port: 1002
```


For each driver, a Python launch file with the name of the ROS2 package is provided in launch directory. When the meta-description is read by gps_driver.launch.py, the correct driver node is launched with the parameters set by the user. Thanks to remapping defined inside each driver launch files, the data provided by drivers are always published in the same topics called:

- nmea(nmea_msgs/sentence)
- gps_fix(sensor_msgs/NavSatFix)
- vel(geometry_msgd/Twist)  

# 4) Supported NTRIP client ROS2 drivers

Currently, the only supported NTRIP client is [ntrip_client](https://github.com/LORD-MicroStrain/ntrip_client). To configure it, specify the NTRIP section as follows::  

```yaml
  package: "ntrip_client"  # ros2 driver package choiced by user and its parameters
  executable: ntrip_ros.py
  parameters: 
    host: caster.centipede.fr : 
    port: 2101
    username: centipede # optional
    password: centipede # optional
    mountpoint: MAGC : 
```
