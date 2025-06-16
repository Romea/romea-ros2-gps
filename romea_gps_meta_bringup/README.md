# romea_meta_gps_meta_bringup #

# 1) Overview #

The romea_gps_meta_bringup package provides  : 

- **A launch file** for launching ROS2 GPS receiver drivers according to a user-provided meta-description file (see Section 2 for details). Supported drivers are :

   - [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver)
   - romea_gps_driver given in this package

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_gps_meta_bringup gps.launch.py mode:=live robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *mode* is the demonstration mode (live or simulation)	

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

- A **Python module** that can load and parse GPS  meta-description files and provides functions to create URDF  descriptions, configuration files, and launch files based on a given  meta-description.

- A **ROS2 python executables** able to create :

  - URDF description :

    ```shell
    ros2 run romea_gps_meta_bringup generate_urdf_description.py mode:live robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > gps.urdf`
    ```

  - Yaml launch file

    ```shell
    ros2 run romea_gps_meta_bringup generate_launch_file.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > gps.launch.yaml`
    ```

  - Configuration file

    ```shell
    ros2 run romea_gps_meta_bringup generate_configuration_file.py extended:true  meta_description_file_path:/path_to_file/meta_description_file.yaml > gps_config.yaml
    ```

  where :

     - *mode* is the demonstration mode (live or simulation)
     - *robot_namespace* is the name of the robot 
     - *meta_description_file_path* is the absolute path of meta-description file    

    

# 2) GPS meta-description #

The GPS meta-description file is a YAML file with five main items:
- **name**: A user-defined name for the GPS receiver.
- **launch**: A minimal yaml launch used to launch GPS driver (see Section 5).
- **configuration**: Basic specifications of the GPS receiver.
- **location**: Describes the location of the GPS receiver antenna on the robot for URDF generation.
- **records**: Topics to be recorded during experiments or simulation

Example :
```yaml
  name: gps  # name of the gps given by user
  launch: # driver launch file
  - include:
      file: "$(find-pkg-share romea_gps_meta_bringup)/profile/romea_gps_serial_driver.launch.py"
      arg:
        - name: device
          value: /dev/ttyUSB0
        - name: baudrate
          value: "115200"
  - include:
      file: "$(find-pkg-share romea_gps_meta_bringup)/profile/ntrip_client.launch.py"
      arg:
        - name: mountpoint
          value: MTLDR
configuration: # GPS basic specifications
    type: drotek  #  type of GPS receiver
    model: f9p  # model of GPS receiver
    rate: 10 # frame rate in hz
location: # geometry configuration 
  parent_link: "base_link"  # name of parent link where is located the GPS antenna
  xyz: [0.0, 0.0, 1.5]  # position of ths GPS antenna according parent_link in meters
records: # topic to be recorded
  nmea: true # nmea sentences will be recorded into bag
  gps_fix: false # gps_fix topic will not be recorded into bag
  vel: false # vel topic will not be recorded into bag
```

For more information on how to write a meta-description, please refer to the [*romea_common_meta_bringup*](https://github.com/Romea/romea-ros2-common.git) documentation.

# 3) Supported GPS receiver models

The following GPS receivers are supported:

|  type  |   model    |
| :----: | :--------: |
| drotek |    f9p     |
| astech | proflex800 |
| ublox  |   evk_m8   |
| septentrio  |   AsterX   |

The specifications for each receiver can be found in the config directory of the *romea_gps_description* package. If you would like to use a new receiver, you will need to add a corresponding file for that sensor in the config directory of the *romea_gps_description* package.

# 4) Supported GPS receiver ROS2 driver

Supported drivers include [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver)  and  romea_gps_driver given in this package. To use one of these drivers, you can add the snippet as shown below into the launch item of the GPS meta-description file:

- **Nmea Navsat driver**:

  ```yaml
    - include:
        file: "$(find-pkg-share romea_gps_meta_bringup)/profile/romea_gps_serial_driver.launch.py"
        arg:
          - name: "device"
            value: "/dev/ttyUSB0"
          - name: "baudrate"
            value: "115200"
  ```
  
- **Romea gps driver using serial connection **:

  ```yaml
    - include:
        file: "$(find-pkg-share romea_gps_meta_bringup)/profile/romea_gps_tcp_driver.launch.py"
        arg:
          - name: "device"
            value: "/dev/ttyUSB0"
          - name: "baudrate"
            value: "115200"
          - name: "container"
            value: "/foo" # default "", if not empty tcp driver plugin is launch in /foo container 
  ```

- **Romea gps driver using tcp connection**:

  ```yaml
    - include:
        file: "$(find-pkg-share romea_gps_meta_bringup)/profile/romea_gps_tcp_driver.launch.py"
        arg:
        - name: ip
          value: 192.168.0.50
        - name: nmea_port
          value: 1001
        - name: rtcm_port
          value: 1002
        - name: container
            value: /foo #  default "", if not empty tcp driver plugin is launch in /foo container 
  ```

You can also launch the NTRIP driver if you require differential correction, as shown below:

```yaml
  - include:
      file: "$(find-pkg-share romea_gps_meta_bringup)/profile/ntrip_client.launch.py"
      arg:
        - name: mountpoint
          value: MTLDR
```

Each driver node has an associated launch file located in the profile directory of this package. If you wish to use a different driver, you  will need to create a new launch file dedicated for that driver.  It is possible to generate the full launch file in

```shell
ros2 run romea_gps_meta_bringup generate_launch_file.py robot_namespace:'robot' meta_description_file_path:/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_gps/romea_gps_meta_bringup/test/test_gps_meta_bringup.yaml > toto.launch.yaml
```

ros2 run romea_gps_meta_bringup generate_launch_file.py mode:live robot_namespace:'robot' meta_description_file_path:/home/jeanlaneurit/dev/romea_ros2/src/interfaces/sensors/romea_gps/romea_gps_meta_bringup/test/test_gps_meta_bringup.yaml > toto.launch.yaml

