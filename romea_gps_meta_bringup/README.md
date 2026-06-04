# romea_gps_meta_bringup

## 1) Overview

romea_gps_meta_bringup provides tools to describe and launch a GPS sensor using a meta-description approach.

It allows defining a GPS sensor in a high-level YAML format and automatically generating consistent ROS2 artifacts such as:

* configuration files -> used as generic ROS2 configuration inputs
* launch files -> used to start GPS drivers, correction clients and bridges for simulation
* URDF description files -> used to load the GPS into simulators

This package is built on top of `romea_common_meta_bringup` and specializes it for GPS sensor integration.

It also provides launch files that allow controlling the GPS both on a real robot and in simulation.

---

## 2) GPS meta-description concept

A `gps meta-description` is a YAML file that defines a GPS sensor and how it should be integrated into a system. It centralizes:

* GPS identification (name, namespace)
* hardware configuration (manufacturer, model, version)
* kinematic attachment (parent link, pose)
* ROS2 launch description
* optional record configuration

---

### Example meta-description

```yaml
name: gps
namespace: ns
launch:
  - group:
      children:
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
      if: $(eval "'$(var mode)' == 'live'")
  - include:
      file: $(find-pkg-share romea_gps_meta_bringup)/profile/gz_bridge.launch.py
      arg:
        - name: container
          value: /gz_container
      if: $(eval "'$(var mode)' == 'simulation_gazebo'")
configuration:
  manufacturer: septentrio
  model: asterx
  rate: 10
  dual_antenna: true
location:
  parent_link: "base_link"
  xyz: [1.0, 2.0, 3.0]
records:
  nmea_sentence: true
  gps_fix: false
  vel: false
```

### Launch files Profiles

The `profile/` directory contains reusable ROS2 launch files dedicated to GPS bringup.

These launch profiles provide predefined setups for common execution contexts, such as:

* starting a real GPS driver, for example `romea_gps_serial_driver.launch.py`
* starting a TCP GPS driver, for example `romea_gps_tcp_driver.launch.py`
* starting a NMEA driver, for example `nmea_navsat_driver.launch.py`
* starting an NTRIP correction client, for example `ntrip_client.launch.py`
* starting a simulation bridge, for example `gz_bridge.launch.py`
* starting a localization plugin that extracts observations from GPS
* reusing standardized bringup configurations across live and simulation modes

Each profile is intended to be included from the launch section of a GPS meta-description. This makes it possible to select the appropriate runtime behavior depending on the selected mode, while keeping the meta-description concise and consistent.

## 3) Scripts

`romea_gps_meta_bringup` provides several scripts to generate ROS2 artifacts (configuration, launch and URDF files) from a GPS meta-description; the usage and resulting outputs are described below.

### Generate configuration file

```bash
ros2 run romea_gps_meta_bringup generate_configuration_file.py \
  meta_description_file_path:path/to/gps_meta_description.yaml \
  extended:false
```

#### Example output

```yaml
model: asterx
version:
manufacturer: septentrio
rate: 10  # unit Hz
gps_fix_uere: 3.0
dgps_fix_uere: 0.5
float_rtk_fix_uere: 0.1
rtk_fix_uere: 0.02
simulation_fix_uere: 0.02
antenna_model: septentrio_polant_
dual_antenna: true
parent_link: base_link
xyz: [1.0, 2.0, 3.0]  # unit m
```

---

### Generate URDF Description

Generates the GPS URDF description from the meta-description.

```bash
ros2 run romea_gps_meta_bringup generate_urdf_description.py \
  robot_namespace:robot \
  meta_description_file_path:path/to/gps_meta_description.yaml \
  mode:simulation_gazebo
```

The generated URDF description defines how the GPS is attached to the robot model. It includes the GPS link, the fixed joint between the parent link and the GPS frame, and, in simulation mode, the Gazebo sensor description.

#### Example output when using gazebo simulation

```xml
<link name="robot_gps_link">
  ...
</link>

<joint name="robot_gps_joint" type="fixed">
  <origin xyz="1.0 2.0 3.0" rpy="0 0 0"/>
  <parent link="robot_base_link"/>
  <child link="robot_gps_link"/>
</joint>

<gazebo reference="robot_gps_link">
  <sensor name="robot_gps" type="navsat">
    <update_rate>10</update_rate>
    ...
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
  </sensor>
</gazebo>
```

This URDF description can then be loaded into the robot description and used by Gazebo to publish simulated GPS measurements.

It can also be directly concatenated with mobile base and other device URDF descriptions to build a complete robot model.

### Generate launch file

Generates a YAML ROS2 launch file from the meta-description.

```bash
ros2 run romea_gps_meta_bringup generate_launch_file.py \
  robot_namespace:robot \
  meta_description_file_path:path/to/gps_meta_description.yaml
```

#### Example output

```yaml
launch:
- arg: {name: mode, default: live}
- group:
  - push-ros-namespace: {namespace: robot}
  - push-ros-namespace: {namespace: ns}
  - push-ros-namespace: {namespace: gps}
  - let: {name: model, value: asterx}
  - let: {name: version, value: ''}
  - let: {name: manufacturer, value: septentrio}
  - let: {name: rate, value: '10'}
  - let: {name: gps_fix_uere, value: '3.0'}
  - let: {name: dgps_fix_uere, value: '0.5'}
  - let: {name: float_rtk_fix_uere, value: '0.1'}
  - let: {name: rtk_fix_uere, value: '0.02'}
  - let: {name: simulation_fix_uere, value: '0.02'}
  - let: {name: antenna_model, value: septentrio_polant_}
  - let: {name: dual_antenna, value: 'true'}
  - let: {name: parent_link, value: base_link}
  - let: {name: xyz, value: '[1.0, 2.0, 3.0]'}
  - let: {name: tf_prefix, value: robot_}
  - let: {name: frame_id, value: robot_gps_link}
  - group:
      children:
      - include:
          file: $(find-pkg-share romea_gps_meta_bringup)/profile/romea_gps_serial_driver.launch.py
          arg: [{name: device, value: /dev/ttyUSB0}, {name: baudrate, value: '115200'}]
      - include:
          file: $(find-pkg-share romea_gps_meta_bringup)/profile/ntrip_client.launch.py
          arg: [{name: mountpoint, value: MTLDR}]
      if: $(eval "'$(var mode)' == 'live'")
  - include:
      file: $(find-pkg-share romea_gps_meta_bringup)/profile/gz_bridge.launch.py
      arg: [{name: container, value: /gz_container}]
      if: $(eval "'$(var mode)' == 'simulation_gazebo'")
```

#### Notes

* the launch file is generated from the `launch` section of the meta-description
* namespaces are automatically constructed (`robot -> ns -> gps`)
* all configuration values are exposed as `let` variables
* the selected profiles are included at the end

This file can be used directly with ROS2 or generated dynamically using `gps.launch.py`.

## 4) Usage

The package provides **two main launch files**:

* `gps.launch.py` -> for dynamic bringup (live or simulation mode)
* `simulation_test.launch.py` -> for full simulation test

---

### Dynamic bringup

When using `gps.launch.py`, the following steps are performed automatically:

```bash
ros2 launch romea_gps_meta_bringup gps.launch.py \
  mode:=live \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/gps_meta_description.yaml
```

* generation of the launch file
* execution of the generated launch file

#### Live or simulation mode

The `mode` parameter controls the behavior:

* `live` -> starts the GPS driver, optional correction client and localization plugin
* `simulation_<simulator>` -> starts simulation bridge

---

### Simulation test

For a complete simulation setup, use:

```bash
ros2 launch romea_gps_meta_bringup simulation_test.launch.py \
  simulator_type:=gazebo \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/gps_meta_description.yaml
```

This launch file:

* starts the simulator
* generates and loads the URDF
* spawns the GPS in simulation
* calls `gps.launch.py` to start the gazebo bridge

---

## 5) Supported GPS receivers

Currently, the following GPS receiver manufacturers and models are supported:

| Manufacturer | Model | Version |
|:------------:|:-----:|:-------:|
| ashtech | proflex | 800 |
| drotek | f9p |  |
| septentrio | asterx |  |
| ublox | evk | m8 |

Details and specifications for each model can be found in the config directory of the `romea_gps_description` package.

## 6) Supported GPS ROS2 Drivers

The package currently supports the following ROS2 GPS drivers and related components:

* `romea_gps_driver`
* `nmea_navsat_driver`
* `ntrip_client`
* `romea_gps_gazebo`
* `romea_localisation_gps_plugin`

Dedicated launch profiles are provided in the `profile/` directory to start each supported driver through a standardized interface.

These profiles can be included directly from the launch section of the GPS meta-description.

#### Example using romea_gps_driver with serial connection

```yaml
launch:
  - include:
      file: "$(find-pkg-share romea_gps_meta_bringup)/profile/romea_gps_serial_driver.launch.py"
      arg:
        - name: device
          value: /dev/ttyUSB0
        - name: baudrate
          value: "115200"
```

#### Example using romea_gps_driver with TCP connection

```yaml
launch:
  - include:
      file: "$(find-pkg-share romea_gps_meta_bringup)/profile/romea_gps_tcp_driver.launch.py"
      arg:
        - name: ip
          value: 192.168.0.50
        - name: nmea_port
          value: "1001"
        - name: rtcm_port
          value: "1002"
```

#### Example using NTRIP corrections

```yaml
launch:
  - include:
      file: "$(find-pkg-share romea_gps_meta_bringup)/profile/ntrip_client.launch.py"
      arg:
        - name: mountpoint
          value: MTLDR
```

Each launch profile is responsible for:

* starting the corresponding driver or processing node
* configuring driver parameters
* applying standardized topic remappings

This abstraction ensures that all supported GPS drivers expose a consistent ROS2 interface independently of their internal implementation.
