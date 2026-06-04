# romea_gps_description

## Overview

`romea_gps_description` provides URDF descriptions and configuration utilities for GPS sensors using a **specification-based approach**.

It allows defining GPS characteristics and generating consistent ROS2 artifacts using the provided Python module, such as:

* configuration files
* URDF description files

This package is designed to be used together with `romea_gps_meta_bringup`.

---

## GPS description concept

A GPS is described using two inputs:

* **GPS description**: defines hardware characteristics (manufacturer, model, version, rate)
* **GPS location**: defines how the GPS is attached to the robot (parent link, pose)

These inputs are combined with GPS specifications to build a complete configuration used to generate ROS2 artifacts.

---

### Example

```yaml
gps_description:
configuration:
  manufacturer: septentrio
  model: asterx
  rate: 10
  dual_antenna: true

gps_location:
  parent_link: base_link
  xyz: [-0.0, 0.0, 2.0] #m
```

---

### Notes

GPS specifications are defined in files located in the `config/` directory and follow the pattern `<manufacturer>_<model>_<version>_specifications.yaml` (e.g. `septentrio_asterx__specifications.yaml`). These specification files provide default values such as fix rate, EUREs, antenna model which can be overridden by user-defined values in `gps_description`. The `xyz` field defines the GPS antenna position relative to its parent link and specifies the translation in meters.

---

## Python API

The package provides utilities to generate configuration, controllers configuration and URDF descriptions from an GPS description.

---

### get_complete_configuration

Builds a complete GPS configuration by combining:

* GPS description
* GPS location
* GPS specifications

---

### generate_configuration_file_str

Generates the GPS configuration file as a YAML string from the configuration returned by get_complete_configuration.

---

### generate_urdf_description_str

Generates the GPS URDF description as a string, ready to be written to a URDF file. The URDF description is generated from the complete configuration returned by `get_complete_configuration` and from an associated geometry configuration file. The generated URDF is used by slidarlators and visualization tools.

---

## Example

```python
from romea_gps_description import (
    get_complete_configuration,
    generate_configuration_file_str,
    generate_urdf_description_str,
)

gps_name = "lidar"

gps_description = {
  "manufacturer": septentrio,
  "model": asterx,
  "rate": 10,
  "dual_antenna": true,
}

gps_location = {
    "parent_link": "base_link",
    "xyz": [0.0, 0.0, 1.5],
    "rpy": [0.0, 0.0, 0.0]
}

prefix = "robot_"
ros_namespace = "/robot/gps"
mode = "live"

configuration = get_complete_configuration(
    gps_name,
    gps_description,
    gps_location,
)

configuration_yaml = generate_configuration_file_str(configuration)

urdf_description = generate_urdf_description_str(
    prefix,
    mode,
    gps_name,
    gps_description,
    gps_location,
    ros_namespace,
)
```

The returned values can be written to files if needed.

---

## Usage

This package is typically used together with:

* `romea_gps_meta_bringup` → generates launch files and handles bringup

---

## Supported GPS

Currently, the package supports the following GPS manufacturers and models:


| Manufacturer | Model | Version |
|:------------:|:-----:|:-------:|
| septentrio   | mosaic |        |
| ublox        | f9p    |        |
| astech       | proflex | 800   |

Support for additional GPS models may be added in future releases.



! ajouter une section qui décrit comment on ajoute un nouveau GPS
