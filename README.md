# romea_ros2_gps

## Overview

`romea_ros2_gps` groups the ROS2 packages used to describe, launch, simulate and interface GPS receivers in the ROMEA ecosystem.

This repository-level README gives a map of the stack. Detailed behavior, configuration formats, generated files and launch examples are documented in the README of each package listed below.

## Packages

| Package | Role |
| --- | --- |
| `romea_gps` | Metapackage for the GPS stack. |
| `romea_gps_description` | GPS receiver specifications, antenna geometry files, Python helpers and URDF generation. |
| `romea_gps_meta_bringup` | Main user entry point: GPS meta-description parser, launch generation, URDF generation and reusable launch profiles. |
| `romea_gps_driver` | Serial and TCP NMEA drivers for live GPS receivers. |
| `romea_gps_ntrip` | NTRIP client tools used to send RTCM corrections and receive NMEA streams. |
| `romea_gps_gazebo` | Gazebo GPS simulation plugins and bridges. |
| `romea_gps_gazebo_classic` | Gazebo Classic GPS simulation plugin. |
| `romea_gps_utils` | ROS2 GPS utilities, data conversions, diagnostics and serial / TCP helpers. |

## Usage

This stack is usually consumed from a larger ROMEA workspace or from a demo configuration that already selects the GPS devices to launch.

In most cases, start with `romea_gps_meta_bringup`. It is the user-facing entry point of the stack: from a GPS meta-description, it can generate the detailed GPS configuration, generate the URDF fragment, and launch the selected live drivers, simulation bridge or localisation plugin. The other packages provide the description data, drivers, simulation plugins and utility code used behind this entry point.

Use the specialized package README files when you need to inspect or extend a specific part of the stack:

* `romea_gps_description` to add or inspect receiver and antenna descriptions;
* `romea_gps_meta_bringup` to write GPS meta-descriptions and launch profiles;
* `romea_gps_driver` and `romea_gps_ntrip` for live data sources;
* `romea_gps_gazebo` or `romea_gps_gazebo_classic` for simulation;
* `romea_gps_utils` for lower-level ROS2 helpers.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `romea_ros2_gps` stack was developed by Jean Laneurit in the context of research projects carried out at INRAE.

## Contact

For questions or comments about this stack, contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
