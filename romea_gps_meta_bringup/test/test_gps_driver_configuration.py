# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
# from romea_common_meta_bringup import DriverLaunchFileConfiguration
from romea_gps_meta_bringup import GPSMetaDescription, get_driver_launch_file_configuration


def get_meta_description(profile_filename):
    meta_description_file_path = os.path.join(os.getcwd(), profile_filename)
    return GPSMetaDescription(meta_description_file_path, "robot")


def get_nodes_configuration(profile_filename, mode="live"):
    meta_description = get_meta_description(profile_filename)
    return get_driver_launch_file_configuration(meta_description, mode)


def test_create_nmea_navsat_driver_profile():

    nodes_configuration = get_nodes_configuration("test_nmea_navsat_driver_profile.yaml")

    assert nodes_configuration["nmea_driver"]["package"] == "nmea_navsat_driver"
    assert nodes_configuration["nmea_driver"]["executable"] == "nmea_topic_serial_reader"
    assert nodes_configuration["nmea_driver"].get("namespace") is None
    assert "plugin" not in nodes_configuration["nmea_driver"]
    assert "component_container" not in nodes_configuration["nmea_driver"]
    assert nodes_configuration["nmea_driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert nodes_configuration["nmea_driver"]["parameters"]["port"] == "/dev/ttyACM0"
    assert nodes_configuration["nmea_driver"]["parameters"]["baud"] == 115200
    assert nodes_configuration["nmea_driver"]["remappings"]["nmea_sentence"] == "nmea"

    assert nodes_configuration["topic_driver"]["package"] == "nmea_navsat_driver"
    assert nodes_configuration["topic_driver"]["executable"] == "nmea_topic_driver"
    assert nodes_configuration["topic_driver"].get("namespace") is None
    assert "plugin" not in nodes_configuration["topic_driver"]
    assert "component_container" not in nodes_configuration["topic_driver"]
    assert nodes_configuration["topic_driver"]["remappings"]["nmea_sentence"] == "nmea"


def test_create_romea_gps_serial_driver_profile():

    nodes_configuration = get_nodes_configuration("test_romea_gps_serial_driver_profile.yaml")

    assert nodes_configuration["driver"]["package"] == "romea_gps_driver"
    assert "excutable" not in nodes_configuration["driver"]
    assert nodes_configuration["driver"]["plugin"] == "romea::ros2::GpsSerialDriver"
    assert nodes_configuration["driver"].get("namespace") is None
    assert nodes_configuration["driver"]["component_container"] == "gps_container"
    assert nodes_configuration["driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert nodes_configuration["driver"]["parameters"]["device"] == "/dev/ttyACM0"
    assert nodes_configuration["driver"]["parameters"]["baudrate"] == 115200


def test_create_romea_gps_tcp_driver_profile():

    nodes_configuration = get_nodes_configuration("test_romea_gps_tcp_driver_profile.yaml")

    assert nodes_configuration["driver"]["package"] == "romea_gps_driver"
    assert nodes_configuration["driver"]["executable"] == "tcp_client_node"
    assert "plugin" not in nodes_configuration["driver"]
    assert "component_container" not in nodes_configuration["driver"]
    assert nodes_configuration["driver"].get("namespace") is None
    assert nodes_configuration["driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert nodes_configuration["driver"]["parameters"]["ip"] == "192.168.0.50"
    assert nodes_configuration["driver"]["parameters"]["nmea_port"] == 1001
    assert nodes_configuration["driver"]["parameters"]["rtcm_port"] == 1002


def test_create_ntrip_client_profile():

    nodes_configuration = get_nodes_configuration("test_ntrip_client_profile.yaml")

    assert nodes_configuration["ntrip"]["package"] == "ntrip_client"
    assert nodes_configuration["ntrip"]["executable"] == "ntrip_ros.py"
    assert "plugin" not in nodes_configuration["ntrip"]
    assert "component_container" not in nodes_configuration["ntrip"]
    assert nodes_configuration["ntrip"].get("namespace") is None
    assert nodes_configuration["ntrip"]["parameters"]["host"] == "caster.centipede.fr"
    assert nodes_configuration["ntrip"]["parameters"]["port"] == 2101
    assert nodes_configuration["ntrip"]["parameters"]["username"] == "centipede"
    assert nodes_configuration["ntrip"]["parameters"]["password"] == "centipede"
    assert nodes_configuration["ntrip"]["parameters"]["mountpoint"] == "MTLDR"
    assert nodes_configuration["ntrip"]["parameters"]["authenticate"] is True
    assert nodes_configuration["ntrip"]["remappings"]["nmea"] == "ntrip/nmea"
    assert nodes_configuration["ntrip"]["remappings"]["rtcm"] == "ntrip/rtcm"
