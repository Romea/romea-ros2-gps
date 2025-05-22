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
import yaml

# from romea_common_meta_bringup import DriverLaunchFileConfiguration
from romea_gps_meta_bringup import GPSMetaDescription, generate_launch_file


def get_meta_description(profile_filename):
    meta_description_file_path = os.path.join(os.getcwd(), profile_filename)
    return GPSMetaDescription(meta_description_file_path, "robot")


def get_nodes_configuration(profile_filename, mode="live"):
    meta_description = get_meta_description(profile_filename)
    launch_file = generate_launch_file(mode, meta_description)
    return yaml.safe_load(launch_file)["launch"][-1]["group"][2:]


def test_create_nmea_navsat_driver_launch_file():

    nodes = get_nodes_configuration("test_nmea_navsat_driver.yaml")

    first_node = nodes[0]["node"]
    assert first_node["pkg"] == "nmea_navsat_driver"
    assert first_node["exec"] == "nmea_topic_serial_reader"
    assert first_node.get("namespace") is None
    assert "plugin" not in first_node
    assert first_node["param"][0]["name"] == "frame_id"
    assert first_node["param"][0]["value"] == "robot_gps_link"
    assert first_node["param"][1]["name"] == "port"
    assert first_node["param"][1]["value"] == "/dev/ttyACM0"
    assert first_node["param"][2]["name"] == "baud"
    assert first_node["param"][2]["value"] == 115200
    assert first_node["remap"][0]["from"] == "nmea_sentence"
    assert first_node["remap"][0]["to"] == "nmea"

    second_node = nodes[1]["node"]
    assert second_node["pkg"] == "nmea_navsat_driver"
    assert second_node["exec"] == "nmea_topic_driver"
    assert second_node.get("namespace") is None
    assert "plugin" not in second_node
    assert second_node["remap"][0]["from"] == "nmea_sentence"
    assert second_node["remap"][0]["to"] == "nmea"


def test_create_romea_gps_serial_driver_launch_file():

    load_composable_node = get_nodes_configuration("test_romea_gps_serial_driver.yaml")[0][
        "load_composable_node"
    ]
    assert load_composable_node["target"] == "sensor_container"

    plugin = load_composable_node["composable_node"][0]
    assert plugin["pkg"] == "romea_gps_driver"
    assert "exec" not in plugin
    assert plugin["plugin"] == "romea::ros2::GpsSerialDriver"
    assert plugin.get("namespace") is None
    assert plugin["param"][0]["name"] == "frame_id"
    assert plugin["param"][0]["value"] == "robot_gps_link"
    assert plugin["param"][1]["name"] == "rate"
    assert plugin["param"][1]["value"] == 10
    assert plugin["param"][2]["name"] == "device"
    assert plugin["param"][2]["value"] == "/dev/ttyACM0"
    assert plugin["param"][3]["name"] == "baudrate"
    assert plugin["param"][3]["value"] == 115200


def test_create_romea_gps_tcp_driver_launch_file():

    node = get_nodes_configuration("test_romea_gps_tcp_driver.yaml")[0]["node"]

    assert node["pkg"] == "romea_gps_driver"
    assert node["exec"] == "tcp_client_node"
    assert "plugin" not in node
    assert node.get("namespace") is None
    assert node["param"][0]["name"] == "frame_id"
    assert node["param"][0]["value"] == "robot_gps_link"
    assert node["param"][1]["name"] == "rate"
    assert node["param"][1]["value"] == 10
    assert node["param"][2]["name"] == "ip"
    assert node["param"][2]["value"] == "192.168.0.50"
    assert node["param"][3]["name"] == "nmea_port"
    assert node["param"][3]["value"] == 1001
    assert node["param"][4]["name"] == "rtcm_port"
    assert node["param"][4]["value"] == 1002


def test_create_ntrip_client_profile():

    node = get_nodes_configuration("test_ntrip_client.yaml")[0]["node"]

    assert node["pkg"] == "ntrip_client"
    assert node["exec"] == "ntrip_ros.py"
    assert "plugin" not in node
    assert node.get("namespace") is None
    assert node["param"][0]["name"] == "host"
    assert node["param"][0]["value"] == "caster.centipede.fr"
    assert node["param"][1]["name"] == "port"
    assert node["param"][1]["value"] == 2101
    assert node["param"][2]["name"] == "username"
    assert node["param"][2]["value"] == "centipede"
    assert node["param"][3]["name"] == "password"
    assert node["param"][3]["value"] == "centipede"
    assert node["param"][4]["name"] == "mountpoint"
    assert node["param"][4]["value"] == "MTLDR"
    assert node["param"][5]["name"] == "authenticate"
    assert node["param"][5]["value"] is True
    assert node["remap"][0]["from"] == "nmea"
    assert node["remap"][0]["to"] == "ntrip/nmea"
    assert node["remap"][1]["from"] == "rtcm"
    assert node["remap"][1]["to"] == "ntrip/rtcm"
