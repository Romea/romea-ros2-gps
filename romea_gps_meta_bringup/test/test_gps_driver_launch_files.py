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
from romea_gps_meta_bringup import GPSMetaDescription, generate_launch_file


def check_param(node, name, value):
    param = [d["value"] for d in node["param"] if d["name"] == name]
    assert param, f"param {name} is note defined in node configuration"
    assert (
        param[0] == value
    ), f"value of param {name} should be equal to {value} instead of {param[0]}"


def check_remap(node, from_, to_):
    remap = [d["to"] for d in node["remap"] if d["from"] == from_]
    assert remap, f"remap for {from_} is note defined in node configuration"
    assert (
        remap[0] == to_
    ), f"remap for {from_} should be equal to {to_} instead of {remap[0]}"


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
    check_param(first_node, "frame_id", "robot_gps_link")
    check_param(first_node, "port", "/dev/ttyACM0")
    check_param(first_node, "baud", 115200)
    check_remap(first_node, "nmea_sentence", "nmea")

    second_node = nodes[1]["node"]
    assert second_node["pkg"] == "nmea_navsat_driver"
    assert second_node["exec"] == "nmea_topic_driver"
    assert second_node.get("namespace") is None
    assert "plugin" not in second_node
    check_remap(second_node, "nmea_sentence", "nmea")


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
    check_param(plugin, "frame_id", "robot_gps_link")
    check_param(plugin, "rate", 10)
    check_param(plugin, "device", "/dev/ttyACM0")
    check_param(plugin, "baudrate", 115200)


def test_create_romea_gps_tcp_driver_launch_file():

    node = get_nodes_configuration("test_romea_gps_tcp_driver.yaml")[0]["node"]

    assert node["pkg"] == "romea_gps_driver"
    assert node["exec"] == "tcp_client_node"
    assert "plugin" not in node
    assert node.get("namespace") is None
    check_param(node, "frame_id", "robot_gps_link")
    check_param(node, "rate", 10)
    check_param(node, "ip", "192.168.0.50")
    check_param(node, "nmea_port", 1001)
    check_param(node, "rtcm_port", 1002)


def test_create_ntrip_client_profile():

    node = get_nodes_configuration("test_ntrip_client.yaml")[0]["node"]

    assert node["pkg"] == "ntrip_client"
    assert node["exec"] == "ntrip_ros.py"
    assert "plugin" not in node
    assert node.get("namespace") is None
    check_param(node, "host", "caster.centipede.fr")
    check_param(node, "port", 2101)
    check_param(node, "username", "centipede")
    check_param(node, "password", "centipede")
    check_param(node, "mountpoint", "MTLDR")
    check_param(node, "authenticate", True)
    check_remap(node, "nmea", "ntrip/nmea")
    check_remap(node, "rtcm", "ntrip/rtcm")
