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

# flake8: noqa Q000

import pytest
import xml.etree.ElementTree as ET
from romea_gps_description import generate_urdf_description as urdf


def urdf_xml(mode):
    prefix = "robot_"
    name = "gps"

    description = {
        "manufacturer": "septentrio",
        "model": "asterx",
        "version": "",
        "dual_antenna": False,
        "antenna_model": "septentrio_polant_",
        "rate": 10,
    }

    location = {
        "parent_link": "base_link",
        "xyz": [1.0, 2.0, 3.0],
    }

    ros_namespace = "ns"


    urdf_txt = urdf(prefix, mode, name, description, location, ros_namespace)

    with open("/tmp/test_gps_urdf_" + mode, "w") as file:
        file.write(urdf_txt)

    return ET.fromstring(urdf_txt)


def test_gps_name():
    assert urdf_xml("simulation").find("link").get("name") == "robot_gps_link"


def test_gps_position():
    assert urdf_xml("simulation").find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_gps_parent_link():
    assert urdf_xml("simulation").find("joint/parent").get("link") == "robot_base_link"


def test_gps_rate():
    assert urdf_xml("simulation_gazebo").find("gazebo/sensor/update_rate").text == "10"

def test_has_dual_antenna_gazebo():
    assert urdf_xml("simulation").find("gazebo/sensor/dual_antenna").text == "False"


def test_has_dual_antenna_gazebo_classic():
    assert urdf_xml("simulation_gazebo_classic").find("gazebo/sensor/plugin/dual_antenna").text == "False"


def test_plugin_namespace_gazebo_classic():
    assert urdf_xml("simulation_gazebo_classic").find("gazebo/sensor/plugin/ros/namespace").text == "ns"
