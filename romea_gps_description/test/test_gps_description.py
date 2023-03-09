# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


import pytest
import xml.etree.ElementTree as ET
from romea_gps_description import urdf


@pytest.fixture(scope="module")
def urdf_xml():
    prefix = "robot_"
    name = "gps"
    type = "drotek"
    model = "f9p"
    rate = "10"
    parent_link = "base_link"
    xyz = [1.0, 2.0, 3.0]
    ros_namespace = "ns"
    return ET.fromstring(urdf(prefix, name,
                              type, model, rate,
                              parent_link, xyz,
                              ros_namespace))


def test_gps_name(urdf_xml):
    assert urdf_xml.find("link").get("name") == "robot_gps_link"


def test_gps_position(urdf_xml):
    assert urdf_xml.find("joint/origin").get("xyz") == "1.0 2.0 3.0"


def test_gps_parent_link(urdf_xml):
    assert urdf_xml.find("joint/parent").get("link") == "robot_base_link"


def test_gps_rate(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/update_rate").text == "10"


def test_plugin_namespace(urdf_xml):
    assert urdf_xml.find("gazebo/sensor/plugin/ros/namespace").text == "ns"
