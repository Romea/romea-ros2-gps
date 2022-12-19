
import os
import pytest

from romea_gps_bringup import urdf_description

import xml.etree.ElementTree as ET

@pytest.fixture(scope="module")
def urdf():
    meta_description_filename = os.path.join(os.getcwd(),"test_gps_bringup.yaml")
    return ET.fromstring(urdf_description("robot_",meta_description_filename))

def test_gps_name(urdf):
    assert urdf.find("link").get("name")=="robot_gps_link"

def test_gps_position(urdf):
    assert urdf.find("joint/origin").get("xyz")=="1.0 2.0 3.0"

def test_gps_parent_link(urdf):
    assert urdf.find("joint/parent").get("link")=="robot_base_link"

def test_gps_rate(urdf):
    assert urdf.find("gazebo/sensor/update_rate").text == "10"
