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
from romea_gps_meta_bringup import GPSMetaDescription, get_complete_driver_parameters


def test_serial_node_driver_parameters():

    meta_description_file_path = os.path.join(os.getcwd(), "test_gps_serial_node_parameters.yaml")
    meta_description = GPSMetaDescription(meta_description_file_path)

    parameters = get_complete_driver_parameters(meta_description, "robot")
    assert parameters["frame_id"] == "robot_gps_link"
    assert parameters["device"] == "/dev/ttyACM0"
    assert parameters["baudrate"] == 115200
    assert parameters["rate"] == 10


def test_tcp_client_node_driver_parameters():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_gps_tcp_client_node_parameters.yaml"
    )
    meta_description = GPSMetaDescription(meta_description_file_path)

    parameters = get_complete_driver_parameters(meta_description, "robot")
    assert parameters["frame_id"] == "robot_gps_link"
    assert parameters["ip"] == "192.168.0.50"
    assert parameters["nmea_port"] == 1001
    assert parameters["rtcm_port"] == 1002
    assert parameters["rate"] == 10


def test_nmea_navsat_driver_parameters():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_gps_nmea_navsat_driver_parameters.yaml"
    )
    meta_description = GPSMetaDescription(meta_description_file_path)

    parameters = get_complete_driver_parameters(meta_description, "robot")
    assert parameters["frame_id"] == "robot_gps_link"
    assert parameters["port"] == "/dev/ttyACM0"
    assert parameters["baud"] == 115200
