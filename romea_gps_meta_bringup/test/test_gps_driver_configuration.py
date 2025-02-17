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

from romea_gps_meta_bringup import GPSMetaDescription, get_driver_launch_file_configuration


def test_create_nmea_navsat_driver_profile():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_nmea_navsat_driver_profile.yaml"
    )

    meta_description = GPSMetaDescription(meta_description_file_path)
    configuration = get_driver_launch_file_configuration(meta_description, "robot")

    assert configuration["nmea_driver"]["package"] == "nmea_navsat_driver"
    assert configuration["nmea_driver"]["executable"] == "nmea_topic_serial_reader"
    assert configuration["nmea_driver"]["plugin"] == "none"
    assert configuration["nmea_driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert configuration["nmea_driver"]["parameters"]["port"] == "/dev/ttyACM0"
    assert configuration["nmea_driver"]["parameters"]["baud"] == 115200
    assert configuration["nmea_driver"]["remappings"]["nmea_sentence"] == "nmea"

    assert configuration["topic_driver"]["package"] == "nmea_navsat_driver"
    assert configuration["topic_driver"]["executable"] == "nmea_topic_driver"
    assert configuration["topic_driver"]["plugin"] == "none"
    assert configuration["nmea_driver"]["remappings"]["nmea_sentence"] == "nmea"

    assert configuration["ntrip"]["package"] == "ntrip_client"
    assert configuration["ntrip"]["executable"] == "ntrip_ros.py"
    assert configuration["ntrip"]["plugin"] == "none"
    assert configuration["ntrip"]["parameters"]["host"] == "caster.centipede.fr"
    assert configuration["ntrip"]["parameters"]["port"] == 2101
    assert configuration["ntrip"]["parameters"]["username"] == "centipede"
    assert configuration["ntrip"]["parameters"]["password"] == "centipede"
    assert configuration["ntrip"]["parameters"]["mountpoint"] == "MTLDR"
    assert configuration["ntrip"]["parameters"]["authenticate"] is True
    assert configuration["ntrip"]["remappings"]["nmea"] == "ntrip/nmea"
    assert configuration["ntrip"]["remappings"]["rtcm"] == "ntrip/rtcm"


def test_create_romea_gps_serial_driver_profile():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_romea_gps_serial_driver_profile.yaml"
    )

    meta_description = GPSMetaDescription(meta_description_file_path)
    configuration = get_driver_launch_file_configuration(meta_description, "robot")

    assert configuration["driver"]["package"] == "romea_gps_driver"
    assert configuration["driver"]["executable"] == "serial_node"
    assert configuration["driver"]["plugin"] == "romea::ros2::GpsSerialDriver"
    assert configuration["driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert configuration["driver"]["parameters"]["device"] == "/dev/ttyACM0"
    assert configuration["driver"]["parameters"]["baudrate"] == 115200

    assert configuration["ntrip"]["package"] == "ntrip_client"
    assert configuration["ntrip"]["executable"] == "ntrip_ros.py"
    assert configuration["ntrip"]["plugin"] == "none"
    assert configuration["ntrip"]["parameters"]["host"] == "caster.centipede.fr"
    assert configuration["ntrip"]["parameters"]["port"] == 2101
    assert configuration["ntrip"]["parameters"]["username"] == "centipede"
    assert configuration["ntrip"]["parameters"]["password"] == "centipede"
    assert configuration["ntrip"]["parameters"]["mountpoint"] == "MTLDR"
    assert configuration["ntrip"]["parameters"]["authenticate"] is True
    assert configuration["ntrip"]["remappings"]["nmea"] == "ntrip/nmea"
    assert configuration["ntrip"]["remappings"]["rtcm"] == "ntrip/rtcm"


def test_create_romea_gps_tcp_driver_profile():

    meta_description_file_path = os.path.join(
        os.getcwd(), "test_romea_gps_tcp_driver_profile.yaml"
    )

    meta_description = GPSMetaDescription(meta_description_file_path)
    configuration = get_driver_launch_file_configuration(meta_description, "robot")

    assert configuration["driver"]["package"] == "romea_gps_driver"
    assert configuration["driver"]["executable"] == "tcp_client_node"
    assert configuration["driver"]["plugin"] == "romea::ros2::GpsTcpDriver"
    assert configuration["driver"]["parameters"]["frame_id"] == "robot_gps_link"
    assert configuration["driver"]["parameters"]["ip"] == "192.168.0.50"
    assert configuration["driver"]["parameters"]["nmea_port"] == 1001
    assert configuration["driver"]["parameters"]["rtcm_port"] == 1002

    assert configuration["ntrip"]["package"] == "ntrip_client"
    assert configuration["ntrip"]["executable"] == "ntrip_ros.py"
    assert configuration["ntrip"]["plugin"] == "none"
    assert configuration["ntrip"]["parameters"]["host"] == "caster.centipede.fr"
    assert configuration["ntrip"]["parameters"]["port"] == 2101
    assert configuration["ntrip"]["parameters"]["username"] == "centipede"
    assert configuration["ntrip"]["parameters"]["password"] == "centipede"
    assert configuration["ntrip"]["parameters"]["mountpoint"] == "MTLDR"
    assert configuration["ntrip"]["parameters"]["authenticate"] is True
    assert configuration["ntrip"]["remappings"]["nmea"] == "ntrip/nmea"
    assert configuration["ntrip"]["remappings"]["rtcm"] == "ntrip/rtcm"
