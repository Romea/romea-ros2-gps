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

import os
import pytest

from romea_gps_meta_bringup import (
    GPSMetaDescription,
    get_receiver_specifications,
    get_antenna_geometry,
    get_complete_receiver_configuration
)


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_gps_meta_bringup.yaml")
    return GPSMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "gps"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_package(meta_description):
    assert meta_description.get_driver_package() == "romea_gps_driver"


def test_get_driver_executable(meta_description):
    assert meta_description.get_driver_executable() == "serial_node"


def test_get_driver_parameters(meta_description):
    parameters = meta_description.get_driver_parameters()
    assert parameters["device"] == "/dev/ttyACM0"
    assert parameters["baudrate"] == 115200


def test_has_ntrip_configuration(meta_description):
    assert meta_description.has_ntrip_configuration() is True


def test_get_ntrip_package(meta_description):
    assert meta_description.get_ntrip_package() == "ntrip_client"


def test_get_ntrip_executable(meta_description):
    assert meta_description.get_ntrip_executable() == "ntrip_ros.py"


def test_get_ntrip_parameter(meta_description):
    parameters = meta_description.get_ntrip_parameters()
    assert parameters["host"] == "caster.centipede.fr"
    assert parameters["port"] == 2101
    assert parameters["username"] == "centipede"
    assert parameters["password"] == "centipede"
    assert parameters["mountpoint"] == "MAGC"


def test_get_manufacturer(meta_description):
    assert meta_description.get_manufacturer() == "septentrio"


def test_get_model(meta_description):
    assert meta_description.get_model() == "asterx"


def test_get_rate(meta_description):
    assert meta_description.get_rate() == 10


def test_get_dual_antenna(meta_description):
    assert meta_description.get_dual_antenna() is True


def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]


def test_get_records(meta_description):
    records = meta_description.get_records()
    assert records["nmea_sentence"] is True
    assert records["gps_fix"] is False
    assert records["vel"] is False


def test_get_receiver_specifications(meta_description):
    gps_specifactions = get_receiver_specifications(meta_description)
    assert gps_specifactions['antenna_model'] == "septentrio_polant"


def test_get_antenna_geometry(meta_description):
    gps_geometry = get_antenna_geometry(meta_description)
    assert gps_geometry['mass'] == 0.447


def test_get_complete_receiver_configuration(meta_description):
    gps_configuration = get_complete_receiver_configuration(meta_description)
    assert gps_configuration['antenna_model'] == "septentrio_polant"
