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
import pytest

from romea_gps_bringup import GPSMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_gps_bringup.yaml")
    return GPSMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "gps"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg() == "romea_ublox_driver"


def test_get_driver_device(meta_description):
    assert meta_description.get_driver_device() == "/dev/ttyACM0"


def test_get_driver_baudrate(meta_description):
    assert meta_description.get_driver_baudrate() == 115200


def test_has_ntrip_configuration(meta_description):
    assert meta_description.has_ntrip_configuration() is True


def test_get_ntrip_pkg(meta_description):
    assert meta_description.get_ntrip_pkg() == "ntrip_client"


def test_get_ntrip_host(meta_description):
    assert meta_description.get_ntrip_host() == "caster.centipede.fr"


def test_get_ntrip_port(meta_description):
    assert meta_description.get_ntrip_port() == 2101


def test_get_ntrip_mountpoint(meta_description):
    assert meta_description.get_ntrip_mountpoint() == "MAGC"


def test_get_ntrip_username(meta_description):
    assert meta_description.get_ntrip_username() == "centipede"


def test_get_ntrip_password(meta_description):
    assert meta_description.get_ntrip_password() == "centipede"


def test_get_type(meta_description):
    assert meta_description.get_type() == "drotek"


def test_get_model(meta_description):
    assert meta_description.get_model() == "f9p"


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
