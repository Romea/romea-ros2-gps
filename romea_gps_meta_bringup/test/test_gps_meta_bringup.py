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
    get_complete_receiver_configuration,
    # get_launch_description_nodes
)


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_gps_meta_bringup.yaml")
    return GPSMetaDescription(meta_description_file_path, "robot")


def test_get_name(meta_description):
    assert meta_description.get_name() == "gps"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


def test_get_launch_file_configuration(meta_description):
    assert "gps_driver" in meta_description.get_launch_file_configuration()
    assert "ntrip_driver" in meta_description.get_launch_file_configuration()


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


# def test_get_launch_description_nodes(meta_description):
#     nodes = get_launch_description_nodes(
#         meta_description, "live", "robot"
#     )
#     assert nodes[0].get["driver]
