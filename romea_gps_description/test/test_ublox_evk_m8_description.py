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

from ament_index_python.packages import get_package_share_directory

from romea_gps_description import (
    get_gps_complete_receiver_configuration,
    get_gps_antenna_geometry_file_path,
    get_gps_antenna_geometry,
    get_gps_receiver_specifications_file_path,
    get_gps_receiver_specifications,
)

import pytest


@pytest.fixture(scope="module")
def user_receiver_description():

    return {"manufacturer": "ublox", "model": "evk", "version": "m8", "rate": 10}


@pytest.fixture(scope="module")
def user_antenna_description():

    return {
        "manufacturer": "ublox",
        "model": "ann",
        "version": "mb5",
    }


def test_get_gps_specifications_file_path_ok(user_receiver_description):
    assert (
        get_gps_receiver_specifications_file_path(user_receiver_description)
        == get_package_share_directory("romea_gps_description")
        + "/config/receiver/ublox_evk_m8_specifications.yaml"
    )


def test_get_gps_receiver_specifications_ok(user_receiver_description):
    assert (
        get_gps_receiver_specifications(user_receiver_description)["antenna_model"]
        == "ublox_ann_mb5"
    )


def test_get_gps_antenna_geometry_file_path_ok(user_antenna_description):
    assert (
        get_gps_antenna_geometry_file_path(user_antenna_description)
        == get_package_share_directory("romea_gps_description")
        + "/config/antenna/ublox_ann_mb5_geometry.yaml"
    )


def test_get_gps_antenna_geometry_ok(user_antenna_description):
    assert get_gps_antenna_geometry(user_antenna_description)["mass"] == 0.173


def test_get_gps_receiver_complete_configuration_ok(user_receiver_description):

    configuration = get_gps_complete_receiver_configuration("gps", user_receiver_description)

    assert configuration["rtk_fix_uere"] == 0.1
    assert configuration["dual_antenna"] is False
    assert configuration["antenna_model"] == "ublox_ann_mb5"
    assert configuration["rate"] == 10
