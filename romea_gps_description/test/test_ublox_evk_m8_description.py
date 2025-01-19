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


def test_get_gps_specifications_file_path_ok():
    assert (
        get_gps_receiver_specifications_file_path("ublox", "evk_m8")
        == get_package_share_directory("romea_gps_description")
        + "/config/receiver/ublox_evk_m8_specifications.yaml"
    )


def test_get_gps_receiver_specifications_ok():
    assert get_gps_receiver_specifications("ublox", "evk_m8")['antenna_model'] == "ublox_ann_mb5"


def test_get_gps_antenna_geometry_file_path_ok():
    assert (
        get_gps_antenna_geometry_file_path("ublox", "ann_mb5")
        == get_package_share_directory("romea_gps_description")
        + "/config/antenna/ublox_ann_mb5_geometry.yaml"
    )


def test_get_gps_antenna_geometry_ok():
    assert get_gps_antenna_geometry("ublox", "ann_mb5")['mass'] == 0.173


def test_get_gps_receiver_complete_configuration_ok():
    user_description = {
       "type": "ublox",
       "model": "evk_m8",
       "rate": 10
    }

    configuration = get_gps_complete_receiver_configuration("gps", user_description)

    assert configuration["rtk_fix_uere"] == 0.1
    assert configuration["dual_antenna"] is False
    assert configuration["antenna_model"] == "ublox_ann_mb5"
    assert configuration["rate"] == 10
