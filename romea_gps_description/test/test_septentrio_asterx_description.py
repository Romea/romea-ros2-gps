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

import pytest

from romea_gps_description import (
    get_antenna_geometry,
    get_antenna_geometry_file_path,
    get_complete_configuration,
    get_receiver_specifications,
    get_receiver_specifications_file_path,
)


@pytest.fixture(scope="module")
def user_receiver_description():

    return {
       "manufacturer": "septentrio",
       "model": "asterx",
       "version": "",
       "dual_antenna": True,
       "rate": 10
    }


@pytest.fixture(scope="module")
def user_antenna_description():

    return {
       "manufacturer": "septentrio",
       "model": "polant",
       "version": "",
    }


def test_get_specifications_file_path_ok(user_receiver_description):
    assert (
        get_receiver_specifications_file_path(user_receiver_description)
        == get_package_share_directory("romea_gps_description")
        + "/config/receiver/septentrio_asterx__specifications.yaml"
    )


def test_get_receiver_specifications_ok(user_receiver_description):
    assert (
        get_receiver_specifications(user_receiver_description)['antenna_model']
        == "septentrio_polant_"
    )


def test_get_antenna_geometry_file_path_ok(user_antenna_description):
    assert (
        get_antenna_geometry_file_path(user_antenna_description)
        == get_package_share_directory("romea_gps_description")
        + "/config/antenna/septentrio_polant__geometry.yaml"
    )


def test_get_antenna_geometry_ok(user_antenna_description):
    assert get_antenna_geometry(user_antenna_description)['mass'] == 0.447


def test_get_complete_configuration_ok(user_receiver_description):

    configuration = get_complete_configuration("gps", user_receiver_description, {})

    assert configuration["rtk_fix_uere"] == 0.02
    assert configuration["dual_antenna"] is True
    assert configuration["antenna_model"] == "septentrio_polant_"
    assert configuration["rate"] == 10
