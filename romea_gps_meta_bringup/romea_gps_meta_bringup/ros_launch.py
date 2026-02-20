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


from launch.substitutions import LaunchConfiguration

import romea_common_meta_bringup.ros_launch as common
from romea_gps_meta_bringup.meta_description import load_meta_description

import yaml


def get_meta_description(context):
    robot_namespace = common.get_robot_namespace(context)
    meta_description_file_path = common.get_meta_description_file_path(context)
    return load_meta_description(meta_description_file_path, robot_namespace)


def declare_wgs84_anchor_file_path(default_value=None):
    return common.declare_argument(
        {
            "name": "wgs84_anchor_file_path",
            "description": "WGS84 anchor configuration filename",
        },
        default_value
    )


def get_wgs84_anchor_file_path(context):
    return LaunchConfiguration("wgs84_anchor_file_path").perform(context)


def get_wgs84_anchor(context):
    wgs84_anchor_file_path = LaunchConfiguration(
        "wgs84_anchor_file_path"
    ).perform(context)

    if not wgs84_anchor_file_path or wgs84_anchor_file_path == "":
        return None

    with open(wgs84_anchor_file_path) as f:
        return yaml.safe_load(f)
