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

from romea_common_meta_bringup.meta_description import SensorMetaDescription
from romea_common_meta_bringup.ros_launch import LaunchFileGenerator

import romea_gps_description


class GPSMetaDescription(SensorMetaDescription):
    def __init__(self, meta_description_file_path, robot_name=None):
        super().__init__("gps", meta_description_file_path, robot_name)

    def get_rate(self):
        return self._get("rate", "configuration")

    def get_dual_antenna(self):
        return self._get_or("dual_antenna", "configuration", False)


def load_meta_description(meta_description_file_path, robot_name=None):
    return GPSMetaDescription(meta_description_file_path, robot_name)


def get_receiver_specifications(meta_description):
    return romea_gps_description.get_receiver_specifications(
        meta_description.get_configuration())


def get_antenna_geometry(meta_description):
    gps_configuration = get_complete_configuration(meta_description)
    antenna_configuration = gps_configuration["antenna_model"].split("_", 2)
    return romea_gps_description.get_antenna_geometry(
        {
            "model": antenna_configuration[1],
            "version": antenna_configuration[2],
            "manufacturer": antenna_configuration[0],
        }
    )


def get_complete_configuration(meta_description):
    return romea_gps_description.get_complete_configuration(
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_location()
    )


def generate_configuration_file(meta_description, extended):
    configuration = get_complete_configuration(meta_description)
    return romea_gps_description.generate_configuration_file(configuration, extended)


def generate_launch_file(meta_description):

    launch_file = meta_description.get_launch_file()
    launch_arguments = [{"name": "mode", "default": "live"}]
    namespaces = [meta_description.get_robot_name(), meta_description.get_name()]
    configuration = get_complete_configuration(meta_description)
    configuration["tf_prefix"] = meta_description.get_urdf_prefix()
    configuration["frame_id"] = meta_description.get_link()

    return LaunchFileGenerator("gps").generate(
        launch_file, launch_arguments, namespaces, configuration
    )


def generate_urdf_description(mode, meta_description):

    return romea_gps_description.generate_urdf_description(
        meta_description.get_urdf_prefix(),
        mode,
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_location(),
        meta_description.get_full_namespace(),
    )
