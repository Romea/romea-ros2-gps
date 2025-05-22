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

import yaml
import romea_gps_description
from romea_common_meta_bringup import SensorMetaDescription, LaunchFileGenerator


class GPSMetaDescription(SensorMetaDescription):
    def __init__(self, meta_description_file_path, robot_name=None):
        super().__init__("gps", meta_description_file_path, robot_name)

    def get_rate(self):
        return self._get("rate", "configuration")

    def get_dual_antenna(self):
        return self._get_or("dual_antenna", "configuration", False)


def load_meta_description(meta_description_file_path):
    return GPSMetaDescription(meta_description_file_path)


def get_receiver_specifications(meta_description):
    return romea_gps_description.get_gps_receiver_specifications(
        meta_description.get_manufacturer(), meta_description.get_model()
    )


def get_antenna_geometry(meta_description):
    gps_configuration = get_complete_receiver_configuration(meta_description)
    antenna_configuration = gps_configuration["antenna_model"].split("_", 1)
    return romea_gps_description.get_gps_antenna_geometry(
        antenna_configuration[0], antenna_configuration[1]
    )


def get_complete_receiver_configuration(meta_description):
    return romea_gps_description.get_gps_complete_receiver_configuration(
        meta_description.get_name(), meta_description.get_configuration()
    )


def generate_configuration_file(meta_description_file_path):
    meta_description = GPSMetaDescription(meta_description_file_path)
    return yaml.dump(get_complete_receiver_configuration(meta_description))


def generate_launch_file(robot_namespace, mode, meta_description_file_path):

    meta_description = GPSMetaDescription(meta_description_file_path, robot_namespace)
    gps_configuration = get_complete_receiver_configuration(meta_description)
    gps_configuration["frame_id"] = meta_description.get_link()

    # gps_full_namespace = meta_description.get_full_namespace()
    return LaunchFileGenerator("gps").generate(
        mode,
        meta_description.get_launch_file(),
        gps_configuration,
        robot_namespace,
        meta_description.get_name(),
    )


def generate_urdf_description(robot_namespace, mode, meta_description_file_path):

    meta_description = GPSMetaDescription(meta_description_file_path, robot_namespace)

    return romea_gps_description.urdf(
        meta_description.get_urdf_prefix(),
        mode,
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_location(),
        meta_description.get_full_namespace(),
    )
