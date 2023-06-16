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


import xacro
import yaml

from ament_index_python.packages import get_package_share_directory


def get_gps_specifications_file_path(type, model):
    return (
        get_package_share_directory("romea_gps_description")
        + "/config/"
        + type
        + "_"
        + model
        + "_specifications.yaml"
    )


def get_gps_specifications(type, model):

    with open(get_gps_specifications_file_path(type, model)) as f:
        return yaml.safe_load(f)


def get_gps_geometry_file_path(type, model):
    return (
        get_package_share_directory("romea_gps_description")
        + "/config/"
        + type
        + "_"
        + model
        + "_geometry.yaml"
    )


def get_gps_geometry(type, model):

    with open(get_gps_geometry_file_path(type, model)) as f:
        return yaml.safe_load(f)


def urdf(prefix, mode, name, type, model, rate, parent_link, xyz, ros_namespace):

    xacro_file = get_package_share_directory("romea_gps_description") + "/urdf/gps.xacro.urdf"

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "name": name,
            "rate": str(rate),
            "parent_link": parent_link,
            "xyz": " ".join(map(str, xyz)),
            "ros_namespace": ros_namespace
        },
    )

    return urdf_xml.toprettyxml()
