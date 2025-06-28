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

import xacro
import yaml
import romea_common_description
# from romea_common_description import get_specifications_file_path
# from romea_common_description import get_geometry_file_path
# from romea_common_description import generate_configuration_file
# from romea_common_description import DeviceConfiguration as Device
from ament_index_python.packages import get_package_share_directory


def get_receiver_specifications_file_path(gps_receiver_description):
    return romea_common_description.get_specifications_file_path(
        "romea_gps_description", gps_receiver_description, "config/receiver"
    )


def get_receiver_specifications(gps_receiver_description):
    with open(get_receiver_specifications_file_path(gps_receiver_description)) as f:
        return yaml.safe_load(f)


def get_antenna_geometry_file_path(gps_antenna_description):
    return romea_common_description.get_geometry_file_path(
        "romea_gps_description", gps_antenna_description, "config/antenna"
    )


def get_antenna_geometry(gps_antenna_description):
    with open(get_antenna_geometry_file_path(gps_antenna_description)) as f:
        return yaml.safe_load(f)


def get_receiver_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_gps_description')
    return f'{pkg_path}/config/receiver/specifications_units.yaml'


def get_receiver_specification_units():
    with open(get_receiver_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_complete_configuration(gps_name, gps_reveiver_description, gps_antennation_location):

    model = gps_reveiver_description["model"]
    version = gps_reveiver_description["version"]
    manufacturer = gps_reveiver_description["manufacturer"]
    gps_name = f'{manufacturer} {model} {version} gps called {gps_name}'
    specifications = get_receiver_specifications(gps_reveiver_description)
    specifications_units = get_receiver_specification_units()

    gps = romea_common_description.DeviceConfiguration(
        gps_name, specifications, gps_reveiver_description, specifications_units
    )

    gps_configuration = {
        "model": gps_reveiver_description["model"],
        "version": gps_reveiver_description["version"],
        "manufacturer": gps_reveiver_description["manufacturer"],
        "rate": gps.get('rate'),
        "gps_fix_uere": gps.get('gps_fix_uere'),
        "dgps_fix_uere": gps.get('dgps_fix_uere'),
        "float_rtk_fix_uere": gps.get('float_rtk_fix_uere'),
        "rtk_fix_uere": gps.get('rtk_fix_uere'),
        "simulation_fix_uere": gps.get('simulation_fix_uere'),
        "antenna_model": gps.get('antenna_model'),
        "dual_antenna": gps.get('dual_antenna')
    }

    return {**gps_configuration, **gps_antennation_location}


def generate_configuration_file(configuration, extended):
    units = get_receiver_specification_units()
    return romea_common_description.generate_configuration_file(configuration, units, extended)


def generate_urdf_description(
    prefix, mode, gps_name, gps_reveiver_description, gps_antenna_location, ros_namespace
):

    configuration = get_complete_configuration(
        gps_name, gps_reveiver_description, gps_antenna_location
    )

    configuration_yaml_file = f'/tmp/{prefix}{gps_name}_configuration.yaml'
    with open(configuration_yaml_file, 'w') as f:
        f.write(generate_configuration_file(configuration, False))

    antenna_configuration = configuration["antenna_model"].split('_', 2)
    geometry_yaml_file = get_antenna_geometry_file_path(
        {
            "manufacturer": antenna_configuration[0],
            "model": antenna_configuration[1],
            "version": antenna_configuration[2],
        }
    )

    xacro_file = get_package_share_directory("romea_gps_description") + "/urdf/gps.xacro.urdf"

    if mode == 'simulation':
        mode += '_gazebo_classic'

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            'prefix': prefix,
            'mode': mode,
            'name': gps_name,
            'sensor_config_yaml_file': configuration_yaml_file,
            'geometry_config_yaml_file': geometry_yaml_file,
            'mesh_visual': str(True),
            'ros_namespace': ros_namespace,
        },
    )

    return urdf_xml.toprettyxml()
