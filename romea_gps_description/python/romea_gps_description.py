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

from romea_common_description import get_specifications_file_path
from romea_common_description import get_geometry_file_path
from romea_common_description import generate_configuration_file
from romea_common_description import DeviceConfiguration as Device
from ament_index_python.packages import get_package_share_directory


def get_gps_receiver_specifications_file_path(gps_receiver_description):
    return get_specifications_file_path(
        "romea_gps_description", gps_receiver_description, "config/receiver"
    )


def get_gps_receiver_specifications(gps_receiver_description):
    with open(get_gps_receiver_specifications_file_path(gps_receiver_description)) as f:
        return yaml.safe_load(f)


def get_gps_antenna_geometry_file_path(gps_antenna_description):
    return get_geometry_file_path(
        "romea_gps_description", gps_antenna_description, "config/antenna"
    )


def get_gps_antenna_geometry(gps_antenna_description):
    with open(get_gps_antenna_geometry_file_path(gps_antenna_description)) as f:
        return yaml.safe_load(f)


def get_gps_receiver_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_gps_description')
    return f'{pkg_path}/config/receiver/specifications_units.yaml'


def get_gps_receiver_specification_units():
    with open(get_gps_receiver_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_gps_complete_receiver_configuration(gps_name, gps_description, gps_location):

    model = gps_description["model"]
    version = gps_description["version"]
    manufacturer = gps_description["manufacturer"]
    gps_name = f'{manufacturer} {model} {version} gps called {gps_name}'
    specifications = get_gps_receiver_specifications(gps_description)
    specifications_units = get_gps_receiver_specification_units()

    gps = Device(gps_name, specifications, gps_description, specifications_units)

    gps_configuration = {
        "model": gps_description["model"],
        "version": gps_description["version"],
        "manufacturer": gps_description["manufacturer"],
        "rate": gps.get('rate'),
        "gps_fix_uere": gps.get('gps_fix_uere'),
        "dgps_fix_uere": gps.get('dgps_fix_uere'),
        "float_rtk_fix_uere": gps.get('float_rtk_fix_uere'),
        "rtk_fix_uere": gps.get('rtk_fix_uere'),
        "simulation_fix_uere": gps.get('simulation_fix_uere'),
        "antenna_model": gps.get('antenna_model'),
        "dual_antenna": gps.get('dual_antenna')
    }
    
    return {**gps_configuration, **gps_location}


def generate_gps_configuration_file(configuration, extended):
    units = get_gps_receiver_specification_units()
    return generate_configuration_file(configuration, units, extended)


def urdf(prefix, mode, gps_name, gps_description, gps_location, ros_namespace):

    configuration = get_gps_complete_receiver_configuration(
        gps_name, gps_description, gps_location
    )

    configuration_yaml_file = f'/tmp/{prefix}{gps_name}_configuration.yaml'
    with open(configuration_yaml_file, 'w') as f:
        f.write(generate_gps_configuration_file(configuration, False))

    antenna_configuration = configuration["antenna_model"].split('_', 2)
    geometry_yaml_file = get_gps_antenna_geometry_file_path(
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
