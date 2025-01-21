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

from romea_common_description import DeviceConfiguration as Device
from ament_index_python.packages import get_package_share_directory


def get_gps_receiver_specifications_file_path(type, model):
    pkg_path = get_package_share_directory('romea_gps_description')
    return f'{pkg_path}/config/receiver/{type}_{model}_specifications.yaml'


def get_gps_receiver_specifications(type, model):
    with open(get_gps_receiver_specifications_file_path(type, model)) as f:
        return yaml.safe_load(f)


def get_gps_antenna_geometry_file_path(type, model):
    pkg_path = get_package_share_directory('romea_gps_description')
    return f'{pkg_path}/config/antenna/{type}_{model}_geometry.yaml'


def get_gps_antenna_geometry(type, model):
    with open(get_gps_antenna_geometry_file_path(type, model)) as f:
        return yaml.safe_load(f)


def get_gps_receiver_specification_units_file_path():
    pkg_path = get_package_share_directory('romea_gps_description')
    return f'{pkg_path}/config/receiver/specifications_units.yaml'


def get_gps_receiver_specification_units():
    with open(get_gps_receiver_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_gps_complete_receiver_configuration(gps_name, gps_description):

    type = gps_description["type"]
    model = gps_description["model"]
    gps_name = f'{type} {model} lidar called {gps_name}'
    specifications = get_gps_receiver_specifications(type, model)
    specifications_units = get_gps_receiver_specification_units()

    gps = Device(gps_name, specifications, gps_description, specifications_units)

    configuration = {}
    configuration['rate'] = gps.get('rate')
    configuration['gps_fix_uere'] = gps.get('gps_fix_uere')
    configuration['dgps_fix_uere'] = gps.get('dgps_fix_uere')
    configuration['float_rtk_fix_uere'] = gps.get('float_rtk_fix_uere')
    configuration['rtk_fix_uere'] = gps.get('rtk_fix_uere')
    configuration['antenna_model'] = gps.get('antenna_model')
    configuration['dual_antenna'] = gps.get('dual_antenna')
    return configuration


def urdf(prefix, mode, gps_name, gps_description, gps_location, ros_namespace):

    configuration = get_gps_complete_receiver_configuration(gps_name, gps_description)

    configuration_yaml_file = f'/tmp/{prefix}{gps_name}_urdf_configuration.yaml'

    with open(configuration_yaml_file, 'w') as f:
        yaml.dump({**configuration, **gps_location}, f)

    antenna_configuration = configuration["antenna_model"].split('_', 1)
    geometry_yaml_file = get_gps_antenna_geometry_file_path(
        antenna_configuration[0], antenna_configuration[1]
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
