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

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_common_bringup import device_link_name
from romea_gps_bringup import GPSMetaDescription

import tempfile
import yaml
import os


def get_mode(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "simulation":
        return "simulation_gazebo_classic"
    else:
        return mode


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description(context):

    meta_description_file_path = LaunchConfiguration("meta_description_file_path").perform(context)

    return GPSMetaDescription(meta_description_file_path)


def generate_yaml_temp_file(prefix: str, data: dict):
    fd, filepath = tempfile.mkstemp(prefix=prefix + '_', suffix='.yaml')
    with os.fdopen(fd, 'w') as file:
        file.write(yaml.safe_dump(data))

    return filepath


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    gps_name = meta_description.get_name()
    gps_namespace = str(meta_description.get_namespace() or "")

    actions = [
        PushRosNamespace(robot_namespace),
        PushRosNamespace(gps_namespace),
        PushRosNamespace(gps_name),
    ]

    if mode == "live" and meta_description.has_driver_configuration():
        driver_parameters = meta_description.get_driver_parameters()
        driver_config_path = generate_yaml_temp_file('gps_driver', driver_parameters)

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("romea_gps_bringup"),
                        "launch",
                        "drivers/" + meta_description.get_driver_package() + ".launch.py",
                    ])
                ]),
                launch_arguments={
                    "configuration_file_path": driver_config_path,
                    "executable": meta_description.get_driver_executable(),
                    "rate": str(meta_description.get_rate()),
                    "frame_id": device_link_name(robot_namespace, gps_name),
                }.items(),
            )
        )

    if mode == "live" and meta_description.has_ntrip_configuration():
        ntrip_parameters = meta_description.get_ntrip_parameters()
        ntrip_config_file_path = generate_yaml_temp_file('ntrip_client', ntrip_parameters)

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("romea_gps_bringup"),
                        "launch",
                        "drivers/" + meta_description.get_ntrip_package() + ".launch.py",
                    ])
                ]),
                launch_arguments={
                    "configuration_file_path": ntrip_config_file_path,
                    "executable": meta_description.get_ntrip_executable(),
                }.items(),
            )
        )

    if mode == "simulation_gazebo":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("romea_gps_bringup"),
                        "launch",
                        "drivers/gazebo_bridge.launch.py",
                    ])
                ]),
            )
        )

    # add launch viewer

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace", default_value=""))

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="live"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
