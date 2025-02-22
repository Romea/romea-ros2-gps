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
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from romea_common_meta_bringup import generate_device_temporary_configuration_file
from romea_gps_meta_bringup import GPSMetaDescription, get_driver_launch_file_configuration


def get_mode(context):
    mode = LaunchConfiguration("mode").perform(context)
    return "simulation_gazebo_classic" if mode == "simulation" else mode


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description(context):
    meta_description_file_path = LaunchConfiguration("meta_description_file_path").perform(context)
    return GPSMetaDescription(meta_description_file_path, get_robot_namespace(context))


def launch_setup(context, *args, **kwargs):
    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)
    driver_configuration = get_driver_launch_file_configuration(meta_description, mode)

    print("driver_configuration", driver_configuration)

    driver_configuration_file = generate_device_temporary_configuration_file(
        meta_description, driver_configuration, "driver_configuration.yaml"
    )

    print("temporary file ", driver_configuration_file)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("romea_gps_bringup"),
                            "launch",
                            "driver.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "driver_namespace": meta_description.get_name(),
                "driver_configuration_file_path": driver_configuration_file,
            }.items(),
        )
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace", default_value=""))

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="live"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
