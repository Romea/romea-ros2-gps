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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("driver_namespace", default_value=""),
            DeclareLaunchArgument("driver_configuration_file_path"),
            DeclareLaunchArgument("component_container", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("romea_common_bringup"),
                                "launch",
                                "driver.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "component_container": LaunchConfiguration("component_container"),
                    "driver_namespace": LaunchConfiguration("driver_namespace"),
                    "driver_configuration_file_path": LaunchConfiguration(
                        "driver_configuration_file_path"
                    ),
                }.items(),
            ),
        ]
    )