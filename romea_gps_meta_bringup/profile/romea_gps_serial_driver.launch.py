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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, LoadComposableNodes
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    container = LaunchConfiguration("container").perform(context)
    device = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)

    frame_id = LaunchConfiguration("frame_id").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    executable = "serial_node"
    plugin = "romea::ros2::GpsSerialDriver"

    common_arguments = {
        "package": "romea_gps_driver",
        "name": "driver",
        "parameters": [
            {
                "frame_id": frame_id,
                "rate": int(rate),
                "device": device,
                "baudrate": int(baudrate)
            }
        ],
    }

    launch = LaunchDescription()
    if mode == "live":
        if container == "":
            launch.add_action(Node(**common_arguments, executable=executable))
        else:
            launch.add_action(
                LoadComposableNodes(
                    target_container=container,
                    composable_node_descriptions=[
                        ComposableNode(**common_arguments, plugin=plugin)
                    ],
                )
            )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("device"),
            DeclareLaunchArgument("baudrate"),
            DeclareLaunchArgument("container", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
