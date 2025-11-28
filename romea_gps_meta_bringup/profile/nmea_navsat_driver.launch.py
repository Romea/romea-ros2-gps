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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    device = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    launch = LaunchDescription()

    if mode == "live":
        launch.add_action(
            Node(
                package="nmea_navsat_driver",
                executable="nmea_topic_serial_reader",
                output="screen",
                name="nmea_driver",
                parameters=[
                    {
                        "port": device,
                        "baud": int(baudrate),
                        "frame_id": frame_id
                    }
                ],
                remappings=[("nmea_sentence", "nmea")],
            )
        )

        launch.add_action(
            Node(
                package="nmea_navsat_driver",
                executable="nmea_topic_driver",
                name="topic_driver",
                output="screen",
                remappings=[("nmea_sentence", "nmea")],
            )
        )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("device"),
            DeclareLaunchArgument("baudrate"),
            OpaqueFunction(function=launch_setup)
        ]
    )
