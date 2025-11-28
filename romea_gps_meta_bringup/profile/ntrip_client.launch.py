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

    host = LaunchConfiguration("host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    username = LaunchConfiguration("username").perform(context)
    password = LaunchConfiguration("password").perform(context)
    mountpoint = LaunchConfiguration("mountpoint").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    launch = LaunchDescription()

    if mode == "live":

        launch.add_action(
            Node(
                package="ntrip_client",
                executable="ntrip_ros.py",
                output="screen",
                name="ntrip_client",
                exec_name="ntrip_client",
                parameters=[
                    {
                        "host": host,
                        "port": int(port),
                        "username": username,
                        "password": password,
                        "mountpoint": mountpoint,
                        "authenticate": username != "" and password != ""
                    }
                ],
                remappings=[("nmea", "ntrip/nmea"), ("rtcm", "ntrip/rtcm")],
            )
        )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="caster.centipede.fr"),
            DeclareLaunchArgument("port", default_value="2101"),
            DeclareLaunchArgument("username", default_value="centipede"),
            DeclareLaunchArgument("password", default_value="centipede"),
            DeclareLaunchArgument("mountpoint"),
            OpaqueFunction(function=launch_setup)
        ]
    )
