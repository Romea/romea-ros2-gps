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

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    host = LaunchConfiguration("host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    mountpoint = LaunchConfiguration("mountpoint").perform(context)
    username = LaunchConfiguration("username").perform(context)
    password = LaunchConfiguration("password").perform(context)

    driver = LaunchDescription()

    ntrip_client_node = Node(
        package="ntrip_client",
        executable="ntrip_ros.py",
        output="screen",
        name="ntrip_client",
        parameters=[
            {"host": host},
            {"port": int(port)},
            {"mountpoint": mountpoint},
            {"username": username},
            {"password": password},
            {"authenticate": username != "" and password != ""},
        ],
        remappings=[("nmea", "ntrip/nmea"), ("rtcm", "ntrip/rtcm")],
    )

    driver.add_action(ntrip_client_node)

    return [driver]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("host"))
    declared_arguments.append(DeclareLaunchArgument("port"))
    declared_arguments.append(DeclareLaunchArgument("mountpoint"))
    declared_arguments.append(DeclareLaunchArgument("username"))
    declared_arguments.append(DeclareLaunchArgument("password"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
