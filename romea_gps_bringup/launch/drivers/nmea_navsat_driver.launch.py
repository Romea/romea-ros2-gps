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

    port = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)

    drivers = LaunchDescription()

    nmea_driver_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_topic_serial_reader",
        output="screen",
        name="nmea_driver",
        parameters=[{"port": port}, {"baud": int(baudrate)}, {"frame_id": frame_id}],
        remappings=[("nmea_sentence", "nmea")],
    )

    drivers.add_action(nmea_driver_node)

    topic_driver_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_topic_driver",
        name="topic_driver",
        output="screen",
        remappings=[("nmea_sentence", "nmea")],
    )

    drivers.add_action(topic_driver_node)

    return [drivers]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("device"))
    declared_arguments.append(DeclareLaunchArgument("baudrate"))
    declared_arguments.append(DeclareLaunchArgument("frame_id"))
    declared_arguments.append(DeclareLaunchArgument("rate"))  # just to be compatible

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
