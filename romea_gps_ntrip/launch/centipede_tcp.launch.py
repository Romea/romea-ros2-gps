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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context):
    nmea_port = int(LaunchConfiguration('gps_nmea_port').perform(context))
    rtcm_port = int(LaunchConfiguration('gps_rtcm_port').perform(context))

    return [
        Node(
            package='romea_gps_ntrip',
            executable='tcp_nmea_ntrip',
            name='gnss_ntrip',
            exec_name='gnss_ntrip',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'gps_host': LaunchConfiguration('gps_host'),
                'gps_nmea_port': nmea_port,
                'gps_rtcm_port': rtcm_port,
                'type': 'ntrip',
                'host': 'caster.centipede.fr',
                'port': 2101,
                'user': 'centipede',
                'password': 'centipede',
                'mountpoint': LaunchConfiguration('mountpoint'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
            remappings=[
                ('nmea_sentence', 'gps/nmea_sentence'),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mountpoint', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('gps_host', default_value='192.168.0.50'),
        DeclareLaunchArgument('gps_nmea_port', default_value='1001'),
        DeclareLaunchArgument('gps_rtcm_port', default_value='1002'),
        DeclareLaunchArgument('frame_id', default_value='gps_link'),
        OpaqueFunction(function=launch_setup),
    ])
