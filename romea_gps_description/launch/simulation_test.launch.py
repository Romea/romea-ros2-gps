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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xml.etree.ElementTree as ET
from romea_gps_description import urdf


def launch_setup(context, *args, **kwargs):

    mode = f'simulation_{LaunchConfiguration("simulator").perform(context)}'

    mode = "simulation"
    prefix = "robot_"
    name = "gps"

    description = {
        "manufacturer": LaunchConfiguration("gps_manufacturer").perform(context),
        "model": LaunchConfiguration("gps_model").perform(context),
        "rate": int(LaunchConfiguration("gps_rate").perform(context)),
    }

    location = {
        "parent_link": "base_link",
        "xyz": [0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 1.0]
    }

    ros_namespace = "robot/gps"

    urdf_xml = ET.fromstring(urdf(prefix, mode, name, description, location, ros_namespace))
    child = ET.SubElement(urdf_xml, "link")
    child.set("name", "robot_base_link")

    with open('/tmp/urdf', 'w') as file:
        file.write(ET.tostring(urdf_xml, encoding='unicode'))

    simulation = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("gazebo_ros")
            + "/launch/gazebo.launch.py"
        ),
    )

    simulation.add_action(gazebo)

    spawn_gps = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_imu",
        output="screen",
        arguments=["-file", "/tmp/urdf", "-entity", "imu"],
    )

    simulation.add_action(spawn_gps)

    return [simulation]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("simulator", default_value="gazebo_classic"),
        DeclareLaunchArgument("gps_manufacturer", default_value=""),
        DeclareLaunchArgument("gps_model", default_value=""),
        DeclareLaunchArgument("gps_rate", default_value="100"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
