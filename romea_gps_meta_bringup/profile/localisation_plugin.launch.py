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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

import yaml


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    container = LaunchConfiguration("container").perform(context)
    restamping = LaunchConfiguration("restamping").perform(context)
    minimal_fix_quality = LaunchConfiguration("minimal_fix_quality").perform(context)
    minimal_speed_over_ground = LaunchConfiguration("minimal_speed_over_ground").perform(context)
    # print(context.launch_configurations)

    with open(LaunchConfiguration("wgs84_anchor_file_path").perform(context)) as f:
        wgs84_anchor = yaml.safe_load(f)

    gps_configuration = {
        "rate": int(LaunchConfiguration("rate").perform(context)),
        "dual_antenna": bool(LaunchConfiguration("dual_antenna").perform(context)),
        "gps_fix_uere": float(LaunchConfiguration("gps_fix_uere").perform(context)),
        "dgps_fix_uere": float(LaunchConfiguration("dgps_fix_uere").perform(context)),
        "float_rtk_fix_uere": float(LaunchConfiguration("float_rtk_fix_uere").perform(context)),
        "rtk_fix_uere": float(LaunchConfiguration("rtk_fix_uere").perform(context)),
        "simulation_fix_uere": float(LaunchConfiguration("simulation_fix_uere").perform(context)),
        "xyz": [float(v) for v in LaunchConfiguration("xyz").perform(context)[1:-1].split(",")],
    }

    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
 
    common_arguments = {
        "package": "romea_localisation_gps_plugin",
        "name": "localisation_plugin",
        "parameters": [
            {
                "restamping": bool(restamping),
                "minimal_fix_quality": int(minimal_fix_quality),
                "minimal_speed_over_ground": float(minimal_speed_over_ground),
                "gps": gps_configuration,
                "wgs84_anchor": wgs84_anchor,
                "use_sim_time": "live" not in mode,
            }
        ],
        "remappings": [
            ("gps/nmea_sentence", "nmea_sentence"),
            ("vehicle_controller/odom", odom_topic),
            ("course", f"/{robot_namespace}/localisation/course"),
            ("position", f"/{robot_namespace}/localisation/position"),
        ]
    }

    launch = LaunchDescription()
    if container == "":
        if bool(gps_configuration["dual_antenna"]):
            executable = "dual_antenna_gps_localisation_plugin_node"
        else:
            executable = "single_antenna_gps_localisation_plugin_node"

        launch.add_action(Node(**common_arguments, executable=executable))
    else:
        if bool(gps_configuration["dual_antenna"]):
            plugin = "romea::ros2::DualAntennaGPSLocalisationPlugin"
        else:
            plugin = "romea::ros2::SingleAntennaGPSLocalisationPlugin"

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

    default_odom_topic = [
        TextSubstitution(text='/'),
        LaunchConfiguration("robot_namespace"),
        TextSubstitution(text='/base/controller/odom')
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("container", default_value=""),
            DeclareLaunchArgument("restamping", default_value="false"),
            DeclareLaunchArgument("minimal_fix_quality", default_value="4"),
            DeclareLaunchArgument("minimal_speed_over_ground", default_value="0.5"),
            DeclareLaunchArgument("odom_topic", default_value=default_odom_topic),
            DeclareLaunchArgument("wgs84_anchor_file_path"),
            OpaqueFunction(function=launch_setup)
        ]
    )
