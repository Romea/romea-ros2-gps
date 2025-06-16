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
    restamping = LaunchConfiguration("restamping").perform(context)
    minimal_fix_quality = LaunchConfiguration("minimal_fix_quality").perform(context)
    minimal_speed_over_ground = LaunchConfiguration("minimal_speed_over_ground").perform(context)

    rate = LaunchConfiguration("rate").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context)
    dual_antenna = LaunchConfiguration("antenna").perform(context)
    gps_fix_uere = LaunchConfiguration("gps_fix_uere").perform(context)
    dgps_fix_uere = LaunchConfiguration("dgps_fix_uere").perform(context)
    float_rtk_fix_uere = LaunchConfiguration("float_rtk_fix_uere").perform(context)
    rtk_fix_uere = LaunchConfiguration("rtk_fix_uere").perform(context)
    simulation_fix_uere = LaunchConfiguration("simulation_fix_uere").perform(context)

    latitude_reference = LaunchConfiguration("wgs84_anchor.latitude").perform(context)
    longitude_reference = LaunchConfiguration("wgs84_anchor.longitude").perform(context)
    altitude_reference = LaunchConfiguration("wgs84_anchor.altitude").perform(context)

    common_arguments = {
        "package": "romea_gps_driver",
        "name": "localisation_plugin",
        "parameters": [
            {
                "restamping": bool(restamping),
                "minimal_fix_quality": int(minimal_fix_quality),
                "minimal_speed_over_ground": float(minimal_speed_over_ground),
                "gps":
                    {
                      "rate": int(rate),
                      "dual_antenna": bool(dual_antenna),
                      "gps_fix_uere": float(gps_fix_uere),
                      "dgps_fix_uere": float(dgps_fix_uere),
                      "float_rtk_fix_uere": float(float_rtk_fix_uere),
                      "rtk_fix_uere": float(rtk_fix_uere),
                      "simulation_fix_uere": float(simulation_fix_uere),
                      "xyz": xyz,
                    },
                "wgs84_anchor":
                    {
                        "latitude": float(latitude_reference),
                        "longitude": float(longitude_reference),
                        "altitude": float(altitude_reference)
                    },
            }
        ],
    }

    launch = LaunchDescription()
    if container == "":
        if bool(dual_antenna):
            executable = "dual_antenna_gps_localisation_plugin_node"
        else:
            executable = "single_antenna_gps_localisation_plugin_node"

        launch.add_action(Node(**common_arguments, executable=executable))
    else:

        if bool(dual_antenna):
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("container", default_value=""),
            DeclareLaunchArgument("restamping", default_value="false"),
            DeclareLaunchArgument("minimal_fix_quality", default_value="4"),
            DeclareLaunchArgument("minimal_speed_over_ground", default_value="0.5"),
            OpaqueFunction(function=launch_setup)
        ]
    )
