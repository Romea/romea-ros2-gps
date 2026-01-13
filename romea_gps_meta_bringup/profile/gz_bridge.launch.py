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

# from launch_ros.actions import SetRemap

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


class LaunchVariables:
    def __init__(self, context):
        self.__context = context

    def get(self, variable_name):
        return LaunchConfiguration(variable_name).perform(self.__context)


def launch_setup(context, *args, **kwargs):

    var = LaunchVariables(context)

    mode = var.get("mode")
    container = var.get("container")
    ros_namespace = var.get("ros_namespace")
    # print(context.launch_configurations)

    common_arguments = {
        "package": "romea_gps_gazebo",
        "name": "gz_bridge",
        "parameters": [
            {"ros_topic_name": f"{ros_namespace}/nmea_sentence"},
            {"gz_topic_name": f"{ros_namespace}/nmea_sentence"},
        ],
        # "parameters": [
        #     {"bridge_names": ["imu_bridge"]},
        #     {"bridges.imu_bridge.ros_topic_name": f"{ros_namespace}/data"},
        #     {"bridges.imu_bridge.gz_topic_name": f"{ros_namespace}/data"},
        #     {"bridges.imu_bridge.ros_type_name": "sensor_msgs/msg/Imu"},
        #     {"bridges.imu_bridge.gz_type_name": "gz.msgs.IMU"},
        #     {"bridges.imu_bridge.direction": "GZ_TO_ROS"},
        #     {"bridges.imu_bridge.lazy": True},
        #     {"bridges.imu_bridge.qos_profile": "SENSOR_DATA"},
        # ],
    }

    launch = LaunchDescription()
    if mode == "simulation_gazebo":

        if container == "":
            executable = "nmea_gps_gz_bridge"
            launch.add_action(Node(**common_arguments, executable=executable))
        else:
            plugin = "ros_gz_bridge::RosGzBridge"
            extra_arguments = ([{"use_intra_process_comms": True}],)
            launch.add_action(
                LoadComposableNodes(
                    target_container=container,
                    composable_node_descriptions=[
                        ComposableNode(
                            **common_arguments, plugin=plugin, extra_arguments=extra_arguments
                        )
                    ],
                )
            )

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("container", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
