from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_gps_bringup import (
    get_gps_name,
    has_gps_driver_configuration,
    get_gps_driver_pkg,
    get_gps_device,
    get_gps_baudrate,
    has_gps_ntrip_configuration,
    get_gps_ntrip_pkg,
    get_gps_ntrip_host,
    get_gps_ntrip_port,
    get_gps_ntrip_mountpoint,
    get_gps_ntrip_username,
    get_gps_ntrip_password,
    get_gps_rate,
)

import yaml


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description(context):
    gps_meta_description_filename = LaunchConfiguration(
        "meta_description_filename"
    ).perform(context)
    with open(gps_meta_description_filename) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not has_gps_driver_configuration(meta_description):
        return []

    gps_name = get_gps_name(meta_description)
    if robot_namespace != "":
        frame_id = robot_namespace + "_" + gps_name + "_link"
    else:
        frame_id = gps_name + "_link"

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_gps_bringup"),
                        "launch",
                        "drivers/"
                        + get_gps_driver_pkg(meta_description)
                        + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "device": get_gps_device(meta_description),
            "baudrate": str(get_gps_baudrate(meta_description)),
            "rate": str(get_gps_rate(meta_description)),
            "frame_id": frame_id,
        }.items(),
    )

    actions = [PushRosNamespace(robot_namespace), PushRosNamespace(gps_name), driver]

    if has_gps_ntrip_configuration(meta_description):

        ntrip = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("romea_gps_bringup"),
                            "launch",
                            "drivers/"
                            + get_gps_ntrip_pkg(meta_description)
                            + ".launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "host": get_gps_ntrip_host(meta_description),
                "port": str(get_gps_ntrip_port(meta_description)),
                "mountpoint": get_gps_ntrip_mountpoint(meta_description),
                "username": get_gps_ntrip_username(meta_description),
                "password": get_gps_ntrip_password(meta_description),
            }.items(),
        )

        actions.append(ntrip)

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []
 
    declared_arguments.append(DeclareLaunchArgument("meta_description_filename"))
 
    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
