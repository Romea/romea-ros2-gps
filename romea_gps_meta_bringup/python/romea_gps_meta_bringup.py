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

from romea_common_meta_bringup import (
    MetaDescription,
    robot_urdf_prefix,
    device_namespace,
    device_link_name
)

import romea_gps_description


class GPSMetaDescription:
    def __init__(self, meta_description_file_path):
        self.meta_description = MetaDescription(
            "gps", meta_description_file_path)

    def get_name(self):
        return self.meta_description.get("name")

    def get_namespace(self):
        return self.meta_description.get_or("namespace", None)

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_package(self):
        return self.meta_description.get("package", "driver")

    def get_driver_executable(self):
        return self.meta_description.get("executable", "driver")

    def get_driver_parameters(self):
        return self.meta_description.get("parameters", "driver")

    def has_ntrip_configuration(self):
        return self.meta_description.exists("ntrip")

    def get_ntrip_package(self):
        return self.meta_description.get("package", "ntrip")

    def get_ntrip_executable(self):
        return self.meta_description.get("executable", "ntrip")

    def get_ntrip_parameters(self):
        return self.meta_description.get("parameters", "ntrip")

    def get_ntrip_host(self):
        return self.meta_description.get("host", "ntrip")

    def get_ntrip_port(self):
        return self.meta_description.get("port", "ntrip")

    def get_ntrip_mountpoint(self):
        return self.meta_description.get("mountpoint", "ntrip")

    def get_ntrip_username(self):
        return self.meta_description.get("username", "ntrip")

    def get_ntrip_password(self):
        return self.meta_description.get("password", "ntrip")

    def get_configuration(self):
        return self.meta_description.get("configuration")

    def get_manufacturer(self):
        return self.meta_description.get("manufacturer", "configuration")

    def get_model(self):
        return self.meta_description.get("model", "configuration")

    def get_rate(self):
        return self.meta_description.get("rate", "configuration")

    def get_dual_antenna(self):
        return self.meta_description.get_or("dual_antenna", "configuration", False)

    def get_geometry(self):
        return self.meta_description.get("geometry")

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")

    def get_records(self):
        return self.meta_description.get_or("records", None, {})

    def get_bridge(self):
        return self.meta_description.get_or("bridge", None, {})


def load_meta_description(meta_description_file_path):
    return GPSMetaDescription(meta_description_file_path)


def get_receiver_specifications(meta_description):
    return romea_gps_description.get_gps_receiver_specifications(
        meta_description.get_manufacturer(), meta_description.get_model()
    )


def get_antenna_geometry(meta_description):
    gps_configuration = get_complete_receiver_configuration(meta_description)
    antenna_configuration = gps_configuration["antenna_model"].split('_', 1)
    return romea_gps_description.get_gps_antenna_geometry(
        antenna_configuration[0], antenna_configuration[1]
    )


def get_complete_receiver_configuration(meta_description):
    return romea_gps_description.get_gps_complete_receiver_configuration(
        meta_description.get_name(), meta_description.get_configuration()
    )


def get_complete_driver_parameters(meta_description, robot_namespace):
    gps_configuration = get_complete_receiver_configuration(meta_description)
    frame_id = device_link_name(robot_namespace, meta_description.get_name())

    executable = meta_description.get_driver_executable()
    parameters = meta_description.get_driver_parameters()
    parameters["frame_id"] = frame_id
    if executable == "serial_node" or executable == "tcp_client_node":
        parameters["rate"] = gps_configuration["rate"]
    return parameters


def urdf_description(robot_namespace, mode, meta_description_file_path):

    meta_description = GPSMetaDescription(meta_description_file_path)

    ros_namespace = device_namespace(
        robot_namespace,
        meta_description.get_namespace(),
        meta_description.get_name()
    )

    return romea_gps_description.urdf(
        robot_urdf_prefix(robot_namespace),
        mode,
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_geometry(),
        ros_namespace
    )
