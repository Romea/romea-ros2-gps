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


from romea_common_bringup import MetaDescription, robot_urdf_prefix, device_namespace
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

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_executable(self):
        return self.meta_description.get("executable", "driver")

    def get_driver_parameters(self):
        return self.meta_description.get("parameters", "driver")

    def has_ntrip_configuration(self):
        return self.meta_description.exists("ntrip")

    def get_ntrip_pkg(self):
        return self.meta_description.get("pkg", "ntrip")

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

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_model(self):
        return self.meta_description.get("model", "configuration")

    def get_rate(self):
        return self.meta_description.get("rate", "configuration")

    def get_dual_antenna(self):
        return self.meta_description.get_or("dual_antenna", "configuration", False)

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


def get_gps_specifications(meta_description):
    return romea_gps_description.get_gps_specifications(
        meta_description.get_type(), meta_description.get_model()
    )


def get_gps_geometry(meta_description):
    return romea_gps_description.get_gps_geometry(
        meta_description.get_type(), meta_description.get_model()
    )


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
        meta_description.get_type(),
        meta_description.get_model(),
        meta_description.get_rate(),
        meta_description.get_dual_antenna(),
        meta_description.get_parent_link(),
        meta_description.get_xyz(),
        ros_namespace
    )
