# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from romea_common_bringup import MetaDescription
import romea_gps_description


class GPSMetaDescription:
    def __init__(self, meta_description_filename):
        self.meta_description = MetaDescription("gps", meta_description_filename)

    def get_name(self):
        return self.meta_description.get("name")

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_device(self):
        return self.meta_description.get("device", "driver")

    def get_driver_baudrate(self):
        return self.meta_description.get("baudrate", "driver")

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

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")


def urdf_description(prefix, meta_description_filename):

    meta_description = GPSMetaDescription(meta_description_filename)

    return romea_gps_description.urdf(
        prefix,
        meta_description.get_name(),
        meta_description.get_type(),
        meta_description.get_model(),
        meta_description.get_rate(),
        meta_description.get_parent_link(),
        meta_description.get_xyz(),
    )
