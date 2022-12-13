#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import romea_gps_description
import yaml

def get_gps_name(gps_meta_description):
  return gps_meta_description["name"]

def has_gps_driver_configuration(gps_meta_description):
    return "driver" in gps_meta_description

def get_gps_driver_pkg(gps_meta_description):
    return gps_meta_description["driver"]["pkg"]

def get_gps_device(gps_meta_description):
    return gps_meta_description["driver"]["device"]

def get_gps_baudrate(gps_meta_description):
    return gps_meta_description["driver"]["baudrate"]

def has_gps_ntrip_configuration(gps_meta_description):
    return "ntrip" in gps_meta_description

def get_gps_ntrip_pkg(gps_meta_description):
    return gps_meta_description["ntrip"]["pkg"]

def get_gps_ntrip_host(gps_meta_description):
    return gps_meta_description["ntrip"]["host"]

def get_gps_ntrip_port(gps_meta_description):
    return gps_meta_description["ntrip"]["port"]

def get_gps_ntrip_mountpoint(gps_meta_description):
    return gps_meta_description["ntrip"]["mountpoint"]

def get_gps_ntrip_username(gps_meta_description):
    return gps_meta_description["ntrip"].get("username","")

def get_gps_ntrip_password(gps_meta_description):
    return gps_meta_description["ntrip"].get("mountpoint","")

def get_gps_type(gps_meta_description):
  return gps_meta_description["configuration"]["type"]

def get_gps_model(gps_meta_description):
  return gps_meta_description["configuration"]["model"]

def get_gps_rate(gps_meta_description):
  return gps_meta_description["configuration"]["rate"]

def get_gps_parent_link(gps_meta_description):
  return gps_meta_description["geometry"]["parent_link"]

def get_gps_xyz(gps_meta_description):
  return gps_meta_description["geometry"]["xyz"]

def urdf_description(prefix,meta_description_filename):

   with open(meta_description_filename) as f:
     meta_description = yaml.safe_load(f)

   return romea_gps_description.urdf(
       prefix,
       get_gps_name(meta_description),
       get_gps_type(meta_description),
       get_gps_model(meta_description),
       get_gps_rate(meta_description), 
       get_gps_parent_link(meta_description), 
       get_gps_xyz(meta_description), 
   )
