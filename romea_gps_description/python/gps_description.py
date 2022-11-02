#!/usr/bin/env python3

import xacro

from ament_index_python.packages import get_package_share_directory

def urdf(prefix,name,rate, parent_link,xyz):

    xacro_file =  get_package_share_directory("romea_gps_description")+ "/urdf/gps.xacro.urdf"

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "name": name,
            "rate": str(rate),
            "parent_link": parent_link,
            "xyz": ' '.join(map(str, xyz)),
        },
    )

    return urdf_xml.toprettyxml()
