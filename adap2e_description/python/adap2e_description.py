#!/usr/bin/env python3

import xacro

from ament_index_python.packages import get_package_share_directory

def urdf(prefix, mode, robot_model, controller_conf_yaml_file):

    xacro_file = (
        get_package_share_directory("adap2e_description")
        + "/urdf/adap2e_"
        + robot_model
        + ".urdf.xacro"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "controller_conf_yaml_file": controller_conf_yaml_file,
        },
    )

    return urdf_xml.toprettyxml()
