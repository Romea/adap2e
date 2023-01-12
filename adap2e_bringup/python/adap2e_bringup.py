# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from ament_index_python.packages import get_package_share_directory
from adap2e_description import urdf


def urdf_description(prefix, mode, model):

    controller_manager_yaml_file = (
        get_package_share_directory("adap2e_bringup")
        + "/config/controller_manager.yaml"
    )

    return urdf(prefix, mode, model, controller_manager_yaml_file)
