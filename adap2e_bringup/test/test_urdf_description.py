# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import subprocess

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import xml.etree.ElementTree as ET


def urdf_xml(mode, model):

    exe = (
        get_package_prefix("adap2e_bringup") + "/lib/adap2e_bringup/urdf_description.py"
    )

    return ET.fromstring(
        subprocess.check_output(
            [exe, "mode:" + mode, "robot_model:" + model, "robot_namespace:robot"],
            encoding="utf-8",
        )
    )


def test_footprint_link_name():
    assert urdf_xml("live", "slim").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    urdf_xml("live", "fat").find(
        "ros2_control/hardware/plugin"
    ).text == "adap2e_hardware/Adap2eHardware"

    urdf_xml("simulation", "slim").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface4WS4WD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation", "slim").find("gazebo/plugin/parameters").text
        == get_package_share_directory("adap2e_bringup")
        + "/config/controller_manager.yaml"
    )


# def test_gps_name(urdf):
#     assert urdf.find("link").get("name") == "robot_gps_link"


# def test_gps_position(urdf):
#     assert urdf.find("joint/origin").get("xyz") == "1.0 2.0 3.0"


# def test_gps_parent_link(urdf):
#     assert urdf.find("joint/parent").get("link") == "robot_base_link"


# def test_gps_rate(urdf):
#     assert urdf.find("gazebo/sensor/update_rate").text == "10"
