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



# import pytest
import xml.etree.ElementTree as ET
from adap2e_description import urdf


def urdf_xml(mode, model):
    prefix = "robot_"
    ros_namespace = "/robot"
    controller_conf_yaml_file = mode + "_" + model + "_controller.yaml"
    return ET.fromstring(urdf(prefix, mode, model, controller_conf_yaml_file, ros_namespace))


def test_footprint_link_name():
    assert urdf_xml("live", "slim").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert urdf_xml("live", "fat").find(
        "ros2_control/hardware/plugin"
    ).text == "adap2e_hardware/Adap2eHardware"

    assert urdf_xml("simulation", "slim").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface4WS4WD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation", "slim").find("gazebo/plugin/parameters").text
        == "simulation_slim_controller.yaml"
    )
