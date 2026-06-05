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

import xml.etree.ElementTree as ET

from adap2e_description import generate_ros2_control_description, generate_urdf_description


def urdf_xml(mode, model):
    prefix = "robot_"
    ros_prefix = "/robot/"
    base_name = "base"
    controller_conf_yaml_file = mode + "_" + model + "_controller.yaml"
  
    return ET.fromstring(
        generate_urdf_description(
            prefix, mode, base_name, model, controller_conf_yaml_file, ros_prefix
        )
    )


def ros2_control_xml(mode, model):
    prefix = "robot_"
    base_name = "base"
    
    return ET.fromstring(
        generate_ros2_control_description(
            prefix, mode, base_name, model
        )
    )


def test_footprint_link_name():

    assert urdf_xml("live", "one").find("link").get("name") == "robot_base_footprint"


def test_controller_filename_name():

    assert (
        urdf_xml("simulation", "two").find("gazebo/plugin/parameters").text
        == "simulation_two_controller.yaml"
    )


def test_ros_namespace():

    assert (
        urdf_xml("simulation", "one").find("gazebo/plugin/ros/namespace").text
        == "/robot/base"
    )


def test_hardware_plugin_name():

    ros2_control_urdf_xml = ros2_control_xml("live", "two")
    assert (
        ros2_control_urdf_xml.find("ros2_control/hardware/plugin").text
        == "adap2e_hardware/Adap2eHardware"
    )

    ros2_control_urdf_xml = ros2_control_xml("simulation", "one")

    assert (
        ros2_control_urdf_xml.find("ros2_control/hardware/plugin").text
        == "romea_mobile_base_gazebo/GazeboSystemInterface4WS4WD"
    )


def test_controller_filename_name():

    assert (
        urdf_xml("simulation", "two").find("gazebo/plugin/parameters").text
        == "simulation_two_controller.yaml"
    )
