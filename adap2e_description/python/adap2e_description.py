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


import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from romea_mobile_base_description import (
    get_command_limits,
    get_command_type,
    get_inertia,
    get_wheelbase,
    get_track,
)


def get_complete_configuration_path_file(robot_model):
    return (
        get_package_share_directory("adap2e_description")
        + "/config/adap2e_"
        + robot_model
        + ".yaml"
    )


def get_complete_configuration(robot_model):
    with open(get_complete_configuration_path_file(robot_model), "w") as f:
        return yaml.safe_load(f)


def get_minimal_configuration(robot_model):
    complete_configuration = get_complete_configuration(robot_model)
    return {
        "command_type": get_command_type(complete_configuration),
        "command_limits": get_command_limits(complete_configuration),
        "inertia": get_inertia(complete_configuration),
        "wheelbase": get_wheelbase(complete_configuration),
        "track": get_track(complete_configuration),
    }


def generate_ros2_control_description(prefix, mode, base_name, robot_model):

    ros2_control_xacro_file = (
        get_package_share_directory("adap2e_description")
        + "/ros2_control/adap2e_"
        + robot_model
        + ".ros2_control.urdf.xacro"
    )

    ros2_control_urdf_xml = xacro.process_file(
        ros2_control_xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
        },
    )

    return ros2_control_urdf_xml.toprettyxml(indent="  ")


def generate_urdf_description(
        prefix, mode, base_name, robot_model, controller_manager_config_yaml_file, ros_prefix
):

    if mode == "simulation":
        mode += "_gazebo_classic"

    base_xacro_file = (
        get_package_share_directory("adap2e_description")
        + "/urdf/adap2e_"
        + robot_model
        + ".urdf.xacro"
    )

    base_urdf_xml = xacro.process_file(
        base_xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
            "controller_manager_config_yaml_file": controller_manager_config_yaml_file,
            # "ros_prefix": ros_prefix,
        },
    )

    return base_urdf_xml.toprettyxml(indent="  ")
