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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

import romea_common_meta_bringup.ros_launch as common
import romea_joystick_meta_bringup.ros_launch as joystick


def launch_setup(context, *args, **kwargs):

    mode = common.get_mode(context)
    robot_model = common.get_robot_model(context)

    joystick_configuration_file_path = (
        get_package_share_directory("romea_joystick_utils")
        + "/config/" + joystick.get_joystick_model(context) + ".yaml"
    )

    robot = []

    if "simulation" in mode:

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("adap2e_bringup")
                    + "/launch/adap2e_gazebo.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "robot_model": robot_model,
                    "robot_namespace": "adap2e",
                    "base_name": "base",
                }.items(),
            )
        )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("adap2e_bringup")
            + "/launch/adap2e_base.launch.py"
        ),
        launch_arguments={
            "mode": mode,
            "robot_model": robot_model,
            "robot_namespace": "adap2e",
            "base_name": "base",
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("adap2e_bringup")
            + "/launch/adap2e_teleop.launch.py"
        ),
        launch_arguments={
            "mode": mode,
            "robot_model": robot_model,
            "joystick_configuration_file_path": joystick_configuration_file_path,
            "joystick_topic": "/adap2e/joystick/joy",
        }.items(),
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("adap2e"),
                PushRosNamespace("base"),
                base,
                teleop,
            ]
        )
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("adap2e"),
                PushRosNamespace("joystick"),
                Node(package="joy", executable="joy_node"),
            ]
        )
    )

    return robot


def generate_launch_description():

    return LaunchDescription(
        [
            common.declare_mode("simulation"),
            common.declare_robot_model(["fat", "slim"], "fat"),
            joystick.declare_joystick_model("microsoft_xbox"),
            OpaqueFunction(function=launch_setup),
        ]
    )
