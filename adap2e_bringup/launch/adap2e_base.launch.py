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


from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    if "replay" in mode:
        return []

    if mode == "simulation":
        mode += "_gazebo_classic"

    robot_model = LaunchConfiguration("robot_model").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    base_configuration_file_path = (
        f'{get_package_share_directory("adap2e_description")}/config/adap2e_{robot_model}.yaml'
    )

    controller_manager_configuration_file_path = (
        f'{get_package_share_directory("adap2e_bringup")}/config/controller_manager.yaml'
    )

    base_controller_configuration_file_path = (
        f'{get_package_share_directory("adap2e_bringup")}/config/mobile_base_controller.yaml'
    )

    base_ros2_control_description_file_path = f"/tmp/{tf_prefix}base_ros2_control.urdf"
    with open(base_ros2_control_description_file_path, "r") as f:
        base_ros2_control_description = f.read()

    controller_manager = Node(
        condition=IfCondition(PythonExpression(["'gazebo' not in '", mode, "'"])),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": base_ros2_control_description},
            controller_manager_configuration_file_path,
        ],
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_mobile_base_controllers")
            + "/launch/mobile_base_controller.launch.py"
        ),
        launch_arguments={
            "joints_prefix": tf_prefix,
            "controller_name": "mobile_base_controller",
            "base_configuration_file_path": base_configuration_file_path,
            "base_controller_configuration_file_path": base_controller_configuration_file_path,
        }.items(),
    )

    cmd_mux = Node(
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": "romea_mobile_base_msgs/TwoAxleSteeringCommand"}],
        remappings=[("~/out", "controller/cmd_two_axle_steering")],
        output="screen",
    )

    # can_receiver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("ros2_socketcan"),
    #                     "launch",
    #                     "socket_can_receiver.launch.py",
    #                 ]
    #             )
    #         ]
    #     ),
    #     launch_arguments={
    #         "interface": "can0"
    #     }.items(),
    # )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                # can_receiver,
                controller_manager,
                controller,
                cmd_mux,
            ]
        )
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode"),
            DeclareLaunchArgument("robot_model"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
