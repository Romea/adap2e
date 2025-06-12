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

from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)
    joystick_model = LaunchConfiguration("joystick_model").perform(context)

    robot = []

    if mode == "simulation":
        mode += "_gazebo_classic"

    if mode == "simulation_gazebo_classic":

        world = PathJoinSubstitution(
            [FindPackageShare("romea_simulation_gazebo_worlds"), "worlds", "friction_cone.world"]
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"]
                        )
                    ]
                ),
                launch_arguments={"world": world, "verbose": "false"}.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"]
                        )
                    ]
                )
            )
        )

        robot_description_file = "/tmp/adap2e_description.urdf"
        with open(robot_description_file, "w") as f:
            f.write(robot_urdf_description)

        robot.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                exec_name="gazebo_spawn_entity",
                arguments=["-file", robot_description_file, "-entity", "adap2e"],
                output={"stdout": "log", "stderr": "log"},
            )
        )

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [FindPackageShare("adap2e_bringup"), "launch", "adap2e_base.launch.py"]
                    )
                ]
            ),
            launch_arguments={"robot_model": robot_model, "mode": mode}.items(),
        )
    )

    joystick_configuration_file_path = (
        get_package_share_directory("romea_joystick_utils") + "/config/" + joystick_model + ".yaml"
    )

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("adap2e_bringup")
                + "/launch/adap2e_teleop.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "robot_model": robot_model,
                "joystick_configuration_file_path": joystick_configuration_file_path,
                "joystick_topic": "/adap2e/joystick/joy"
            }.items(),
        ),
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("adap2e"),
                PushRosNamespace("joystick"),
                Node(package="joy", executable="joy_node")
            ]
        )
    )

    return robot


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    declared_arguments.append(DeclareLaunchArgument("robot_model", default_value="fat"))

    urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", "adap2e_bringup"),
            " robot_namespace:adap2e",
            " robot_model:",
            LaunchConfiguration("robot_model"),
            " base_name:base",
            " mode:",
            LaunchConfiguration("mode"),
        ],
        on_stderr="ignore",
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_urdf_description", default_value=urdf_description)
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_model", default_value="microsoft_xbox")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
