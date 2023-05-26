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


from launch import LaunchDescription  # , LaunchContext

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    base_name = LaunchConfiguration("base_name").perform(context)
    urdf_description = LaunchConfiguration("urdf_description").perform(context)

    if robot_namespace:
        controller_manager_name = "/" + robot_namespace + "/"+base_name+"/controller_manager"
        robot_prefix = robot_namespace + "_"
    else:
        controller_manager_name = "/"+base_name+"/controller_manager"
        robot_prefix = ""

    use_sim_time = (mode == "simulation") or (mode == "replay")

    base_description_yaml_file = (
        get_package_share_directory("adap2e_description")
        + "/config/adap2e_"
        + robot_model
        + ".yaml"
    )

    controller_manager_yaml_file = (
        get_package_share_directory("adap2e_bringup")
        + "/config/controller_manager.yaml"
    )

    base_controller_yaml_file = (
        get_package_share_directory("adap2e_bringup")
        + "/config/mobile_base_controller.yaml"
    )

    robot_description_file = "/tmp/"+robot_prefix+"description.urdf"
    with open(robot_description_file, "w") as f:
        f.write(urdf_description)

    base_ros2_control_description_file = "/tmp/"+robot_prefix+base_name+"_ros2_control.urdf"
    with open(base_ros2_control_description_file, "r") as f:
        base_ros2_control_description = f.read()

    spawn_entity = Node(
        condition=LaunchConfigurationEquals("mode", "simulation"),
        package="gazebo_ros",
        executable="spawn_entity.py",
        exec_name="gazebo_spawn_entity",
        arguments=[
            "-file",
            robot_description_file,
            "-entity",
            robot_namespace,
        ],
        output={
            'stdout': 'log',
            'stderr': 'log',
        }
    )

    controller_manager = Node(
        condition=LaunchConfigurationEquals("mode", "live"),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": base_ros2_control_description},
            controller_manager_yaml_file
        ],
        # output="screen",
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_mobile_base_controllers"),
                        "launch",
                        "mobile_base_controller.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "joints_prefix": robot_prefix,
            "controller_name": "mobile_base_controller",
            "controller_manager_name": controller_manager_name,
            "base_description_yaml_filename": base_description_yaml_file,
            "base_controller_yaml_filename": base_controller_yaml_file,
        }.items(),
        condition=LaunchConfigurationNotEquals("mode", "replay"),
    )

    cmd_mux = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": "romea_mobile_base_msgs/TwoAxleSteeringCommand"}],
        remappings=[("~/out", "controller/cmd_two_axle_steering")],
        output="screen",
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=use_sim_time),
                PushRosNamespace(robot_namespace),
                spawn_entity,
                PushRosNamespace(base_name),
                controller_manager,
                controller,
                cmd_mux,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_model"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="adap2e")
    )

    declared_arguments.append(
        DeclareLaunchArgument("base_name", default_value="base")
    )

    urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", "adap2e_bringup"),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " robot_model:",
            LaunchConfiguration("robot_model"),
            " base_name:",
            LaunchConfiguration("base_name"),
            " mode:",
            LaunchConfiguration("mode"),
        ]
    )

    declared_arguments.append(
        DeclareLaunchArgument("urdf_description", default_value=urdf_description)
    )

    # DeclareLaunchArgument("mode").execute(LaunchContext)
    # mode = LaunchConfiguration("mode").perform(LaunchContext)
    # print()
    # if mode == "simulation":
    # declared_arguments.append(DeclareLaunchArgument("xyz"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
