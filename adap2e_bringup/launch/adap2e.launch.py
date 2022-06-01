from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import yaml


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    launch_gazebo = LaunchConfiguration("launch_gazebo").perform(context)

    if robot_namespace:
        robot_description_name = "/" + robot_namespace + "/robot_description"
        controller_manager_name = "/" + robot_namespace + "/controller_manager"
        joints_prefix = robot_namespace + "_"
    else:
        robot_description_name = "/robot_description"
        controller_manager_name = "/controller_manager"
        joints_prefix = ""

    launch_gazebo = (mode == "simulation") and launch_gazebo
    use_sim_time = (mode == "simulation") or (mode == "replay")

    base_description_yaml_file = (
        get_package_share_directory("adap2e_description")
        + "/config/adap2e_"
        + robot_model
        + ".yaml"
    )

    joystick_remapping_yaml_file = (
        get_package_share_directory("romea_teleop")
        + "/config/"
        + joystick_type
        + "_two_axle_steering_remappings.yaml"
    )

    controller_manager_yaml_file = (
        get_package_share_directory("adap2e_bringup")
        + "/config/controller_manager.yaml"
    )

    base_controller_yaml_file = (
        get_package_share_directory("adap2e_bringup")
        + "/config/mobile_base_controller.yaml"
    )

    xacro_file = (
        get_package_share_directory("adap2e_description")
        + "/urdf/adap2e_"
        + robot_model
        + ".urdf.xacro"
    )

    full_base_controller_yaml_file = "/tmp/mobile_base_controller.yaml"

    command_message_type = "romea_mobile_base_msgs/TwoAxleSteeringCommand"
    command_message_priority = 100

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={"verbose": "false"}.items(),
        condition=IfCondition(str(launch_gazebo)),
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " prefix:=",
            joints_prefix,
            " mode:=",
            mode,
            " controller_conf_yaml_file:=",
            controller_manager_yaml_file,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        namespace=robot_namespace,
    )

    spawn_entity = Node(
        condition=LaunchConfigurationEquals("mode", "simulation"),
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            robot_description_name,
            "-entity",
            robot_namespace,
            "-robot_namespace",
            robot_namespace,
        ],
        output="screen",
        namespace=robot_namespace,
    )

    controller_manager = Node(
        condition=LaunchConfigurationEquals("mode", "live"),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_manager_yaml_file],
        namespace=robot_namespace,
        output="screen",
    )

    joint_state_broadcaster = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
        output="screen",
        namespace=robot_namespace,
    )

    with open(base_description_yaml_file, "r") as f:
        base_description_root = yaml.load(f, Loader=yaml.FullLoader)
        base_description_node = base_description_root["/**"]
        base_description_ros_params = base_description_node["ros__parameters"]
        base_info = base_description_ros_params["base_info"]

    with open(base_controller_yaml_file, "r") as f:
        base_controller_root = yaml.load(f, Loader=yaml.FullLoader)
        base_controller_node = base_controller_root["/**"]
        base_controller_ros_params = base_controller_node["ros__parameters"]
        base_controller_ros_params["base_info"] = base_info
        base_controller_ros_params["controller"]["joints_prefix"] = joints_prefix

    with open(full_base_controller_yaml_file, "w") as f:
        yaml.dump(base_controller_root, f)

    mobile_base_controller = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mobile_base_controller",
            "--param-file",
            full_base_controller_yaml_file,
            "--controller-manager",
            controller_manager_name,
        ],
        output="screen",
        namespace=robot_namespace,
    )

    joy = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
        package="joy",
        executable="joy_node",
        name="joy",
        namespace=robot_namespace,
        output="log",
    )

    wheels_steering_control_info = base_info["wheels_steering_control"]
    wheels_steering_command_info = wheels_steering_control_info["command"]
    maximal_steering_angle = wheels_steering_command_info["maximal_angle"]

    wheels_speed_control_info = base_info["wheels_speed_control"]
    wheels_speed_command_info = wheels_speed_control_info["command"]
    maximal_wheel_speed = wheels_speed_command_info["maximal_speed"]

    teleop = Node(
        package="romea_teleop",
        executable="two_axle_steering_teleop_node",
        name="teleop",
        parameters=[
            {"joystick.type": LaunchConfiguration("joystick_type")},
            joystick_remapping_yaml_file,
            {"cmd_output.type": command_message_type},
            {"cmd_output.priority": command_message_priority},
            {"cmd_range.maximal_linear_speed.slow_mode": 1.0},
            {"cmd_range.maximal_linear_speed.turbo_mode": maximal_wheel_speed},
            {"cmd_range.maximal_front_steering_angle": maximal_steering_angle},
            {"cmd_range.maximal_rear_steering_angle": maximal_steering_angle},
        ],
        remappings=[("cmd_two_axle_steering", "~/cmd_two_axle_steering")],
        namespace=robot_namespace,
        output="screen",
    )

    cmd_mux = Node(
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": command_message_type}],
        remappings=[("~/out", "cmd_two_axle_steering")],
        output="screen",
        namespace=robot_namespace,
    )

    return [
        gazebo,
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=use_sim_time),
                robot_state_publisher,
                spawn_entity,
                controller_manager,
                joint_state_broadcaster,
                mobile_base_controller,
                joy,
                teleop,
                cmd_mux,
            ]
        ),
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    declared_arguments.append(DeclareLaunchArgument("robot_model", default_value="fat"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="adap2e")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_type", default_value="xbox")
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_gazebo", default_value="True")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
