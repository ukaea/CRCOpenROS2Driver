"""
Launch file: comau_core.launch.py

Purpose:
    Core bring-up for the Comau CRCOpen ROS 2 driver using ros2_control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    """
    Create the LaunchDescription for the core driver stack.

    This launch file:
      - Accepts a robot_type to select the appropriate description package.
      - Generates robot_description via xacro.
      - Starts ros2_control_node with the controllers configuration.
      - Starts robot_state_publisher.
      - Spawns joint_state_broadcaster.

    Returns:
        LaunchDescription: Actions to bring up ros2_control and state publishing.
    """
    # Parameter arguments
    declared_arguments = []

    # Choose the robot model
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="nj_130_2_6",
            description="Robot model number (note: use '-' instead of '.' or ' ')",
            choices=["nj_130_2_6",]
        )
    )
    robot_type = LaunchConfiguration("robot_type")
    robot_package = FindPackageShare(PathJoinSubstitution([['comau_',robot_type,'_description']]))

    # Allow additional parameters for custom controllers
    declared_arguments.append(
        DeclareLaunchArgument(
            "extra_parameter_file",
            default_value="",
            description="Path to additional parameter file to be passed to the controller manager.",
        )
    )
    extra_parameter_file = LaunchConfiguration("extra_parameter_file")

    # Get robot URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            robot_package,
            "urdf",
            [robot_type,'.urdf.xacro']
        ]),
        " ",
        "hardware_type:=",
        "crcopen",
    ])
 
    robot_description = {"robot_description": ParameterValue(robot_description_content,value_type=str)}

    # Get robot controllers config yaml
    robot_controllers = PathJoinSubstitution([
        robot_package,
        "controllers",
        "controllers.yaml",
    ])

    # Initialise launch nodes list
    nodes = []

    # ROS2 Controller Manager Node
    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters = [robot_controllers, extra_parameter_file],
        output="both",
    ))

    # Robot State Publisher
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    ))

    # Spawn Joint State Broadcaster to publish joint_states
    nodes.append(Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    ))
    
    return LaunchDescription(declared_arguments + nodes)