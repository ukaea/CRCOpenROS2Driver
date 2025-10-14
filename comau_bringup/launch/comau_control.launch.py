"""
Launch file: comau_control.launch.py

Purpose:
    Includes the core driver bring-up (comau_core.launch.py) and spawns a
    topic-based controller via controller_manager's spawner.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    """
    Create the LaunchDescription for core bring-up + a selected topic controller.

    This launch file:
      1) Includes comau_core.launch.py to start the ROS 2 control stack and robot_state_publisher.
      2) Spawns one topic-based controller chosen by the mode launch argument.

    Returns:
        LaunchDescription: Actions required to start the driver and the requested controller.
    """
    ld = []

    ld.append(
        DeclareLaunchArgument(
            "mode",
            description="Topic command controller to spawn.",
            choices=["position", "velocity", "acceleration", "current", "torque"]
        )
    )
    mode = LaunchConfiguration("mode")

    ld.append(
        IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('comau_bringup'), 'launch', 'comau_core.launch.py']),
        launch_arguments={'robot_type': 'nj_130_2_6'}.items()
    ))

    ld.append(
        Node(
        package="controller_manager",
        executable="spawner",
        arguments=[PathJoinSubstitution([[mode,"_controller"]]), "--controller-manager", "/controller_manager"],
    ))
    
    return LaunchDescription(ld)