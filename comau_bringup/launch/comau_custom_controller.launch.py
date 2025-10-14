"""
Launch file: comau_custom_controller.launch.py

Purpose:
    Includes the core bring-up (comau_core.launch.py) and spawns a custom
    ros2_control controller, passing a user-provided parameter file.
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
    Create the LaunchDescription for core bring-up + a custom controller.

    This launch file:
      1) Includes comau_core.launch.py, forwarding an optional parameter file.
      2) Spawns the <controller>_controller via controller_manager's spawner,
        applying parameters from param_file.

    Returns:
        LaunchDescription: Actions to start the driver and the specified custom controller.
    """

    ld = []

    ld.append(
        DeclareLaunchArgument(
            "controller",
            description="Name of controller to spawn. This must be a prefix to a '_controller' package.",
        )
    )
    controller = LaunchConfiguration("controller")
    controller_full_name = PathJoinSubstitution([[controller,"_controller"]])

    ld.append(
        DeclareLaunchArgument(
            "param_file",
            description="Path to parameter file specific to this controller.",
        )
    )
    param_file_path = LaunchConfiguration("param_file")
    controller_full_name = PathJoinSubstitution([[controller,"_controller"]])

    # Start Comau ROS2 Driver
    ld.append(
        IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('comau_bringup'), 'launch', 'comau_core.launch.py']),
        launch_arguments={'robot_type': 'nj_130_2_6',
                          'extra_parameter_file' : param_file_path
                          }.items()
    ))
    
    # Spawn controller
    ld.append(
        Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_full_name, 
                   "--controller-manager", "/controller_manager",
                   "-p", param_file_path
                   ],
    ))
    
    return LaunchDescription(ld)