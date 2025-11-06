"""
Display launch for RViz.

What this file does:
1) Loads your Xacro/URDF into 'robot_description'.
2) Starts robot_state_publisher (publishes TF from the URDF).
3) Starts joint_state_publisher_gui so you can move joints by sliders.
4) Opens RViz with a predefined config file.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Where the package 'bgr_description' keeps its files (URDF, meshes, RViz config).
    bgr_description_dir = get_package_share_directory("bgr_description")

    # Launch argument for the robot model file to use.
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bgr_description_dir, "urdf", "bgr.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    # Create robot_description parameter from xacro file.
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

     # Publishes TF tree and the 'robot_description' param so RViz can visualize the robot.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Joint state publisher GUI to move joints with sliders.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    # RViz node to visualize the robot.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bgr_description_dir, "rviz", "display.rviz")],
    )

     # Return everything to be launched.
    return LaunchDescription(
        [
            model_arg,                          # allows overriding the Xacro path
            joint_state_publisher_gui_node,     # sliders for joints
            robot_state_publisher_node,         # publishes TF and robot_description
            rviz_node,                          # starts RViz    
        ]
    )
