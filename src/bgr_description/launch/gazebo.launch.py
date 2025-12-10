"""
GZ Sim (Gazebo) launch file for the BGR car.

What this file does:
1) Loads  URDF/Xacro and creates a 'robot_description' string.
2) Tells GZ Sim where to find meshes and resources.
3) Starts GZ Sim with an empty world.
4) Spawns the robot into the world.
5) Starts robot_state_publisher (publishes TF from the URDF).
6) Bridges /clock from GZ to ROS 2 (so nodes can use simulated time).
"""


import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Where the package 'bgr_description' keeps its files.
    bgr_description = get_package_share_directory("bgr_description")

    world_path = os.path.join(bgr_description, "worlds", "track1.sdf")

    # Launch argument for the robot model file to use.
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bgr_description, "urdf", "bgr.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=[str(Path(bgr_description).parent.resolve())]
    )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # *** פה מחליפים את empty.sdf ב-world_path ***
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[
            (
                "gz_args",
                [" -v 4 -r ", world_path],
            )
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "bgr",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "1.0",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            gazebo_resource_path,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
        ]
    )
