"""
Controllers launch.

This version fixes startup race conditions by spawning controllers in order:

1) joint_state_broadcaster
2) forward_velocity_controller
3) forward_position_controller

Why:
ROS 2 launch starts Nodes in parallel by default.
So simply writing them in order inside LaunchDescription does NOT guarantee order.

This file uses OnProcessExit so each controller starts only after the previous
spawner finished successfully.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


CONTROLLER_MANAGER = "/controller_manager"


def make_spawner(controller_name: str) -> Node:
    """
    Create a controller spawner node.

    --controller-manager:
        Tells the spawner which controller_manager service to use.

    --controller-manager-timeout 60:
        Wait up to 60 seconds for /controller_manager to exist.

    --service-call-timeout 60:
        Wait up to 60 seconds for controller_manager service calls to answer.
        This helps when Gazebo / ros2_control is still loading.
    """
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawner_{controller_name}",
        output="screen",
        arguments=[
            controller_name,
            "--controller-manager",
            CONTROLLER_MANAGER,
            "--controller-manager-timeout",
            "60",
            "--service-call-timeout",
            "60",
        ],
    )


def generate_launch_description():

    joint_state_broadcaster_spawner = make_spawner(
        "joint_state_broadcaster"
    )

    velocity_controller_spawner = make_spawner(
        "forward_velocity_controller"
    )

    position_controller_spawner = make_spawner(
        "forward_position_controller"
    )

    start_velocity_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    start_position_after_velocity = RegisterEventHandler(
        OnProcessExit(
            target_action=velocity_controller_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            start_velocity_after_joint_state,
            start_position_after_velocity,
        ]
    )