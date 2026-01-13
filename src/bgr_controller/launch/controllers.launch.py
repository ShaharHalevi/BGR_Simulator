"""
Controllers launch.

What this file does:
1) Starts the joint_state_broadcaster (publishes /joint_states).
2) Starts your effort controller (name must match your YAML).
3) Starts your position controller (name must match your YAML).
4) Starts the AMK motor controller logic (converts throttle to torque).
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1) Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 2) Effort Controller
    effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_effort_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 3) Position Controller
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 4) Motor Controller Node (The new addition)
    # This connects the /throttle topic to the /forward_effort_controller/commands
    motor_controller_node = Node(
        package="bgr_controller",
        executable="motor_controller.py",
        name="motor_controller",
        output="screen",
        parameters=[{'use_sim_time': True}] # Ensures it stays synced with Gazebo clock
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            effort_controller,
            position_controller,
            motor_controller_node, # Added to the launch list
        ]
    )