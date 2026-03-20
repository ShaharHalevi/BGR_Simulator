"""
Controllers launch.

What this file does:
1) Starts the joint_state_broadcaster (publishes /joint_states).
2) Starts the velocity controller (forward_velocity_controller).
3) Starts the position controller (forward_position_controller).

Notes:
- The controller names here must be exactly the same as in your
  ros2_control YAML (e.g., bgr_controllers.yaml).
- We point to '/controller_manager' which is the default for the robot.
  If you use a namespace, change it (e.g., '/bgr/controller_manager').
- The joy_array_bridge is intentionally NOT launched here.
  Run bridge.launch.py separately after this file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1) Joint State Broadcaster
    # Publishes /joint_states from the controller manager.
    # This should start before other controllers.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 2) Velocity Controller
    # Sends velocity commands to joints (as defined in your YAML).
    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 3) Position Controller
    # Sends position commands to joints (e.g., steering joints).
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,  # start first
            velocity_controller,              # then velocity controller
            position_controller,              # then position controller
            # joy_array_bridge is launched separately via bridge.launch.py
        ]
    )
