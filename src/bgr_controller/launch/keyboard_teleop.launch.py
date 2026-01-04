from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time",
    )

    
    joy_array_bridge = Node(
        package="bgr_controller",
        executable="joy_array_bridge.py",
        name="joy_array_bridge",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_array_bridge,
        ]
    )