from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_share = get_package_share_directory('bgr_description')
    ctrl_share = get_package_share_directory('bgr_controller')

    gazebo_launch = os.path.join(desc_share, 'launch', 'gazebo.launch.py')
    controllers_launch = os.path.join(ctrl_share, 'launch', 'controller.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(controllers_launch)),
    ])
