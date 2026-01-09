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
    ExecuteProcess,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Where the package 'bgr_description' keeps its files.
    bgr_description = get_package_share_directory("bgr_description")
    #world_path = os.path.join(bgr_description, 'worlds', 'empty.sdf')
    
    # NOTE: Update this path to your adjusted location
    fsa_models_path = os.path.expanduser("~/BGR_WS/BGR_Simulator/src/TracksV0/models")

    # Set the GZ_SIM_RESOURCE_PATH environment variable to include both the package's share directory and the FSA models path.
    # Make GZ Sim look for resources (meshes, textures, etc.) in this folder.
    # We point to the parent folder of the 'share' dir. This helps GZ find assets.

    # # Gazebo Sim process
    # gazebo = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', world_path],
    #     output='screen'
    # )

    # Set GZ_SIM_RESOURCE_PATH to find robot and track models.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bgr_description).parent.resolve()), 
            ":", 
            fsa_models_path  
        ]
    )

    # Launch argument for the robot model file to use.
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bgr_description, "urdf", "bgr.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    # Pick which Gazebo plugin family to use.
    # 'humble' uses Ignition; newer distros use GZ Sim.
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Create robot_description parameter from xacro file.
    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]
        ),
        value_type=str,
    )

    # Start robot_state_publisher node to publish TFs.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Start GZ Sim. We use the 'empty.sdf' world.
    # Flags:
    #   -v 4 : verbose logging
    #   -r   : run immediately
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", [" -v 4", " -r", " empty.sdf"])],
    )

    # Spawn the robot into the world from the 'robot_description' topic.
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

    # Bridge topics from GZ to ROS 2.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/bgr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            #"/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked@/lidar/points",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[('/lidar/points', '/scan/points')],

        output="screen",
    )

    # NOTE: Update this path to your adjusted location (change in track_gui.py too!)
    gui_script_path = os.path.expanduser("~/BGR_WS/BGR_Simulator/src/TracksV0/tracks/track_gui.py")
    
    track_gui_process = ExecuteProcess(
        cmd=['python3', gui_script_path],
        output='screen'
    )
    # --------------------------------------
    # Car & Map specific nodes
    # --------------------------------------

    # Car state publisher node
    car_state_node = Node(
        package="bgr_description",
        executable="car_state_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # Car wheel publisher node
    car_wheel_node = Node(
        package="bgr_description",
        executable="car_wheel_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    # Car dashboard GUI node
    car_dashboard_node = Node(
    package="bgr_description",
    executable="car_dashboard.py",
    output="screen",
    )
    # Cone service node
    cone_service_node = Node(
        package="bgr_description",
        executable="cone_service.py",
        name="cone_service",
        output="screen"
    )
    
    # TF Bridge: Connects the Gazebo Lidar frame to the Robot base frame
    # This makes the fix permanent
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # Arguments: x y z yaw pitch roll parent_frame child_frame
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "bgr/base_footprint/lidar"],
        output="screen"
    )


    # Return everything we want to start.
    return LaunchDescription(
        [
            model_arg,                      # lets you override the URDF path
            gazebo_resource_path,           # tells GZ where to find assets
            robot_state_publisher_node,     # starts robot_state_publisher
            gazebo,                         # starts the simulator
            gz_spawn_entity,                # spawns the robot in GZ
            gz_ros2_bridge,                 # bridges /clock topic
            track_gui_process,              # starts the track GUI
            car_state_node,                 # starts the car state publisher node
            car_wheel_node,                 # starts the car wheel publisher node
            car_dashboard_node,             # starts the car dashboard GUI node
            cone_service_node,               # starts the cone service node
            static_tf_node                 # starts the static TF publisher node
        ]
    )
