"""
GZ Sim (Gazebo) STAGED launch file for the BGR car.

Implements the 3-stage event-driven pipeline described in launch/README.md.
Unlike gazebo.launch.py (which uses blind TimerActions), this file uses
Readiness Verification Gates: each stage only fires when the previous one
is *actually* ready, not after an arbitrary fixed delay.

Stage 1 → Gazebo + Bridge start. Gate: wait for /clock.
Stage 2 → Vehicle spawns.         Gate: wait for /model/bgr/odometry.
Stage 3 → All tooling launches.   Gate: GUI tracker retry-loop.
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
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PythonExpression
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Where the package 'bgr_description' keeps its files.
    bgr_description = get_package_share_directory("bgr_description")

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='Acceleration.world',
        description='Name of the .world file to load'
    )

    # This waits until runtime to combine: [package_path] + "worlds" + [user_input]
    world_file_path = PathJoinSubstitution([
        bgr_description,
        "worlds",
        LaunchConfiguration("world_name")
    ])

    # The models are now installed dynamically via CMakeLists
    fsa_models_path = os.path.join(bgr_description, "TracksV0", "models")

    # Set GZ_SIM_RESOURCE_PATH to find robot and track models.
    # We must explicitly prepend the current workspace's install and src directories
    # to avoid conflicts with older `bgr_ws` workspaces that might be sourced in .bashrc.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.dirname(bgr_description),
            ":",
            fsa_models_path  
        ]
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bgr_description, "urdf", "bgr.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Run Gazebo headlessly (server only)",
    )
    headless = LaunchConfiguration("headless")

    # Inserts the "-s" (server-only/headless) flag if headless=True.
    # If headless=True, it runs without GUI.
    gz_args = PythonExpression([
        '" -s -v 4 -r " + "', world_file_path, '" if "', LaunchConfiguration("headless"), '" in ["True", "true", "1"] else " -v 4 -r " + "', world_file_path, '"'
    ])

    # Pick which Gazebo plugin family to use.
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

    # Pass the dynamic 'world_file_path' to Gazebo via gz_args.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", gz_args)],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/generated_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/bgr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/front_cam@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[('/lidar/points', '/scan/points')],
        output="screen",
    )

    # STAGE 1 GATE
    # Process blocks until single /clock message is received, meaning Gazebo
    # and ROS bridge are loaded. Triggers Stage 2 when message received.
    stage1_gate = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/clock'],
        output='screen'
    )

    # STAGE 2: VEHICLE SPAWN
    # Activates after Stage 1. Added 7 sec buffer to allow extra loading time.
    gz_spawn_entity = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='[STAGE 2 START] Clock detected. Buffering for physics meshes...'),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world", "generated_world",
                    "-topic", "robot_description",
                    "-name", "bgr",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "1",
                ],
            )
        ]
    )

    # STAGE 2 GATE
    # Blocks until the vehicle's odometry topic appears,
    # This happens once the URDF loaded into the Gazebo.
    stage2_gate = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/model/bgr/odometry'],
        output='screen'
    )

    # STAGE 3 NODES
    # Originally all nodes fired at once. Now the car nodes fire up only after
    # the previous stages completed, making sure the car physically exists.
    car_state_node = Node(
        package="bgr_description",
        executable="car_state_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    car_wheel_node = Node(
        package="bgr_description",
        executable="car_wheel_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    car_dashboard_node = Node(
        package="bgr_description",
        executable="car_dashboard.py",
        output="screen",
        condition=UnlessCondition(headless) # In headless mode this is irrelevant
    )
    cone_service_node = Node(
        package="bgr_description",
        executable="cone_service.py",
        name="cone_service",
        output="screen"
    )
    noisy_sensor_node = Node(
        package="bgr_description",
        executable="noisy_sensor_publisher.py",
        output="screen"
    )
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "bgr/base_footprint/lidar"],
        output="screen"
    )

    # STAGE 3 GATE: GUI TRACKER WITH RETRY LOOP
    # Tries to send follow command up repeatedly, when the GUI is up.
    # Skipped in headless.
    car_tracker = ExecuteProcess(
        cmd=['bash', '-c',
             'for i in $(seq 1 30); do '
             'gz service -s /gui/follow '
             '--reqtype gz.msgs.StringMsg '
             '--reptype gz.msgs.Boolean '
             '--timeout 1000 '
             '--req \'data: "bgr"\' 2>/dev/null && '
             'echo "[GUI TRACKER] Successfully locked onto the vehicle!" && break; '
             'sleep 1; done'],
        output='screen',
        condition=UnlessCondition(headless) # In headless mode this is irrelevant
    )

    # EVENT HANDLERS — Chains the stages together
    # Each RegisterEventHandler watches for it's designated process to exit and
    # then executes the next step.

    # Stage 1 → Stage 2: once /clock is seen, spawn vehicle + monitor odometry.
    stage1_to_stage2 = RegisterEventHandler(
        OnProcessExit(
            target_action=stage1_gate,
            on_exit=[
                gz_spawn_entity,
                stage2_gate,
            ]
        )
    )

    # Stage 2 → Stage 3: when odometry appears, launch the car related nodes.
    stage2_to_stage3 = RegisterEventHandler(
        OnProcessExit(
            target_action=stage2_gate,
            on_exit=[
                LogInfo(msg='[STAGE 3 START] Vehicle is responsive. Launching tooling...'),
                car_state_node,                 # starts the car state publisher node
                car_wheel_node,                 # starts the car wheel publisher node
                car_dashboard_node,             # starts the car dashboard GUI node
                noisy_sensor_node,              # starts the IMU and GPS simulation node
                cone_service_node,              # starts the cone service node
                static_tf_node,                 # starts the static TF publisher node
                car_tracker,                    # makes GUI follow the car
            ]
        )
    )

    # LAUNCH DESCRIPTION
    # We only start the core infrastructure immediately. Everything else is triggered by events.
    return LaunchDescription([
        headless_arg,               # toggles headless mode
        world_arg,                  # selects the world file
        model_arg,                  # lets you override the URDF path
        gazebo_resource_path,       # tells GZ where to find assets
        robot_state_publisher_node, # starts robot_state_publisher
        gazebo,                     # starts the simulator
        gz_ros2_bridge,             # bridges /clock and sensor topics
        stage1_gate,                # starts immediately, watches for /clock
        stage1_to_stage2,           # chains Stage 1 → Stage 2 on /clock ready
        stage2_to_stage3,           # chains Stage 2 → Stage 3 on odometry ready
    ])
