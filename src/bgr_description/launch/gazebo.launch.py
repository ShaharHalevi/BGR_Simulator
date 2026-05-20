"""
GZ Sim (Gazebo) STAGED launch file for the BGR car.

Implements the 3-stage event-driven pipeline described in launch/README.md.
Unlike gazebo.launch.py (which uses blind TimerActions), this file uses
Readiness Verification Gates: each stage only fires when the previous one
is *actually* ready, not after an arbitrary fixed delay.

Stage 1 → Gazebo + Clock Bridge. Gate: wait for /clock.
Stage 2 → GUI readiness check.   Gate: wait for /gui/ services.
Stage 3 → Vehicle + Sensor Bridge. Gate: wait for /model/bgr/odometry.
Stage 4 → All tooling launches.   Gate: GUI tracker retry-loop.
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

    # BRIDGE 1: SYSTEM CLOCK
    # Only bridge /clock initially to keep startup overhead low.
    gz_ros2_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # BRIDGE 2: VEHICLE SENSORS
    # Bridges all vehicle-specific topics once the car is actually spawned.
    gz_ros2_vehicle_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/generated_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/bgr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/front_cam@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[('/lidar/points', '/lidar/raw')],
        output="screen",
    )

    # STAGE 1 GATE
    # Triggers Stage 2 when simulation time ticks past 0.1s, guaranteeing that the world 
    # is fully loaded (avoiding long waits on heavy maps with low RTF).
    stage1_gate = ExecuteProcess(
        cmd=['python3', '-c', "import rclpy; from rosgraph_msgs.msg import Clock; rclpy.init(); node=rclpy.create_node('gate'); node.create_subscription(Clock, '/clock', lambda msg: exit(0) if (msg.clock.sec > 0 or msg.clock.nanosec >= 100000000) else None, 1); rclpy.spin(node)"],
        output='screen'
    )

    # STAGE 2 GATE: GUI READINESS
    # Deterministically waits for the Gazebo GUI to finish initializing its rendering 
    # pipeline by checking for the existence of /gui/ services.
    gui_ready_gate = ExecuteProcess(
        cmd=['sh', '-c', 
             'if [ "$1" = "True" ] || [ "$1" = "true" ] || [ "$1" = "1" ]; then '
             '  echo "[STAGE 2] Headless mode detected. Skipping GUI readiness check."; '
             '  exit 0; '
             'fi; '
             'echo "[STAGE 2] Waiting for Gazebo GUI services to initialize..."; '
             'for i in $(seq 1 60); do '
             '  if gz service -l | grep -q "/gui/"; then '
             '    echo "[STAGE 2 COMPLETE] Gazebo GUI is ready."; '
             '    exit 0; '
             '  fi; '
             '  sleep 1; '
             'done; '
             'echo "[STAGE 2 WARNING] GUI services not found after 60s. Proceeding anyway..."; '
             'exit 0;',
             'gui_gate_script', headless],
        output='screen'
    )

    # STAGE 3: VEHICLE SPAWN
    # Activates immediately upon Stage 2 completion.
    # Wraps the spawn command in a retry loop to prevent timeouts on slow hardware
    # when loading heavy worlds (like CompetitionMap1.world).
    gz_spawn_entity = ExecuteProcess(
        cmd=['bash', '-c',
             'for i in $(seq 1 30); do '
             'echo "[STAGE 3] Attempting to spawn vehicle..." && '
             'ros2 run ros_gz_sim create -world generated_world -topic robot_description -name bgr -x 0.0 -y 0.0 -z 1 && '
             'echo "[STAGE 3 SUCCESS] Vehicle spawn request accepted!" && break; '
             'echo "[STAGE 3 WARNING] Spawn request timed out or failed. Gazebo is busy loading world. Retrying in 2s..." && '
             'sleep 2; done'],
        output='screen'
    )

    # STAGE 3 GATE
    # Blocks until the vehicle's odometry topic starts publishing.
    # This happens once the URDF loaded into the Gazebo.
    stage3_gate = ExecuteProcess(
        cmd=['python3', '-c', "import rclpy; from nav_msgs.msg import Odometry; rclpy.init(); node=rclpy.create_node('gate'); node.create_subscription(Odometry, '/model/bgr/odometry', lambda msg: exit(0), 1); rclpy.spin(node)"],
        output='screen'
    )

    # STAGE 4 NODES
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
        condition=UnlessCondition('true' if os.environ.get('GITHUB_ACTIONS') == 'true' else 'false') # Won't display in Git Actions
    )
    cone_service_node = Node(
        package="bgr_description",
        executable="cone_service.py",
        name="cone_service",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    visible_cones_node = Node(
        package="bgr_description",
        executable="visible_cones.py",
        name="visible_cones_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "world_name": LaunchConfiguration("world_name")
        }],
    )
    noisy_sensor_node = Node(
        package="bgr_description",
        executable="noisy_sensor_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "base_link", "--child-frame-id", "bgr/base_footprint/lidar"],
        output="screen"
    )
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('bgr_controller'), '/launch/controller.launch.py'
        ])
    )

    # STAGE 3 GATE: GUI tracker with tracking loop.
    # Sends follow command up repeatedly until the GUI successfully follows the car.
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

    # Stage 1 → Stage 2: once /clock >= 0.1s, start checking for GUI.
    stage1_to_stage2 = RegisterEventHandler(
        OnProcessExit(
            target_action=stage1_gate,
            on_exit=[gui_ready_gate]
        )
    )

    # Stage 2 → Stage 3: once GUI is ready (or skipped), spawn vehicle and sensor bridge.
    stage2_to_stage3 = RegisterEventHandler(
        OnProcessExit(
            target_action=gui_ready_gate,
            on_exit=[
                LogInfo(msg='[STAGE 3 START] Spawning vehicle and monitoring odometry...'),
                gz_ros2_vehicle_bridge,
                gz_spawn_entity,
                stage3_gate,
            ]
        )
    )

    stage4_to_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=stage3_gate,
            on_exit=[
                LogInfo(msg='[CONTROLLERS] Spawning joint controllers...'),
                controller_launch,              # spawns joint_state_broadcaster + velocity/position controllers
            ]
        )
    )

    # Stage 3 → Stage 4: when odometry appears, launch the car related nodes.
    stage3_to_stage4 = RegisterEventHandler(
        OnProcessExit(
            target_action=stage3_gate,
            on_exit=[
                LogInfo(msg='[STAGE 4 START] Vehicle is responsive. Launching tooling...'),
                car_state_node,                 # starts the car state publisher node
                car_wheel_node,                 # starts the car wheel publisher node
                car_dashboard_node,             # starts the car dashboard GUI node
                noisy_sensor_node,              # starts the IMU and GPS simulation node
                cone_service_node,              # starts the cone service node
                visible_cones_node,             # starts the visible cones streaming node
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
        gz_ros2_clock_bridge,       # bridges /clock ONLY
        stage1_gate,                # starts immediately, watches for /clock
        stage1_to_stage2,           # chains Stage 1 → Stage 2
        stage2_to_stage3,           # chains Stage 2 → Stage 3
        stage3_to_stage4,           # chains Stage 3 → Stage 4 on odometry ready
        stage4_to_controllers,      # chains Stage 3 → Controllers (runs in parallel with Stage 4)
    ])
