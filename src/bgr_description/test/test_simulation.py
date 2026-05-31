# Command to run this test suite locally:
# source install/setup.bash && colcon test --packages-select bgr_description --event-handlers console_direct+
# colcon test-result --all --verbose

import os
import subprocess
import math

# Force localhost discovery loopback only if not running in GitHub Actions CI
if not os.environ.get('GITHUB_ACTIONS') == 'true':
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import time
import unittest
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2, JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from bgr_description.msg import ConeArray
from controller_manager_msgs.srv import ListControllers

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_testing


# 1. LAUNCH EXECUTION
# This runs once. Gazebo persists until all test classes complete.
@pytest.mark.launch_test
def generate_test_description():
    headless_val = 'True' if ('DISPLAY' not in os.environ or os.environ.get('GITHUB_ACTIONS') == 'true') else 'False'
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('bgr_description'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={ 
            'headless': headless_val,
            'world_name': 'SkidpadOpt.world'
        }.items()
    )

    return LaunchDescription([
        gazebo_launch,
        launch_testing.actions.ReadyToTest()
    ])


# 2. BASE FIXTURE
# Handles ROS 2 initialization and teardown for every test class
class BaseTestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        
        # Create a temporary node to poll the controller manager silently
        # We wait until the controllers are all 'active'
        startup_node = rclpy.create_node('test_startup_node')
        client = startup_node.create_client(ListControllers, '/controller_manager/list_controllers')
        
        start_wait = time.time()
        while not client.wait_for_service(timeout_sec=0.5) and time.time() - start_wait < 120.0:
            pass
            
        controllers_ready = False
        future = None
        
        while not controllers_ready and time.time() - start_wait < 120.0:
            if future is None:
                future = client.call_async(ListControllers.Request())
                
            rclpy.spin_once(startup_node, timeout_sec=0.1)
            
            if future.done():
                try:
                    res = future.result()
                    states = {c.name: c.state for c in res.controller}
                    if (states.get('joint_state_broadcaster') == 'active' and
                        states.get('forward_velocity_controller') == 'active' and
                        states.get('forward_position_controller') == 'active'):
                        controllers_ready = True
                except Exception:
                    pass
                future = None
                
            time.sleep(0.5)
            
        startup_node.destroy_node()
        time.sleep(3)
        # Finished waiting, start the suite
        suite_logger = rclpy.logging.get_logger('test_suite')
        
        suite_logger.info("\n" + "="*50 + "\n\n" + "="*50)
        suite_logger.info("🚀 [ENVIRONMENT] BGR Test Suite Diagnostics")
        suite_logger.info(f"   - ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'N/A')}")
        suite_logger.info(f"   - ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
        suite_logger.info(f"   - WORKSPACE: {os.getcwd()}")
        suite_logger.info("="*50 + "\n\n" + "="*50)

        if controllers_ready:
            suite_logger.info("✅ [READY] All controllers are active (joint_state_broadcaster, forward_velocity_controller, forward_position_controller). Proceeding to tests.")
            suite_logger.info("⏲️  [WAIT] Waiting 5s for internal sensor pipelines to settle...")
            time.sleep(5)
        else:
            suite_logger.error("❌ [FATAL] Timeout waiting for controllers to become active.")
            raise RuntimeError("Controllers failed to load/activate in time.")

    @classmethod
    def tearDownClass(cls):
        suite_logger = rclpy.logging.get_logger('test_suite')
        suite_logger.info("\n" + "="*50)
        suite_logger.info("🛑 [EXIT] Simulation Test Suite Complete. Shutting down ROS...")
        suite_logger.info("="*50)
        
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(
            f'{self.__class__.__name__}_node',
            parameter_overrides=[rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )
        self.node.get_logger().info(f'🧪 Starting test: {self._testMethodName}')

    def tearDown(self):
        self.node.get_logger().info(f'🏁 Finished test: {self._testMethodName}')
        self.node.destroy_node()

    def wait_for_topic(self, topic_class, topic_name, timeout=35.0):
        self.node.get_logger().info(f'🔄 Waiting for topic: {topic_name} (timeout {timeout}s)...')
        received_msg = None
        start_time = self.node.get_clock().now()
        
        def callback(msg):
            nonlocal received_msg
            received_msg = msg

        sub = self.node.create_subscription(
            topic_class,
            topic_name,
            callback,
            qos_profile_sensor_data
        )
        
        while received_msg is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            elapsed = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= timeout:
                break
            
        self.node.destroy_subscription(sub)
        
        if received_msg is not None:
             elapsed = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
             self.node.get_logger().info(f'✅ SUCCESS: Data received on {topic_name} after {elapsed:.2f}s!')
        else:
             self.node.get_logger().error(f'❌ FAILURE: Timeout reached for {topic_name} after {timeout}s!')
             
        return received_msg

    def test_01_lidar_active(self):
        """Verification Phase 1: LiDAR Connectivity and Density"""
        self.node.get_logger().info('--- Verification Phase 1: LiDAR ---')
        msg = self.wait_for_topic(PointCloud2, '/lidar/raw')
        self.assertIsNotNone(msg, "Failed: No PointCloud2 data on /scan/points (LiDAR)")
        self.assertGreater(msg.width * msg.height, 0, "Failed: LiDAR point cloud contains 0 points!")
        self.assertGreater(len(msg.data), 0, "Failed: LiDAR binary data buffer is empty!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] LiDAR PointCloud detected with {msg.width * msg.height} points.')

    def test_02_imu_active(self):
        """Verification Phase 2: IMU Connectivity and Orientation"""
        self.node.get_logger().info('--- Verification Phase 2: IMU ---')
        msg = self.wait_for_topic(Imu, '/imu')
        self.assertIsNotNone(msg, "Failed: No Imu data on /imu")
        self.assertFalse(math.isnan(msg.linear_acceleration.z), "Failed: IMU linear acceleration Z is NaN!")
        self.assertGreater(abs(msg.linear_acceleration.z), 5.0, f"Failed: IMU acceleration Z ({msg.linear_acceleration.z}) did not detect gravity!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] IMU Detected. Linear Accel Z: {msg.linear_acceleration.z:.2f} m/s² (Gravity check)')

    def test_03_camera_active(self):
        """Verification Phase 3: Camera Stream Resolution"""
        self.node.get_logger().info('--- Verification Phase 3: Camera ---')
        msg = self.wait_for_topic(Image, '/front_cam', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No Image data on /front_cam")
        self.assertGreater(msg.width, 0, "Failed: Camera width is 0!")
        self.assertGreater(msg.height, 0, "Failed: Camera height is 0!")
        self.assertGreater(len(msg.data), 0, "Failed: Camera image data buffer is empty!")
        # Sample the image to verify it is not completely black (contains environment render data)
        has_non_zero = False
        for i in range(0, len(msg.data), 100):
            if msg.data[i] > 0:
                has_non_zero = True
                break
        self.assertTrue(has_non_zero, "Failed: Camera image is completely black (all zeros)!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Camera Online. Resolution: {msg.width}x{msg.height}')

    def test_04_odometry_active(self):
        """Verification Phase 4: Odometry Ground Truth Check"""
        self.node.get_logger().info('--- Verification Phase 4: Odometry ---')
        msg = self.wait_for_topic(Odometry, '/model/bgr/odometry', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No Odometry data on /model/bgr/odometry")
        self.assertFalse(math.isnan(msg.pose.pose.position.x), "Failed: Odometry position X is NaN!")
        self.assertFalse(math.isnan(msg.pose.pose.position.y), "Failed: Odometry position Y is NaN!")
        self.assertFalse(math.isnan(msg.pose.pose.orientation.w), "Failed: Odometry orientation W is NaN!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Odom Received. Initial Position: X={msg.pose.pose.position.x:.2f}, Y={msg.pose.pose.position.y:.2f}')

    def test_05_full_state_active(self):
        """Verification Phase 5: Car State Publisher"""
        self.node.get_logger().info('--- Verification Phase 5: Car State ---')
        msg = self.wait_for_topic(Float64MultiArray, '/robot/full_state', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No Float64MultiArray data on /robot/full_state")
        self.assertGreater(len(msg.data), 0, "Failed: Car state array is empty!")
        self.assertTrue(all(not math.isnan(x) for x in msg.data), "Failed: Car state contains NaN values!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Car State Received. Array Length: {len(msg.data)}')

    def test_06_wheels_status_active(self):
        """Verification Phase 6: Wheel Status Publisher"""
        self.node.get_logger().info('--- Verification Phase 6: Wheels Status ---')
        msg = self.wait_for_topic(Float64MultiArray, '/robot/wheels_status', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No Float64MultiArray data on /robot/wheels_status")
        self.assertGreater(len(msg.data), 0, "Failed: Wheels status array is empty!")
        self.assertTrue(all(not math.isnan(x) for x in msg.data), "Failed: Wheels status contains NaN values!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Wheels Status Received. Array Length: {len(msg.data)}')

    def test_07_noisy_sensors_active(self):
        """Verification Phase 7: Noisy Sensors Publisher"""
        self.node.get_logger().info('--- Verification Phase 7: Noisy Sensors ---')
        msg = self.wait_for_topic(Float64MultiArray, '/robot/noisy_state', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No Float64MultiArray data on /robot/noisy_state")
        self.assertGreater(len(msg.data), 0, "Failed: Noisy sensors array is empty!")
        self.assertTrue(all(not math.isnan(x) for x in msg.data), "Failed: Noisy sensors contains NaN values!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Noisy Sensors Received. Array Length: {len(msg.data)}')

    def test_08_visible_cones_active(self):
        """Verification Phase 8: Visible Cones Service & Publisher"""
        self.node.get_logger().info('--- Verification Phase 8: Visible Cones ---')
        msg = self.wait_for_topic(ConeArray, '/visible_cones', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No ConeArray data on /visible_cones")
        self.assertGreater(len(msg.cones), 0, "Failed: No visible cones detected from the spawn pose!")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Visible Cones Received. Count: {len(msg.cones)}')

    def test_09_car_path_tracking_and_collisions(self):
        """Verification Phase 9: Car Path Tracking and Cone Collisions"""
        self.node.get_logger().info('--- Verification Phase 9: Path Tracking & Cone Collisions ---')
        
        # Publishers to controllers
        pub_wheels = self.node.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        pub_steer = self.node.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        # Track path and collision history
        trajectory = []
        hit_cones = []
        
        def odom_callback(msg):
            pos = msg.pose.pose.position
            trajectory.append((pos.x, pos.y))
            
        def collided_callback(msg):
            for cone in msg.cones:
                if cone.id not in [c.id for c in hit_cones]:
                    hit_cones.append(cone)
                    
        sub_odom = self.node.create_subscription(Odometry, '/model/bgr/odometry', odom_callback, qos_profile_sensor_data)
        sub_collided = self.node.create_subscription(ConeArray, '/collided_cones', collided_callback, qos_profile_sensor_data)
        
        self.node.get_logger().info('🏎️  Executing steering path sequence...')
        
        # Wait for simulation clock to initialize (non-zero)
        while self.node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        start_sim_t = self.node.get_clock().now()
        
        # State machine definitions
        STATE_FORWARD = 0
        STATE_STOP_1 = 1
        STATE_BACKWARD = 2
        STATE_STOP_2 = 3
        STATE_CIRCLE = 4
        
        current_state = STATE_FORWARD
        state_start_sim_t = start_sim_t
        last_log_sim_t = 0.0
        
        # Execute for up to 50 seconds of simulation time
        elapsed = 0.0
        while elapsed < 50.0:
            current_sim_t = self.node.get_clock().now()
            elapsed = (current_sim_t - start_sim_t).nanoseconds / 1e9
            state_elapsed = (current_sim_t - state_start_sim_t).nanoseconds / 1e9
            
            # Default values
            speed = 0.0
            steer = 0.0
            
            # Read current position from odometry history
            curr_x = trajectory[-1][0] if trajectory else 0.0
            
            # State Machine transitions and control actions
            if current_state == STATE_FORWARD:
                speed = 8.0
                steer = 0.0
                # Transition to STOP_1 if we hit any orange blocker cone (x > 5.0)
                hit_orange = any(cone.color == 'orange' and cone.x > 5.0 for cone in hit_cones)
                if hit_orange:
                    self.node.get_logger().info("🏎️  Reached orange cone,Stopping...")
                    current_state = STATE_STOP_1
                    state_start_sim_t = current_sim_t
                    
            elif current_state == STATE_STOP_1:
                speed = 0.0
                steer = 0.0
                if state_elapsed >= 1.0:
                    self.node.get_logger().info("🏎️  Reversing back...")
                    current_state = STATE_BACKWARD
                    state_start_sim_t = current_sim_t
                    
            elif current_state == STATE_BACKWARD:
                speed = -8.0
                steer = 0.0
                # Transition to STOP_2 once we have backed up near the starting gate (X < 0.2)
                if curr_x < 1.1:
                    self.node.get_logger().info("🏎️  Reached target position, Stopping...")
                    current_state = STATE_STOP_2
                    state_start_sim_t = current_sim_t
                    
            elif current_state == STATE_STOP_2:
                speed = 0.0
                steer = 0.0
                if state_elapsed >= 1.0:
                    self.node.get_logger().info("🏎️ Following the right skidpad circle...")
                    current_state = STATE_CIRCLE
                    state_start_sim_t = current_sim_t
                    
            elif current_state == STATE_CIRCLE:
                speed = 8.0
                steer = -0.28  # Right circle steering angle (concentric with Skidpad centerline)
                if state_elapsed >= 32.0:
                    self.node.get_logger().info("🏎️  Circle completed. Completing test...")
                    break
                
            wheels_msg = Float64MultiArray()
            wheels_msg.data = [speed, speed, speed, speed]
            pub_wheels.publish(wheels_msg)
            steer_msg = Float64MultiArray()
            steer_msg.data = [steer]
            pub_steer.publish(steer_msg)

            # Periodic logging every 2.0s of simulation time to keep stdout active on slow runners
            if elapsed - last_log_sim_t >= 2.0:
                state_names = {
                    STATE_FORWARD: "FORWARD",
                    STATE_STOP_1: "STOP_1",
                    STATE_BACKWARD: "BACKWARD",
                    STATE_STOP_2: "STOP_2",
                    STATE_CIRCLE: "CIRCLE"
                }
                state_str = state_names.get(current_state, "UNKNOWN")
                curr_y = trajectory[-1][1] if trajectory else 0.0
                self.node.get_logger().info(
                    f"⏱️ [SIM TIME: {elapsed:.1f}s] State: {state_str} | Pos: ({curr_x:.2f}, {curr_y:.2f}) | "
                    f"Cmd Vel: {speed:.1f} m/s | Steer: {steer:.2f} rad | Collisions: {len(hit_cones)}"
                )
                last_log_sim_t = elapsed

            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        # Send stop commands to controllers before completing the test
        stop_wheels = Float64MultiArray()
        stop_wheels.data = [0.0, 0.0, 0.0, 0.0]
        pub_wheels.publish(stop_wheels)
        
        stop_steer = Float64MultiArray()
        stop_steer.data = [0.0]
        pub_steer.publish(stop_steer)
        
        # Spin once to ensure the stop messages are actually transmitted
        rclpy.spin_once(self.node, timeout_sec=0.2)
        
        self.node.destroy_publisher(pub_wheels)
        self.node.destroy_publisher(pub_steer)
        self.node.destroy_subscription(sub_odom)
        self.node.destroy_subscription(sub_collided)
        
        # Verify, report and plot results
        dist = self._report_trajectory_and_collisions(trajectory, hit_cones)
        self._generate_trajectory_plot(trajectory, hit_cones)
        
        self.assertGreater(dist, 0.5, f"Vehicle did not move enough! Traveled only {dist:.2f} meters.")

    def _report_trajectory_and_collisions(self, trajectory, hit_cones):
        """Helper to format and log trajectory coordinates and collision results."""
        self.assertTrue(len(trajectory) > 0, "No odometry history was recorded!")
        start_pos = trajectory[0]
        end_pos = trajectory[-1]
        dist = ((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)**0.5
        
        self.node.get_logger().info(f"📊 Traveled distance: {dist:.2f} meters")
        self.node.get_logger().info(f"🗺️  Start position: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) -> End position: ({end_pos[0]:.2f}, {end_pos[1]:.2f})")
        
        # Log path sample points
        step = max(1, len(trajectory) // 5)
        key_points = [trajectory[idx] for idx in range(0, len(trajectory), step)]
        if trajectory[-1] not in key_points:
            key_points.append(trajectory[-1])
        path_str = " -> ".join([f"({pt[0]:.2f}, {pt[1]:.2f})" for pt in key_points])
        self.node.get_logger().info(f"📍 Path History: {path_str}")
        
        # Log collided cones
        self.node.get_logger().info(f"🛑 Cone Collisions: {len(hit_cones)} cones hit.")
        for cone in hit_cones:
            self.node.get_logger().info(f"   - [HIT] Cone ID: {cone.id}, Color: {cone.color}, Pos: ({cone.x:.2f}, {cone.y:.2f})")
            
        return dist

    def _generate_trajectory_plot(self, trajectory, hit_cones):
        """Helper to fetch layout and generate visual matplotlib path plot."""
        try:
            import matplotlib
            matplotlib.use('Agg')  # Headless mode safe backend
            import matplotlib.pyplot as plt
            from bgr_description.srv import GetTrack

            all_cones = []
            track_client = self.node.create_client(GetTrack, 'get_track')
            if track_client.wait_for_service(timeout_sec=5.0):
                req = GetTrack.Request()
                req.track_name = 'SkidpadOpt.world'
                future = track_client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                if future.done() and future.result() and future.result().success:
                    all_cones = future.result().cones

            plt.figure(figsize=(10, 10))
            
            # Plot all map cones
            hit_ids = {c.id for c in hit_cones}
            for cone in all_cones:
                if cone.id in hit_ids:
                    plt.scatter(cone.x, cone.y, c='red', marker='x', s=80, zorder=5)
                else:
                    color_map = {'blue': 'blue', 'yellow': 'gold', 'orange': 'orange', 'orange_big': 'orange'}
                    plot_color = color_map.get(cone.color.lower(), 'gray')
                    plt.scatter(cone.x, cone.y, c=plot_color, marker='^', s=40, zorder=4)

            # Plot vehicle trajectory
            xs, ys = zip(*trajectory)
            plt.plot(xs, ys, 'k--', linewidth=1.5, label='Car Trajectory', zorder=2)
            plt.scatter(xs[0], ys[0], c='green', marker='o', s=100, label='Start', zorder=3)
            # Plot the car emoji at the final position
            plt.text(xs[-1], ys[-1], '🏎️', fontsize=24, ha='center', va='center', zorder=5)
            plt.scatter(xs[-1], ys[-1], alpha=0, label='Car (End)')

            # Legend placeholders
            if hit_cones:
                plt.scatter([], [], c='red', marker='x', s=80, label='Hit Cone')
            plt.scatter([], [], c='blue', marker='^', s=40, label='Blue Cone')
            plt.scatter([], [], c='gold', marker='^', s=40, label='Yellow Cone')

            plt.title("BGR Simulation Trajectory & Cone Collisions")
            plt.xlabel("X Position (m)")
            plt.ylabel("Y Position (m)")
            plt.grid(True, linestyle=':', alpha=0.6)
            plt.axis('equal')
            plt.legend(loc='best')
            
            plot_path = os.path.abspath('output_result.png')
            plt.savefig(plot_path, dpi=150)
            plt.close()
            self.node.get_logger().info(f"💾 Trajectory plot saved successfully to: {plot_path}")
        except Exception as e:
            self.node.get_logger().error(f"⚠️ Failed to generate visual trajectory plot: {e}")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        print("\n[Post-Shutdown] Checking process exit codes...")
        try:
            launch_testing.asserts.assertExitCodes(
                proc_info,
                allowable_exit_codes=[0, 2, 1, -2, -9, -15, 255, 130, 137]
            )
            print("[Post-Shutdown] ✅ Exit codes are within allowable range.")
        finally:
            print("[Post-Shutdown] Running post-shutdown nuclear cleanup of any leftover processes...")
            commands = [
                'ros2 daemon stop 2>/dev/null || true',
                'killall -9 ros2 ros2-daemon gz ruby rviz2 parameter_bridge 2>/dev/null || true',
                'killall -9 gz-sim-server gz-sim-gui ign-gazebo-server ign-gazebo-gui 2>/dev/null || true',
                'killall -9 robot_state_publisher static_transform_publisher ros2_control_node 2>/dev/null || true',
                'pkill -9 -f "car_state_publisher.py" 2>/dev/null || true',
                'pkill -9 -f "car_wheel_publisher.py" 2>/dev/null || true',
                'pkill -9 -f "cone_service.py" 2>/dev/null || true',
                'pkill -9 -f "visible_cones.py" 2>/dev/null || true',
                'pkill -9 -f "noisy_sensor_publisher.py" 2>/dev/null || true',
                'pkill -9 -f "bgr_controller" 2>/dev/null || true',
                'pkill -9 -f "controller_manager/spawner" 2>/dev/null || true',
                'pkill -9 -f "car_dashboard.py" 2>/dev/null || true',
                'pkill -9 -f "keyboard_teleop" 2>/dev/null || true',
                'rm -rf /tmp/ignition_* /tmp/gz_* /tmp/gazebo_* /dev/shm/rtps* /dev/shm/fastdds* || true'
            ]
            for cmd in commands:
                subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[Post-Shutdown] ✅ Cleanup complete.")
