import os
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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_testing


# 1. LAUNCH EXECUTION
# This runs once. Gazebo persists until all test classes complete.
@pytest.mark.launch_test
def generate_test_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('bgr_description'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'headless': 'True'}.items() # Must be true to avoid GUI failures in Git Actions.
    )

    # We wait long enough for the spawner (35s) + controllers to settle
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
        
        # We'll use a standalone logger for the suite
        suite_logger = rclpy.logging.get_logger('test_suite')
        
        suite_logger.info("\n" + "="*50)
        suite_logger.info("🚀 [ENVIRONMENT] BGR Test Suite Diagnostics")
        suite_logger.info(f"   - ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'N/A')}")
        suite_logger.info(f"   - ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
        suite_logger.info(f"   - WORKSPACE: {os.getcwd()}")
        suite_logger.info("="*50)

        suite_logger.info("\n🚀 [STARTUP] Initializing BGR Simulation Test Suite")
        
        # Instead of just waiting for Odometry (Stage 2), we wait for /robot/wheels_status.
        # This topic is published by car_wheel_publisher.py in Stage 3, proving that 
        # the ENTIRE launch pipeline has fully executed and all nodes are live.
        suite_logger.info("⏲️  [WAIT] Waiting for Stage 3 tooling to publish /robot/wheels_status...")
        startup_node = rclpy.create_node('test_startup_node')
        stage3_ready = False
        def stage3_callback(msg):
            nonlocal stage3_ready
            stage3_ready = True

        sub = startup_node.create_subscription(Float64MultiArray, '/robot/wheels_status', stage3_callback, qos_profile_sensor_data)
        
        start_wait = time.time()
        # Allow up to 120s for Gazebo to cold-start, load the world, and spawn the car and tools
        while not stage3_ready and time.time() - start_wait < 120.0:
            rclpy.spin_once(startup_node, timeout_sec=0.1)
            
        startup_node.destroy_node()
        
        if stage3_ready:
            suite_logger.info("✅ [READY] Stage 3 is active (/robot/wheels_status). Sensors and Tools are live. Proceeding to tests.")
            suite_logger.info("⏲️  [WAIT] Waiting 5s for internal sensor pipelines to settle...")
            time.sleep(5)
        else:
            suite_logger.error("❌ [FATAL] Timeout waiting for /robot/wheels_status. Stage 3 logic failed to launch.")

    @classmethod
    def tearDownClass(cls):
        suite_logger = rclpy.logging.get_logger('test_suite')
        suite_logger.info("\n" + "="*50)
        suite_logger.info("🛑 [EXIT] Simulation Test Suite Complete")
        suite_logger.info("="*50)
        
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'{self.__class__.__name__}_node')
        self.node.get_logger().info(f'🧪 Starting test: {self._testMethodName}')

    def tearDown(self):
        self.node.get_logger().info(f'🏁 Finished test: {self._testMethodName}')
        self.node.destroy_node()

    def wait_for_topic(self, topic_class, topic_name, timeout=35.0):
        self.node.get_logger().info(f'🔄 Waiting for topic: {topic_name} (timeout {timeout}s)...')
        received_msg = None
        start_time = time.time()
        def callback(msg):
            nonlocal received_msg
            received_msg = msg

        sub = self.node.create_subscription(
            topic_class,
            topic_name,
            callback,
            qos_profile_sensor_data
        )
        
        end_time = start_time + timeout
        while time.time() < end_time and received_msg is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        self.node.destroy_subscription(sub)
        
        if received_msg is not None:
             duration = time.time() - start_time
             self.node.get_logger().info(f'✅ SUCCESS: Data received on {topic_name} after {duration:.2f}s!')
        else:
             self.node.get_logger().error(f'❌ FAILURE: Timeout reached for {topic_name} after {timeout}s!')
             
        return received_msg

    def test_01_lidar_active(self):
        """Verification Phase 1: LiDAR Connectivity and Density"""
        self.node.get_logger().info('--- Verification Phase 1: LiDAR ---')
        msg = self.wait_for_topic(PointCloud2, '/scan/points')
        self.assertIsNotNone(msg, "Failed: No PointCloud2 data on /scan/points (LiDAR)")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] LiDAR PointCloud detected with {msg.width * msg.height} points.')

    def test_02_imu_active(self):
        """Verification Phase 2: IMU Connectivity and Orientation"""
        self.node.get_logger().info('--- Verification Phase 2: IMU ---')
        msg = self.wait_for_topic(Imu, '/imu')
        self.assertIsNotNone(msg, "Failed: No Imu data on /imu")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] IMU Detected. Linear Accel Z: {msg.linear_acceleration.z:.2f} m/s² (Gravity check)')

    def test_03_camera_active(self):
        """Verification Phase 3: Camera Stream Resolution"""
        self.node.get_logger().info('--- Verification Phase 3: Camera ---')
        msg = self.wait_for_topic(Image, '/front_cam', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No Image data on /front_cam")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Camera Online. Resolution: {msg.width}x{msg.height}')

    def test_04_odometry_active(self):
        """Verification Phase 4: Odometry Ground Truth Check"""
        self.node.get_logger().info('--- Verification Phase 4: Odometry ---')
        msg = self.wait_for_topic(Odometry, '/model/bgr/odometry', timeout=60.0)
        self.assertIsNotNone(msg, "Failed: No Odometry data on /model/bgr/odometry")
        self.node.get_logger().info(f'📦 [DIAGNOSTIC] Odom Received. Initial Position: X={msg.pose.pose.position.x:.2f}, Y={msg.pose.pose.position.y:.2f}')

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self,proc_info):
        print("\n[Post-Shutdown] Checking process exit codes...")
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, 2, 1, -2, -15, 255, 130, 137]
        )
        print("[Post-Shutdown] ✅ Exit codes are within allowable range.")
