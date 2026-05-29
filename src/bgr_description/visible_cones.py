#!/usr/bin/env python3
"""
Visible Cones & Collision Detector Node

This node serves two distinct, optimized purposes for the BGR Simulator:
1. Collision Tracking (50Hz+): Actively listens to the high-frequency vehicle odometry state
   and performs an Oriented Bounding Box (OBB) intersection to detect if the car physically
   overlaps with any track cones. This enables cones to be "ghosts" in Gazebo while still
   accurately tracking hitting penalties using a debounce mechanism.
2. Field of View (FOV) Simulation (5Hz): Evaluates which cones fall within a 30m x 6m 
   rectangular viewing frustum in front of the car, publishing a continuous stream of
   visible cones to simulate a camera/perception system without overloading the CPU.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from bgr_description.srv import GetTrack
from bgr_description.msg import Cone, ConeArray

class VisibleConesNode(Node):
    def __init__(self):
        super().__init__('visible_cones_node')
        
        # 1. Parameters
        self.declare_parameter('world_name', 'CompetitionMap1')
        self.declare_parameter('car_length', 2.55)
        self.declare_parameter('car_width', 1.45)
        self.declare_parameter('cone_radius', 0.15)

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.car_length = self.get_parameter('car_length').get_parameter_value().double_value
        self.car_width = self.get_parameter('car_width').get_parameter_value().double_value
        self.cone_radius = self.get_parameter('cone_radius').get_parameter_value().double_value
        
        # 2. State Variables
        self.all_cones = []
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.currently_overlapping_cones = set()
        self.all_hit_events = []
        
        # 3. QoS Configuration (Matching Gazebo/Publisher)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 4. Service Client (Fetch track once)
        self.client = self.create_client(GetTrack, 'get_track')
        self.refresh_track_data()

        # 5. Subscriber (Car Pose)
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/robot/full_state',
            self.car_state_callback,
            qos_profile)

        # 6. Publishers (Visible & Hit Cones)
        self.cone_pub = self.create_publisher(
            ConeArray,
            'visible_cones',
            10)
        self.cone_hit_pub = self.create_publisher(
            Cone,
            'cone_collision',
            10)
        self.collided_cones_pub = self.create_publisher(
            ConeArray,
            'collided_cones',
            10)

        # 7. Timer (5Hz calculation)
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        self.get_logger().info(f'Visible Cones Node Started (5Hz). World: {self.world_name}')

    def refresh_track_data(self):
        """Calls the /get_track service to populate the local cache of cones."""
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /get_track not available! Make sure cone_service.py is running.")
            return

        request = GetTrack.Request()
        request.track_name = self.world_name
        
        future = self.client.call_async(request)
        # In a node, we wait for the callback rather than spinning synchronously
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response and response.success:
                self.all_cones = response.cones
                self.get_logger().info(f"Successfully cached {len(self.all_cones)} cones.")
            else:
                self.get_logger().error(f"Failed to fetch track: {response.message if response else 'No response'}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def car_state_callback(self, msg):
        """Updates internal car position and yaw, and checks for collisions."""
        if len(msg.data) >= 6:
            self.car_x = msg.data[0]
            self.car_y = msg.data[1]
            self.car_yaw = msg.data[5]

            # Run high-frequency OBB collision check
            self.check_collisions()

    def check_collisions(self):
        """Performs precise OBB intersection to detect if the car hits any ghost cone."""
        if not self.all_cones:
            return

        # Fast broad-phase squared radius (max possible distance for a corner hit)
        broad_radius_sq = ((self.car_length / 2.0) + self.cone_radius) ** 2 + ((self.car_width / 2.0) + self.cone_radius) ** 2
        
        new_hit = False
        current_frame_overlaps = set()
        
        cos_yaw = math.cos(self.car_yaw)
        sin_yaw = math.sin(self.car_yaw)

        for cone in self.all_cones:
            dx = cone.x - self.car_x
            dy = cone.y - self.car_y
            
            # Broad phase: fast squared distance check
            if (dx*dx + dy*dy) > broad_radius_sq:
                continue

            # Narrow phase: precise OBB intersection
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw

            if (abs(local_x) <= (self.car_length / 2.0) + self.cone_radius) and \
               (abs(local_y) <= (self.car_width / 2.0) + self.cone_radius):
                
                current_frame_overlaps.add(cone.id)
                
                # If this is a NEW overlap (not already overlapping from previous frame)
                if cone.id not in self.currently_overlapping_cones:
                    # self.get_logger().warn(f"[COLLISION] Ghost hit detected! Cone: {cone.id} ({cone.color}) at x:{cone.x:.2f}, y:{cone.y:.2f}")
                    
                    # Register hit event
                    self.all_hit_events.append(cone)
                    self.cone_hit_pub.publish(cone)
                    new_hit = True

        # Update the debounce set: remove cones we are no longer touching
        self.currently_overlapping_cones = current_frame_overlaps

        # Publish the full updated list of collided cones if there was a new hit
        if new_hit:
            collided_msg = ConeArray()
            collided_msg.cones = self.all_hit_events
            self.collided_cones_pub.publish(collided_msg)

    def timer_callback(self):
        """Calculates and publishes visible cones at 5Hz."""
        if not self.all_cones:
            return

        visible_msg = ConeArray()
        DEPTH = 30.0   
        WIDTH = 6.0    
        HALF_WIDTH = WIDTH / 2.0
        
        cos_yaw = math.cos(self.car_yaw)
        sin_yaw = math.sin(self.car_yaw)

        for cone in self.all_cones:
            # 1. Translate
            dx = cone.x - self.car_x
            dy = cone.y - self.car_y
            
            # 2. Rotate into Car's Local Frame
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw
            
            # 3. Check bounds
            if 0.0 < local_x < DEPTH and -HALF_WIDTH < local_y < HALF_WIDTH:
                visible_msg.cones.append(cone)
                
        # 4. Publish
        self.cone_pub.publish(visible_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisibleConesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
