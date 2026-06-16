#!/usr/bin/env python3
"""
Visible Cones & Collision Detector Node
=======================================

This node serves two distinct, optimized purposes for the BGR Simulator:
1. Collision Tracking (20Hz): Actively listens to the vehicle state
   and performs an Oriented Bounding Box (OBB) intersection to detect if the car physically
   overlaps with any track cones. This enables cones to be "ghosts" in Gazebo while still
   accurately tracking hitting penalties using a debounce mechanism.
2. Field of View (FOV) Simulation (5Hz): Evaluates which cones fall within a 30m x 6m 
   rectangular viewing frustum in front of the car, publishing a continuous stream of
   visible cones to simulate a camera/perception system without overloading the CPU.

Published Topics:
---------------------------------------------------------------------------------------------------------------------
Topic                       | Rate   | Message Type                            | Data Description
---------------------------------------------------------------------------------------------------------------------
visible_cones               | 5 Hz   | bgr_description/msg/ConeArray           | Cones currently in the camera's FOV (front frustum)
collided_cones              | 20 Hz  | bgr_description/msg/ConeArray           | List of all cones that have been hit so far
cone_collision              | Event  | bgr_description/msg/Cone                | Published once at the exact moment a cone is hit
/collision/count            | 20 Hz  | std_msgs/msg/Float64                    | Cumulative count of all cone collisions
/collision/markers          | 20 Hz  | visualization_msgs/msg/MarkerArray      | Red sphere markers at collision sites for Foxglove/RViz
---------------------------------------------------------------------------------------------------------------------
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from bgr_description.srv import GetTrack
from bgr_description.msg import Cone, ConeArray

class VisibleConesNode(Node):
    def __init__(self):
        super().__init__('visible_cones_node')
        
        # 1. Parameters (matching URDF body size and offset)
        self.declare_parameter('world_name', 'Map1Opt')
        self.declare_parameter('car_length', 3.14)
        self.declare_parameter('car_width', 1.43)
        self.declare_parameter('car_x_offset', 0.074)
        self.declare_parameter('cone_radius', 0.115)

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.car_length = self.get_parameter('car_length').get_parameter_value().double_value
        self.car_width = self.get_parameter('car_width').get_parameter_value().double_value
        self.car_x_offset = self.get_parameter('car_x_offset').get_parameter_value().double_value
        self.cone_radius = self.get_parameter('cone_radius').get_parameter_value().double_value
        
        # 2. State Variables
        self.all_cones = []
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.currently_overlapping_cones = set()
        self.all_hit_events = []
        self.last_collision_check_time = 0.0
        
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

        # Foxglove-facing collision visualization
        self.collision_count_pub = self.create_publisher(Float64, '/collision/count', 10)
        self.collision_marker_pub = self.create_publisher(MarkerArray, '/collision/markers', 10)

        # 7. Timer (5Hz calculation)
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        self.get_logger().info(f'Visible Cones Node Started (5Hz). World: {self.world_name}')

        # Seed the count so Foxglove shows 0 before the first hit.
        self._publish_collision_count()

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

            # Throttle collision checking to 20 Hz using simulation time
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_collision_check_time >= 0.05:
                self.last_collision_check_time = now
                self.check_collisions()

    def check_collisions(self):
        """Performs precise OBB intersection to detect if the car hits any ghost cone."""
        if not self.all_cones:
            return

        # Fast broad-phase squared radius (max possible distance from axle midpoint to any point on the collision box)
        max_x = (self.car_length / 2.0) + abs(self.car_x_offset) + self.cone_radius
        max_y = (self.car_width / 2.0) + self.cone_radius
        broad_radius_sq = max_x**2 + max_y**2
        
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

            # Narrow phase: precise OBB intersection relative to offset box center
            local_x = (dx * cos_yaw + dy * sin_yaw) - self.car_x_offset
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
            self._publish_collision_count()
            self._publish_collision_markers()

    def _publish_collision_count(self):
        msg = Float64()
        msg.data = float(len(self.all_hit_events))
        self.collision_count_pub.publish(msg)

    def _publish_collision_markers(self):
        marker_array = MarkerArray()
        for i, cone in enumerate(self.all_hit_events):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'collisions'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(cone.x)
            marker.pose.position.y = float(cone.y)
            marker.pose.position.z = 0.3
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 0.6
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            marker_array.markers.append(marker)
        self.collision_marker_pub.publish(marker_array)

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
