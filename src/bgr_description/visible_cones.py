#!/usr/bin/env python3
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
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        
        # 2. State Variables
        self.all_cones = []
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        
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

        # 6. Publisher (Visible Cones)
        self.cone_pub = self.create_publisher(
            ConeArray,
            'visible_cones',
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
        """Updates internal car position and yaw."""
        if len(msg.data) >= 6:
            self.car_x = msg.data[0]
            self.car_y = msg.data[1]
            self.car_yaw = msg.data[5]

    def timer_callback(self):
        """Calculates and publishes visible cones at 5Hz."""
        if not self.all_cones:
            return

        visible_msg = ConeArray()
        
        # Vision parameters
        DEPTH = 30.0   
        WIDTH = 6.0    
        HALF_WIDTH = WIDTH / 2.0

        for cone in self.all_cones:
            # 1. Translate
            dx = cone.x - self.car_x
            dy = cone.y - self.car_y
            
            # 2. Rotate into Car's Local Frame
            local_x = dx * math.cos(self.car_yaw) + dy * math.sin(self.car_yaw)
            local_y = -dx * math.sin(self.car_yaw) + dy * math.cos(self.car_yaw)
            
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
