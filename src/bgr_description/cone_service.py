#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

# --------------------------------
# NOTE: To use, start gazebo.launch.py first, and enter:
# ros2 service call /get_track bgr_description/srv/GetTrack "{track_name: 'CompetitionMap1'}"
# --------------------------------

# Imports the necessary srv and msg files.
from bgr_description.srv import GetTrack
from bgr_description.msg import Cone

class ConeService(Node):
    def __init__(self):
        super().__init__('cone_service')
        
        # Service Name: '/get_track'
        self.srv = self.create_service(GetTrack, 'get_track', self.track_callback)
        
        # The Worlds are loaded relative to the dynamically found share directory
        bgr_dir = get_package_share_directory('bgr_description')
        self.world_base_dir = os.path.join(bgr_dir, 'worlds')
        
        self.get_logger().info(f'Cone Service Ready. Reading worlds from: {self.world_base_dir}')

    def track_callback(self, request, response):
        """
        Triggered when a client calls: ros2 service call /get_track ...
        Request: track_name (string)
        Response: cones (Cone[]), success (bool), message (string)
        """
        requested_name = request.track_name
        
        # Add .world extension if the user didn't provide it
        if not requested_name.endswith('.world'):
            filename = f"{requested_name}.world"
        else:
            filename = requested_name

        full_path = os.path.join(self.world_base_dir, filename)
        
        self.get_logger().info(f"Client requested world data for track: {requested_name}")

        if not os.path.exists(full_path):
            response.success = False
            response.message = f"World file not found: {full_path}"
            self.get_logger().warn(response.message)
            return response

        try:
            loaded_cones = []
            
            # Parse the Gazebo World XML (SDF)
            tree = ET.parse(full_path)
            root = tree.getroot()
            
            # Find the <world> tag
            world_tag = root.find('world')
            if world_tag is None:
                response.success = False
                response.message = f"Invalid SDF: No <world> tag found in {filename}"
                return response

            # Iterate through all <include> models
            for include in world_tag.findall('include'):
                uri_tag = include.find('uri')
                name_tag = include.find('name')
                pose_tag = include.find('pose')
                
                if uri_tag is not None and 'cone' in uri_tag.text.lower():
                    c = Cone()
                    
                    # 1. Set ID from the name tag
                    c.id = name_tag.text if name_tag is not None else "unknown_cone"
                    
                    # 2. Extract Color from URI (model://cone_yellow -> yellow)
                    uri = uri_tag.text
                    if 'cone_' in uri.lower():
                        c.color = uri.lower().split('cone_')[-1]
                    else:
                        c.color = "unknown"

                    # 3. Parse Pose (x y z roll pitch yaw)
                    if pose_tag is not None:
                        pose_parts = pose_tag.text.split()
                        if len(pose_parts) >= 2:
                            c.x = float(pose_parts[0])
                            c.y = float(pose_parts[1])
                        else:
                            c.x = 0.0
                            c.y = 0.0
                    
                    loaded_cones.append(c)

            response.cones = loaded_cones
            response.success = True
            response.message = f"Successfully loaded {len(loaded_cones)} cones from {filename}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error parsing World file: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ConeService()
    
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