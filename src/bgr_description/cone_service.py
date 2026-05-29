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
        
        # Cache parsed results so repeated calls skip disk I/O
        self._cache: dict = {}
        
        self.get_logger().info(f'Cone Service Ready. Reading worlds from: {self.world_base_dir}')

    @staticmethod
    def _color_from_text(text: str) -> str:
        """Determine cone color from a URI or visual name string."""
        t = text.lower()
        if 'cone_orange_big' in t:
            return 'orange_big'
        if 'cone_orange' in t:
            return 'orange'
        if 'cone_yellow' in t:
            return 'yellow'
        if 'cone_blue' in t:
            return 'blue'
        return 'unknown'

    @staticmethod
    def _parse_xy(pose_tag) -> tuple:
        """Return (x, y) from an SDF <pose> tag, or (0.0, 0.0) on failure."""
        if pose_tag is not None and pose_tag.text:
            parts = pose_tag.text.split()
            if len(parts) >= 2:
                return float(parts[0]), float(parts[1])
        return 0.0, 0.0

    def track_callback(self, request, response):
        """
        Triggered when a client calls: ros2 service call /get_track ...
        Request: track_name (string)
        Response: cones (Cone[]), success (bool), message (string)
        """
        requested_name = request.track_name
        
        # Add .world extension if the user didn't provide it
        filename = requested_name if requested_name.endswith('.world') else f"{requested_name}.world"
        full_path = os.path.join(self.world_base_dir, filename)
        
        self.get_logger().info(f"Client requested world data for track: {requested_name}")

        if not os.path.exists(full_path):
            response.success = False
            response.message = f"World file not found: {full_path}"
            self.get_logger().warn(response.message)
            return response

        # Serve from cache if available
        if filename in self._cache:
            response.cones = self._cache[filename]
            response.success = True
            response.message = f"Successfully loaded {len(response.cones)} cones from {filename} (cached)"
            self.get_logger().info(response.message)
            return response

        try:
            loaded_cones = []
            
            tree = ET.parse(full_path)
            root = tree.getroot()
            
            world_tag = root.find('world')
            if world_tag is None:
                response.success = False
                response.message = f"Invalid SDF: No <world> tag found in {filename}"
                return response

            # 1. Parse original include-style world files
            for include in world_tag.findall('include'):
                uri_tag  = include.find('uri')
                name_tag = include.find('name')
                pose_tag = include.find('pose')
                
                if uri_tag is None or 'cone' not in uri_tag.text.lower():
                    continue

                c = Cone()
                c.id    = name_tag.text if name_tag is not None else "unknown_cone"
                c.color = self._color_from_text(uri_tag.text)
                c.x, c.y = self._parse_xy(pose_tag)
                loaded_cones.append(c)

            # 2. Parse optimized single-link model style world files (e.g. SkidpadOpt.world)
            for model_tag in world_tag.findall('model'):
                for link_tag in model_tag.findall('link'):
                    for visual_tag in link_tag.findall('visual'):
                        vis_name  = visual_tag.get('name', '')
                        geom_tag  = visual_tag.find('geometry')
                        mesh_tag  = geom_tag.find('mesh') if geom_tag is not None else None
                        uri_tag   = mesh_tag.find('uri') if mesh_tag is not None else None
                        uri_text  = uri_tag.text if uri_tag is not None and uri_tag.text else ''

                        if 'cone' not in uri_text.lower() and 'cone' not in vis_name.lower():
                            continue

                        c = Cone()
                        # Strip leading "visual_" prefix from the ID
                        c.id    = vis_name[7:] if vis_name.startswith('visual_') else vis_name
                        c.color = self._color_from_text(uri_text or vis_name)
                        c.x, c.y = self._parse_xy(visual_tag.find('pose'))
                        loaded_cones.append(c)

            self._cache[filename] = loaded_cones
            response.cones   = loaded_cones
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