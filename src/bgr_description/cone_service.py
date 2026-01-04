#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import os

# --------------------------------
# NOTE: To use, start gazebo.launch.py first, and enter:
# ros2 service call /get_track bgr_description/srv/GetTrack "{track_name: '<insert_track_name>'}"
# --------------------------------

# Imports the necessary srv and msg files.
from bgr_description.srv import GetTrack
from bgr_description.msg import Cone

class ConeService(Node):
    def __init__(self):
        super().__init__('cone_service')
        
        # Service Name: '/get_track'
        self.srv = self.create_service(GetTrack, 'get_track', self.track_callback)
        
        # NOTE: Update this path to your adjusted location
        self.csv_base_dir = os.path.expanduser(
            "~/BGR_Simulator/BGR_Simulator/src/TracksV0/tracks/csv"
        )
        
        self.get_logger().info(f'Cone Service Ready. Reading maps from: {self.csv_base_dir}')

    def track_callback(self, request, response):
        """
        Triggered when a client calls: ros2 service call /get_track ...
        Request: track_name (string)
        Response: cones (Cone[]), success (bool), message (string)
        """
        requested_name = request.track_name
        
        # Add .csv extension if the user didn't provide it
        if not requested_name.endswith('.csv'):
            filename = f"{requested_name}.csv"
        else:
            filename = requested_name

        full_path = os.path.join(self.csv_base_dir, filename)
        
        self.get_logger().info(f"Client requested track: {requested_name}")

        if not os.path.exists(full_path):
            response.success = False
            response.message = f"File not found: {full_path}"
            self.get_logger().warn(response.message)
            return response

        try:
            loaded_cones = []
            with open(full_path, 'r') as f:
                reader = csv.DictReader(f)
                
                # Verify CSV headers exist
                if not {'x_m', 'y_m', 'color_id'}.issubset(reader.fieldnames):
                     response.success = False
                     response.message = f"CSV missing required columns (x_m, y_m, color_id). Found: {reader.fieldnames}"
                     return response

                for row in reader:
                    c = Cone()
                    # Parse x, y positions
                    c.x = float(row['x_m'])
                    c.y = float(row['y_m'])
                    
                    # Parse color ID
                    # (1=Yellow, 2=Blue, 3=Orange 4=Orange_Big)
                    c.color = int(row['color_id'])
                    
                    loaded_cones.append(c)

            response.cones = loaded_cones
            response.success = True
            response.message = f"Successfully loaded {len(loaded_cones)} cones from {filename}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error parsing CSV: {str(e)}"
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()