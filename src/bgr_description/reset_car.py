#!/usr/bin/env python3
"""
Script to reset vehicle's position back to the origin (0,0,1.0) in Gazebo.
"""
# While running the simulator, to execute this script:
#   - Open a new terminal 
#   - Source the environment: source install/setup.bash
#   - Run the script: ros2 run bgr_description reset_car.py
# NOTE: The vehicle will remain in motion when calling this script.
#       Make sure the vehicle is idle before resetting its position.
import subprocess


def main():
    # Gazebo service command to reset the pose
    # Gazebo Harmonic/Ignition uses 'gz service'
    cmd = [
        "gz", "service", "-s", "/world/generated_world/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", 'name: "bgr", position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}'
    ]
    
    print("[RESET] Teleporting vehicle back to (0.0, 0.0, 1.0)...")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        # Gazebo service returns 0 if successful
        if result.returncode == 0 and "data: true" in result.stdout.lower():
            print("[RESET] Success! Vehicle snapped back to starting line.")
        else:
            print("[RESET] Failed to reset vehicle. Is Gazebo running and the world fully loaded?")
            if result.stderr:
                print(f"Error: {result.stderr.strip()}")
            if result.stdout and "data: true" not in result.stdout.lower():
                print(f"Output: {result.stdout.strip()}")
    except Exception as e:
        print(f"[RESET] Execution error: {str(e)}")

if __name__ == '__main__':
    main()
