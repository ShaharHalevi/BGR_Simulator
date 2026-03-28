#!/usr/bin/env python3
"""
Batch CSV -> Gazebo WORLD generator (NORMALIZED & ORGANIZED)
1. Scans for .csv files.
2. Centers the track start line to (0,0).
3. Saves all .world files into a 'worlds' subfolder.
"""

from __future__ import annotations
import os
import csv
from pathlib import Path
from typing import Dict, List, Tuple

# --- CONFIGURATION ---
URI_MAP = {
    1: "model://cone_yellow",
    2: "model://cone_blue",
    3: "model://cone_orange",
    4: "model://cone_orange_big",
}

WORLD_HEADER = """<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="generated_world">
    <scene>
      <sky><clouds><speed>12</speed></clouds></sky>
      <grid>false</grid>
      <shadows>1</shadows>
    </scene>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <!-- Required Gazebo system plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -1</direction>
      <attenuation>
        <range>300</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>true</cast_shadows>
    </light>
    <model name='asphalt_ground'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

WORLD_FOOTER = """
  </world>
</sdf>
"""

def parse_csv(csv_path: Path) -> List[Tuple[int, str, float, float]]:
    cones = []
    with csv_path.open("r", newline="", encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames:
            reader.fieldnames = [name.strip() for name in reader.fieldnames]

        for row in reader:
            try:
                c_id = int(row.get("color_id", 0))
                c_name = str(row.get("color_name", "Unknown"))
                
                # Logic to guess ID if missing
                if c_id == 0:
                    if "yellow" in c_name.lower(): c_id = 1
                    elif "blue" in c_name.lower(): c_id = 2
                    elif "big" in c_name.lower(): c_id = 4
                    elif "orange" in c_name.lower(): c_id = 3

                x = float(row["x_m"])
                y = float(row["y_m"])
                cones.append((c_id, c_name, x, y))
            except:
                pass 
    return cones

def calculate_offset(cones: List[Tuple]) -> Tuple[float, float]:
    """
    Finds the 'Start Line' to determine the (0,0) center point.
    Logic: Finds the first two ORANGE cones (ID 3 or 4) and takes their midpoint.
    """
    orange_cones = [c for c in cones if c[0] in [3, 4]]
    
    if len(orange_cones) >= 2:
        # Take the first two orange cones (Start Gate)
        c1 = orange_cones[0]
        c2 = orange_cones[1]
        mid_x = (c1[2] + c2[2]) / 2
        mid_y = (c1[3] + c2[3]) / 2
        print(f"   [OFFSET] Found Start Line. Centering world at: {mid_x:.2f}, {mid_y:.2f}")
        return mid_x, mid_y
    elif len(cones) > 0:
        # Fallback
        print(f"   [OFFSET] No orange cones. Centering on first cone.")
        return cones[0][2], cones[0][3]
    return 0.0, 0.0

def get_cone_xml(uri: str, name: str, x: float, y: float) -> str:
    return f"""    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
    </include>
"""

def process_file(csv_file: Path, output_dir: Path):
    print(f"Processing {csv_file.name}...")
    cones = parse_csv(csv_file)
    
    if not cones:
        print("   [SKIP] No data found.")
        return

    # Calculate Offset
    off_x, off_y = calculate_offset(cones)
    
    xml_content = WORLD_HEADER
    counts = {}
    
    for c_id, _, x, y in cones:
        if c_id not in URI_MAP: continue
        
        # Apply Offset
        final_x = x - off_x
        final_y = y - off_y
        
        counts[c_id] = counts.get(c_id, 0) + 1
        uri = URI_MAP[c_id]
        
        if c_id == 1: base = "cone_yellow"
        elif c_id == 2: base = "cone_blue"
        elif c_id == 3: base = "cone_orange"
        elif c_id == 4: base = "cone_orange_big"
        else: base = f"cone_{c_id}"
        
        name = f"{base}_{counts[c_id]:03d}"
        xml_content += get_cone_xml(uri, name, final_x, final_y)
        
    xml_content += WORLD_FOOTER
    
    # Save into the 'worlds' folder
    out_file = output_dir / csv_file.with_suffix(".world").name
    out_file.write_text(xml_content, encoding="utf-8")
    print(f" -> Saved to: {out_file}")

def main():
    current_dir = Path(".")
    
    # Create 'worlds' folder if it doesn't exist
    output_dir = current_dir / "worlds"
    output_dir.mkdir(exist_ok=True)
    
    csv_files = list(current_dir.glob("*.csv"))
    
    if not csv_files:
        print("No CSV files found!")
        return

    print(f"Found {len(csv_files)} CSV files. Generating worlds...")
    for f in csv_files:
        process_file(f, output_dir)
    print("Done!")

if __name__ == "__main__":
    main()