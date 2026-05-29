import pandas as pd
import os

# שמות המודלים כמו שהגזיבו מכיר אותם
MODELS = {
    'Yellow': 'model://cone_yellow',
    'Blue': 'model://cone_blue',
    'Orange': 'model://cone_orange'
}

def create_sdf(csv_file, output_file):
    print(f"Reading {csv_file}...")
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: Could not find file {csv_file}")
        return

    sdf_content = """<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='track_generated'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry><plane><normal>0 0 1</normal><size>0.1 0.1</size></plane></geometry>
      </visual>
    </link>
"""
    
    count = 0
    for i, row in df.iterrows():
        color = row['color_name']
        if color not in MODELS: continue
        
        sdf_content += f"""
    <include>
      <uri>{MODELS[color]}</uri>
      <name>cone_{i}_{color}</name>
      <pose>{row['x_m']} {row['y_m']} 0 0 0 0</pose>
    </include>
"""
        count += 1

    sdf_content += "  </model>\n</sdf>"
    
    with open(output_file, 'w') as f:
        f.write(sdf_content)
    print(f"Success! Created {output_file} with {count} cones.")

# הפעלת הפונקציה
create_sdf('CompetitionMapTestday1.csv', 'model.sdf')