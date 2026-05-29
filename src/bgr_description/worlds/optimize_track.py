#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import sys
import os

def optimize_world(input_path, output_path):
    if not os.path.exists(input_path):
        print(f"Error: Input file {input_path} does not exist.")
        return False

    print(f"Processing: {input_path} -> {output_path}")

    # Parse XML
    try:
        tree = ET.parse(input_path)
        root = tree.getroot()
    except Exception as e:
        print(f"Error parsing XML: {e}")
        return False

    world_tag = root.find('world')
    if world_tag is None:
        print("Error: No <world> tag found in SDF.")
        return False

    # Find all includes and separate cone includes from others
    includes_to_remove = []
    cone_visuals = []
    
    cone_colors = {
        'cone_yellow': {
            'mesh': 'model://cone_yellow/meshes/cone.stl',
            'ambient': '1.0 1.0 0.0 1',
            'diffuse': '1.0 1.0 0.0 1'
        },
        'cone_blue': {
            'mesh': 'model://cone_blue/meshes/cone.stl',
            'ambient': '0.0 0.0 1.0 1',
            'diffuse': '0.0 0.0 1.0 1'
        },
        'cone_orange': {
            'mesh': 'model://cone_orange/meshes/cone.stl',
            'ambient': '1.0 0.35 0.0 1',
            'diffuse': '1.0 0.35 0.0 1'
        }
    }

    for include in world_tag.findall('include'):
        uri_tag = include.find('uri')
        if uri_tag is None or not uri_tag.text:
            continue
            
        uri_text = uri_tag.text.strip().lower()
        
        # Check if it's a cone
        matched_color = None
        for color_key in cone_colors:
            if color_key in uri_text:
                matched_color = color_key
                break
                
        if matched_color:
            name_tag = include.find('name')
            pose_tag = include.find('pose')
            
            name = name_tag.text.strip() if (name_tag is not None and name_tag.text) else f"cone_{matched_color}_{len(cone_visuals)}"
            pose = pose_tag.text.strip() if (pose_tag is not None and pose_tag.text) else "0 0 0 0 0 0"
            
            # Create visual element
            visual = ET.Element('visual', {'name': f"visual_{name}"})
            
            pose_elem = ET.SubElement(visual, 'pose')
            pose_elem.text = pose
            
            geom = ET.SubElement(visual, 'geometry')
            mesh = ET.SubElement(geom, 'mesh')
            uri = ET.SubElement(mesh, 'uri')
            uri.text = cone_colors[matched_color]['mesh']
            scale = ET.SubElement(mesh, 'scale')
            scale.text = '0.005 0.005 0.005'
            
            material = ET.SubElement(visual, 'material')
            ambient = ET.SubElement(material, 'ambient')
            ambient.text = cone_colors[matched_color]['ambient']
            diffuse = ET.SubElement(material, 'diffuse')
            diffuse.text = cone_colors[matched_color]['diffuse']
            
            cone_visuals.append(visual)
            includes_to_remove.append(include)

    if not cone_visuals:
        print("No cone includes found to optimize.")
        return False

    # Remove includes from world
    for include in includes_to_remove:
        world_tag.remove(include)

    # Check if a model named competition_track_cones already exists and remove it
    for model in world_tag.findall('model'):
        if model.get('name') == 'competition_track_cones':
            world_tag.remove(model)

    # Create the optimized single model
    model = ET.Element('model', {'name': 'competition_track_cones'})
    static_elem = ET.SubElement(model, 'static')
    static_elem.text = 'true'
    
    link = ET.SubElement(model, 'link', {'name': 'link'})
    pose_link = ET.SubElement(link, 'pose')
    pose_link.text = '0 0 0 0 0 0'
    
    for visual in cone_visuals:
        link.append(visual)
        
    world_tag.append(model)

    # Apply shadows and physics settings
    scene = world_tag.find('scene')
    if scene is not None:
        shadows = scene.find('shadows')
        if shadows is not None:
            shadows.text = '0'
        else:
            shadows_elem = ET.SubElement(scene, 'shadows')
            shadows_elem.text = '0'

    physics = world_tag.find('physics')
    if physics is not None:
        max_step = physics.find('max_step_size')
        if max_step is not None:
            max_step.text = '0.002'
        else:
            max_step_elem = ET.SubElement(physics, 'max_step_size')
            max_step_elem.text = '0.002'

        update_rate = physics.find('real_time_update_rate')
        if update_rate is not None:
            update_rate.text = '500'
        else:
            update_rate_elem = ET.SubElement(physics, 'real_time_update_rate')
            update_rate_elem.text = '500'

    for light in world_tag.findall('light'):
        if light.get('name') == 'sun':
            cast_shadows = light.find('cast_shadows')
            if cast_shadows is not None:
                cast_shadows.text = 'false'
            else:
                cast_shadows_elem = ET.SubElement(light, 'cast_shadows')
                cast_shadows_elem.text = 'false'

    # Pretty-print
    if hasattr(ET, 'indent'):
        ET.indent(tree, space="  ", level=0)
        
    try:
        tree.write(output_path, encoding="utf-8", xml_declaration=True)
        print(f"Success! Optimized {len(cone_visuals)} cones into {output_path}.")
        return True
    except Exception as e:
        print(f"Error writing output file: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 optimize_track.py <input_world_path> <output_world_path>")
        sys.exit(1)
        
    optimize_world(sys.argv[1], sys.argv[2])
