import json
import argparse
import xml.etree.ElementTree as ET
from xml.dom import minidom

def calculate_wall_geometry(start_str, direction, hall_width, height, thickness):
    """
    Calculates the center point and size of a wall segment.
    """
    # Scale coordinates to match the requested hall_width directly
    # since the new JSON uses a 1.0 unitary grid.
    scale_factor = hall_width
    
    raw_x, raw_y = map(float, start_str.split(','))
    x_start = raw_x * scale_factor
    y_start = raw_y * scale_factor
    
    # Extend the wall length by its thickness to close the corner gaps
    length = hall_width + thickness
    
    if direction == "north":
        center_x = x_start
        center_y = y_start + (hall_width / 2.0)
        size = f"{thickness} {length} {height}"
    elif direction == "south":
        center_x = x_start
        center_y = y_start - (hall_width / 2.0)
        size = f"{thickness} {length} {height}"
    elif direction == "east":
        center_x = x_start + (hall_width / 2.0)
        center_y = y_start
        size = f"{length} {thickness} {height}"
    elif direction == "west":
        center_x = x_start - (hall_width / 2.0)
        center_y = y_start
        size = f"{length} {thickness} {height}"

    center_z = height / 2.0
    pose = f"{center_x} {center_y} {center_z} 0 0 0"
    
    return pose, size

def generate_sdf(json_file, output_file, hall_width, height, thickness):
    with open(json_file, 'r') as f:
        wall_data = json.load(f)

    sdf = ET.Element('sdf', version='1.8')
    world = ET.SubElement(sdf, 'world', name='maze_world')

    physics = ET.SubElement(world, 'plugin', name='ignition::gazebo::systems::Physics', filename='libignition-gazebo-physics-system.so')
    scene_broadcaster = ET.SubElement(world, 'plugin', name='ignition::gazebo::systems::SceneBroadcaster', filename='libignition-gazebo-scene-broadcaster-system.so')
    user_commands = ET.SubElement(world, 'plugin', name='ignition::gazebo::systems::UserCommands', filename='libignition-gazebo-user-commands-system.so')

    sensors = ET.SubElement(world, 'plugin', name='ignition::gazebo::systems::Sensors', filename='libignition-gazebo-sensors-system.so')
    ET.SubElement(sensors, 'render_engine').text = 'ogre'

    light = ET.SubElement(world, 'light', type='directional', name='sun')
    ET.SubElement(light, 'cast_shadows').text = 'true'
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'

    ground = ET.SubElement(world, 'model', name='ground_plane')
    ET.SubElement(ground, 'static').text = 'true'
    link = ET.SubElement(ground, 'link', name='link')
    ET.SubElement(link, 'pose').text = '0 0 0 0 0 0'
    
    collision = ET.SubElement(link, 'collision', name='collision')
    geom_col = ET.SubElement(collision, 'geometry')
    plane_col = ET.SubElement(geom_col, 'plane')
    ET.SubElement(plane_col, 'normal').text = '0 0 1'
    ET.SubElement(plane_col, 'size').text = '100 100'
    
    visual = ET.SubElement(link, 'visual', name='visual')
    geom_vis = ET.SubElement(visual, 'geometry')
    plane_vis = ET.SubElement(geom_vis, 'plane')
    ET.SubElement(plane_vis, 'normal').text = '0 0 1'
    ET.SubElement(plane_vis, 'size').text = '100 100'
    
    mat = ET.SubElement(visual, 'material')
    ambient = ET.SubElement(mat, 'ambient')
    ambient.text = '0.8 0.8 0.8 1'
    diffuse = ET.SubElement(mat, 'diffuse')
    diffuse.text = '0.8 0.8 0.8 1'

    for wall in wall_data:
        if wall['direction'] == "none":
            continue
        pose_str, size_str = calculate_wall_geometry(
            wall['start'], wall['direction'], hall_width, height, thickness
        )
        
        model = ET.SubElement(world, 'model', name=f"wall_{wall['id']}")
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = pose_str
        
        link = ET.SubElement(model, 'link', name='link')
        
        collision = ET.SubElement(link, 'collision', name='collision')
        geom_col = ET.SubElement(collision, 'geometry')
        box_col = ET.SubElement(geom_col, 'box')
        ET.SubElement(box_col, 'size').text = size_str
        
        visual = ET.SubElement(link, 'visual', name='visual')
        geom_vis = ET.SubElement(visual, 'geometry')
        box_vis = ET.SubElement(geom_vis, 'box')
        ET.SubElement(box_vis, 'size').text = size_str
        
        mat = ET.SubElement(visual, 'material')
        ambient = ET.SubElement(mat, 'ambient')
        ambient.text = '0.7 0.7 0.7 1'

    xml_string = ET.tostring(sdf, encoding='unicode')
    parsed_xml = minidom.parseString(xml_string)
    pretty_xml = parsed_xml.toprettyxml(indent="  ")
    
    with open(output_file, "w") as f:
        f.write(pretty_xml)
    
    print(f"Successfully generated {output_file} with {len(wall_data)} walls scaled to {hall_width}m width.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert JSON wall layout to Gazebo SDF.')
    parser.add_argument('--input', type=str, default='wall_coordinates.json', help='Input JSON file path')
    parser.add_argument('--output', type=str, default='maze.sdf', help='Output SDF file path')
    parser.add_argument('--hall_width', type=float, default=2.0, help='Width of the hallways in meters')
    parser.add_argument('--wall_height', type=float, default=1.0, help='Height of the walls in meters')
    parser.add_argument('--wall_thickness', type=float, default=0.1, help='Thickness of the walls in meters')
    
    args = parser.parse_args()
    generate_sdf(args.input, args.output, args.hall_width, args.wall_height, args.wall_thickness)