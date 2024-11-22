import bpy
import numpy as np
import math
import mathutils 
import csv

output_path = bpy.path.abspath('//uma_signal_strength.csv')


c = 3e8  # Speed of light (m/s)
frequency = 2.4e9  # Frequency in Hz (example: 2.4 GHz for Wi-Fi)
tx_pow = 35  # Transmitter (cell tower) gain in dBi
rx_gain = 0  # Receiver (mobile) gain in dBi
beamwidth = 90  # Antenna beamwidth in degrees


h_bs = 25  # Base station antenna height in meters
h_ut = 1.5  # User terminal (receiver) antenna height in meters
distance_breakpoint = 4 * h_bs * h_ut * frequency / c  # Breakpoint distance


transmitter_pos = np.array([138, -33, 25])
transmitter_orientation = np.array([119.7, -95.079, 0])


def uma_path_loss_los(distance, frequency):
    if distance == 0:
        return 0
    if distance <= distance_breakpoint:
        return 22 * np.log10(distance) + 28 + 20 * np.log10(frequency / 1e9)
    else:
        return (40 * np.log10(distance) + 7.8 - 18 * np.log10(h_bs) - 18 * np.log10(h_ut) 
                + 2 * np.log10(frequency / 1e9))


def uma_path_loss_nlos(distance, frequency):
    if distance == 0:
        return 0
    los_path_loss = uma_path_loss_los(distance, frequency)
    nlos_path_loss = (161.04 - 7.1 * np.log10(h_ut) + 7.5 * np.log10(h_ut) 
                      - (24.37 - 3.7 * (h_ut / h_bs)**2) * np.log10(h_bs)
                      + (43.42 - 3.1 * np.log10(h_bs)) * (np.log10(distance) - 3)
                      + 20 * np.log10(frequency / 1e9) - (3.2 * (np.log10(11.75 * h_ut)**2 - 4.97)))
    return max(los_path_loss, nlos_path_loss)


def directional_gain(tx_location, rx_location, tx_orientation, beamwidth):
    
    point = mathutils.Vector(tx_orientation)
    tx_location = mathutils.Vector(tx_location)
    rx_location = mathutils.Vector(rx_location)
    direction_vector = (rx_location - tx_location).normalized()
    orientation_vector = (point - tx_location).normalized()

    # Compute the dot product of the two vectors
    dot_product = direction_vector.dot(orientation_vector)

    # Clamp the dot product value between -1 and 1 to avoid numerical issues with acos
    dot_product = max(min(dot_product, 1.0), -1.0)

    # Calculate the angle in radians using arccos (inverse cosine)
    angle_radians = math.acos(dot_product)

    # Optionally convert the angle to degrees
    angle_between = math.degrees(angle_radians)
    
#    direction_vector = (rx_location - tx_location).normalized()
#    tx_direction = tx_orientation.to_matrix() @ mathutils.Vector((0, 0, 1))
#    
#    
#    angle_between = math.degrees(direction_vector.angle(tx_direction))
    
   
    if angle_between > beamwidth / 2:
        return 0 
    else:
        return tx_pow 


def signal_strength_uma(distance, frequency, pow, los=True):
    if los:
        path_loss = uma_path_loss_los(distance, frequency)
    else:
        path_loss = uma_path_loss_nlos(distance, frequency)
    
   
    received_power = pow + rx_gain - path_loss
    return received_power




def yagi_signal_strength(theta, phi):

    theta_rad = np.radians(theta)
    phi_rad = np.radians(phi)


    main_lobe_gain = 1
    side_lobe_gain = 0.5
    null_gain = 0.1


    if 75 < theta < 105: 
        return tx_pow + 10 * np.log10(main_lobe_gain)
    elif 30 < theta < 75 or 105 < theta < 150:
        return tx_pow + 10 * np.log10(side_lobe_gain)
    else:  # Nulls
        return tx_pow + 10 * np.log10(null_gain)


def is_los(transmitter_pos, receiver_pos, ground_obj_name):

    transmitter_pos = mathutils.Vector(transmitter_pos)
    receiver_pos = mathutils.Vector(receiver_pos)


    direction = receiver_pos - transmitter_pos
    distance = direction.length  # Exact distance
    direction_normalized = direction.normalized()
    
   
    depsgraph = bpy.context.evaluated_depsgraph_get()

  
    hit, location, normal, index, obj, matrix = bpy.context.scene.ray_cast(
        depsgraph, transmitter_pos, direction_normalized, distance=distance
    )


    print(f"Ray-casting from {transmitter_pos} to {receiver_pos}")
    print(f"Hit result: {hit}")
    hit_coords = None
    if hit and obj:
        print(f"Object hit: {obj.name} (type: {obj.type}) at location: {location}")
        hit_coords = location
    else:
        print("No object hit, clear LOS")



    
    if hit:
        if obj.type == 'MESH' and obj.name.split('.')[0] != ground_obj_name:
            print(f"LOS blocked by: {obj.name} at location {location}")
#            create_debug_line(transmitter_pos, receiver_pos)
#            highlight_coordinate(hit_coords)
            return False, hit_coords

        
    return True, hit_coords


def create_debug_line(start, end):
    curve_data = bpy.data.curves.new('debug_line', type='CURVE')
    curve_data.dimensions = '3D'
    
    polyline = curve_data.splines.new('POLY')
    polyline.points.add(1)
    polyline.points[0].co = (start.x, start.y, start.z, 1)
    polyline.points[1].co = (end.x, end.y, end.z, 1)

    curve_obj = bpy.data.objects.new('DebugLine', curve_data)
    bpy.context.scene.collection.objects.link(curve_obj)
    curve_obj.hide_select = True
    print(f"Debug line created from {start} to {end}")

def highlight_coordinate(coordinate, radius=0.1, color=(1.0, 0.0, 0.0)):
    
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=coordinate)
    obj = bpy.context.object
    obj.name = "HighlightSphere"
    
    
    mat = bpy.data.materials.new(name="HighlightMaterial")
    mat.diffuse_color = (*color, 1.0)
    
    
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)
    
    print(f"Highlighted coordinate at: {coordinate}")

def interpolate_color(normalized_value):
    # VIBGYOR colors in RGB
    vibgyor_colors = [
        (0.56, 0.0, 1.0, 1.0),  # Violet
        (0.29, 0.0, 0.51, 1.0), # Indigo
        (0.0, 0.0, 1.0, 1.0),   # Blue
        (0.0, 1.0, 0.0, 1.0),   # Green
        (1.0, 1.0, 0.0, 1.0),   # Yellow
        (1.0, 0.65, 0.0, 1.0),  # Orange
        (1.0, 0.0, 0.0, 1.0)    # Red
    ]
    
    num_colors = len(vibgyor_colors)
    
    # Find two adjacent colors for interpolation
    position = normalized_value * (num_colors - 1)  # Scale to color index range
    index1 = int(position)
    index2 = min(index1 + 1, num_colors - 1)  # Ensure we don't go out of bounds
    t = position - index1  # Interpolation factor between index1 and index2

    # Interpolate between the two colors
    color1 = vibgyor_colors[index1]
    color2 = vibgyor_colors[index2]
    interpolated_color = tuple((1 - t) * c1 + t * c2 for c1, c2 in zip(color1, color2))

    return interpolated_color

#point = np.array([119.7, -95.079, 0])
#point1  = np.array([186.58, -190.62, 0])
#res, coord1 = is_los(transmitter_pos, point, "ground")
#res1, coord2 = is_los(transmitter_pos, point1, "ground")


lines = []

with open(output_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['ID', 'Distance', 'Signal Strength (dBm)', 'Norm', 'LOS/NLOS'])
    
    obj_list = []
    collection_name = "Collection"

    # Loop through all objects in the scene
    for obj in bpy.context.scene.objects:
        obj_list.append(obj)

    # Loop through all objects in the specified collection
    if collection_name in bpy.data.collections:
        collection = bpy.data.collections[collection_name]
        for obj in collection.objects:
            obj_list.append(obj)
        

    for obj in obj_list:
        if obj.type == 'MESH' and 'ground' in  obj.name:

            if not obj.data.vertex_colors:
                obj.data.vertex_colors.new()

            color_layer1 = obj.data.vertex_colors.get("SignalStrengthLayer"+str(obj.name)) or obj.data.vertex_colors.new(name="SignalStrengthLayer"+str(obj.name))
            
            color_layer2 = obj.data.vertex_colors.get("LOSLayer"+str(obj.name)) or obj.data.vertex_colors.new(name="LOSLayer"+str(obj.name))

            # Iterate over the mesh vertices
            for poly in obj.data.polygons:
                for loop_index in poly.loop_indices:
                    vertex_index = obj.data.loops[loop_index].vertex_index
                    vertex = obj.data.vertices[vertex_index]
                    vertex_pos = np.array(obj.matrix_world @ vertex.co)

                    
                    distance = np.linalg.norm(vertex_pos - transmitter_pos)
                    
                   
                    los = is_los(transmitter_pos, vertex_pos, 'ground')
                    pow = directional_gain(transmitter_pos, vertex_pos, transmitter_orientation, beamwidth)
                    signal_strength = signal_strength_uma(distance, frequency, pow, los)

                   
                    min_signal = -100 
                    max_signal = -40
                    normalized_strength = (signal_strength - min_signal) / (max_signal - min_signal)
                    normalized_strength = max(0, min(1, normalized_strength))
                    
                    color_layer1.data[loop_index].color = interpolate_color(normalized_strength)

                    if los[0]:
                        color_layer2.data[loop_index].color = (1, 0, 0, 1)
                        lines.append(vertex_pos)
                    else:
                        color_layer2.data[loop_index].color = (0, 0, 1, 1)
                    writer.writerow([str(obj.name)+str(vertex_pos), distance, signal_strength, normalized_strength, str(los)])


#for i in lines:
#    create_debug_line(mathutils.Vector(transmitter_pos), mathutils.Vector(i))

bpy.context.view_layer.update()

def clear_previous_debug_lines():
    
    for obj in bpy.data.objects:
        if obj.name.startswith("DebugLine"):
            bpy.data.objects.remove(obj, do_unlink=True)
            
#clear_previous_debug_lines()

