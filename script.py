import bpy
import numpy as np
import math
import time
import mathutils 
import csv
import bmesh
from mathutils.bvhtree import BVHTree

output_path = bpy.path.abspath('//uma_signal_strength.csv')


c = 3e8  # Speed of light (m/s)
frequency = 5e9 # Frequency in Hz (example: 2.4 GHz for Wi-Fi)
tx_pow = 35  # Transmitter (cell tower) gain in dBi
rx_gain = 0  # Receiver (mobile) gain in dBi
beamwidth = 45  # Antenna beamwidth in degrees


h_bs = 10  # Base station antenna height in meters
h_ut = 1.5  # User terminal (receiver) antenna height in meters
distance_breakpoint = 4 * h_bs * h_ut * frequency / c  # Breakpoint distance


transmitter_pos = np.array([-641.09, -33.909, h_bs])



transmitter_orientation = np.array([-89.307, -161.96, 0])


def clear_previous_debug_lines(name):
    
    for obj in bpy.data.objects:
        if obj.name.startswith(name):
            bpy.data.objects.remove(obj, do_unlink=True)
            
clear_previous_debug_lines("HighlightSphere")
clear_previous_debug_lines("DebugLine")


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

def signal_strength_oti(distance, frequency, pow, d2d):
    s_uma = signal_strength_uma(distance, frequency, pow,  False)
    
    
    res = (0.5*d2d) + 20 + s_uma
    print(s_uma, '\t', distance, 't', frequency, 't', pow, 't', res,'t', d2d)
    return res



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
    
def get_mesh(obj, depsgraph):
    """Convert an object to a temporary mesh (if not already a mesh)."""
    if obj.type == 'MESH':
        return obj.to_mesh(preserve_all_data_layers=True, depsgraph=depsgraph)
    else:
        return obj.to_mesh(depsgraph=depsgraph)




def is_los(transmitter_pos, receiver_pos, ground_obj_name, bvh):

    transmitter_pos = mathutils.Vector(transmitter_pos)
    receiver_pos = mathutils.Vector(receiver_pos)


    direction = receiver_pos - transmitter_pos
    distance = direction.length  # Exact distance
    direction_normalized = direction.normalized()
    
   
    

  
#    hit, location, normal, index, obj, matrix = bpy.context.scene.ray_cast(
#        depsgraph, transmitter_pos, direction_normalized, distance=distance
#    )
    
    hit, normal, index, distance = bvh.ray_cast(transmitter_pos, direction_normalized)


#    print(f"Ray-casting from {transmitter_pos} to {receiver_pos}")
#    print(f"Hit result: {hit}")


    
    if hit:
#        print(f"obj.type={obj.type}, obj.name={obj.name}" )
#        if obj.type == 'MESH' and 'ground' not in obj.name.split('.')[0] and obj.name != 'oti_layer':
#            print(f"LOS blocked by: {obj.name} at location {location}")
##            create_debug_line(transmitter_pos, receiver_pos)
##            highlight_coordinate(hit_coords)
        return (False, hit)

    return (True, hit)


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



lines = []



def create_bvh_for_objects(obj_names):
    """Creates a BVH tree for multiple objects by merging their meshes."""
    depsgraph = bpy.context.evaluated_depsgraph_get()
    combined_bm = bmesh.new()  # Create an empty BMesh

    for obj in obj_names:
        if obj is None:
            print(f"Warning: Object '{obj_name}' not found!")
            continue

        obj_eval = obj.evaluated_get(depsgraph)

        # Convert object to BMesh
        bm = bmesh.new()
        bm.from_object(obj_eval, depsgraph)
        bm.verts.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        if len(bm.faces) == 0:
            print(f"Warning: Object '{obj_name}' has no faces!")

        # Merge the BMesh into the combined mesh
        bm.transform(obj.matrix_world)  # Apply world transformation
        combined_bm.from_mesh(obj_eval.to_mesh(preserve_all_data_layers=True, depsgraph=depsgraph))
        bm.free()  # Free individual BMesh

    # Create a BVH tree from the merged BMesh
    if len(combined_bm.faces) == 0:
        print("Error: No faces found in any object, BVH tree cannot be built!")
        return None

    bvh_tree = BVHTree.FromBMesh(combined_bm)
    combined_bm.free()
    return bvh_tree


obj = None
collection_name = "OSM.001"
hit_list = []
for i in bpy.data.collections:
    if 'building' in i.name:
        for j in i.objects:
            hit_list.append(j)
            print(j)
bvh = create_bvh_for_objects(hit_list)

             

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
        if obj.type == 'MESH' and 'ground' ==  obj.name:

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
                    
                   
                    los = is_los(transmitter_pos, vertex_pos, 'ground', bvh)
                    
                    
                    
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
        
        if obj.type == 'MESH' and 'oti_layer' ==  obj.name:

            if not obj.data.vertex_colors:
                obj.data.vertex_colors.new()

            color_layer3 = obj.data.vertex_colors.get("OTI"+str(obj.name)) or obj.data.vertex_colors.new(name="OTI"+str(obj.name))
            
            min_signal = math.inf 
            max_signal = -math.inf
            
            store = {}

            # Iterate over the mesh vertices
            for poly in obj.data.polygons:
                for loop_index in poly.loop_indices:
                    vertex_index = obj.data.loops[loop_index].vertex_index
                    vertex = obj.data.vertices[vertex_index]
                    vertex_pos = np.array(obj.matrix_world @ vertex.co)

                    
                    distance = np.linalg.norm(vertex_pos - transmitter_pos)
                    
                   
                    los = is_los(transmitter_pos, vertex_pos, 'ground', bvh)
                    writer.writerow([1, 1, 1, 1, str(los)]) 
                    
                    if los[0] == False:
                        t1 = transmitter_pos
                        t1[2] = 0
                        t2 = los[1]
                        t2[2] = 0
                        d2d = np.linalg.norm(t1 - t2)
                        
                        pow = directional_gain(transmitter_pos, vertex_pos, transmitter_orientation, beamwidth)
                        signal_strength = signal_strength_oti(distance, frequency, pow, d2d)

                       
                        if signal_strength > max_signal:
                            max_signal = signal_strength
                        if signal_strength < min_signal:
                            min_signal = signal_strength
                        
                        store[loop_index] = signal_strength
            
            max_signal = sorted(store.values())[-1]
            print(f'Max: {max_signal}, Min: {min_signal}')
            for (key, val) in store.items():
                normalized_strength = (val - min_signal) / (max_signal - min_signal)
                normalized_strength = max(0, min(1, normalized_strength))
                        
                color_layer3.data[key].color = interpolate_color(normalized_strength)
                        
                writer.writerow([str(obj.name)+str(vertex_pos), distance, signal_strength, normalized_strength, str(los)])
                        
            


#for i in lines:
#    create_debug_line(mathutils.Vector(transmitter_pos), mathutils.Vector(i))


highlight_coordinate(transmitter_pos, 0.3, (0, 0, 0))

bpy.context.view_layer.update()

