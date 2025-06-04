from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import numpy as np
from second_floor_kitchen import second_floor_kitchen
import trimesh


# Hardcoded/Estimated dimensions: translation up to wall cabinets, double drawers at the bottom, countertop, floor buffer
# Ideally get better scaled meshes from the scene

drawer_raw_mesh_path = 'second_floor_1_5_img/top_drawer_left/object_5_mesh.obj'
door_raw_mesh_path = 'second_floor_1/bottom_cabinet_left_door/object_1_mesh.obj'
upper_cabinet_raw_mesh_path = 'second_floor_1/top_cabinet_left_door/object_9_mesh.obj'
big_drawer_raw_mesh_path = 'second_floor_1_5_img/bottom_drawer/object_4_mesh.obj'

drawer_dims = sorted(extract_dimensions(drawer_raw_mesh_path))
door_dims = sorted(extract_dimensions(door_raw_mesh_path))
upper_cabinet_dims = sorted(extract_dimensions(upper_cabinet_raw_mesh_path))
big_drawer_dims = sorted(extract_dimensions(big_drawer_raw_mesh_path))

print("Drawer dimensions:", drawer_dims)
print("Lower door dimensions:", door_dims)

# Assign to variables with rounding
drawer_height = round(drawer_dims[0], 3)
width = round(door_dims[1], 3)
depth = round(drawer_dims[2], 3)

cabinet_height = round(np.max(door_dims), 3)

# Scale upper cabinet door
mesh = trimesh.load(upper_cabinet_raw_mesh_path)
scale = round(width / upper_cabinet_dims[1], 3)
mesh.apply_scale(scale)
base, ext = os.path.splitext(upper_cabinet_raw_mesh_path)
scaled_upper_cabinet_raw_mesh_path = f"{base}_scaled_{scale}{ext}"
mesh.export(scaled_upper_cabinet_raw_mesh_path)
scaled_upper_cabinet_dims = sorted(extract_dimensions(scaled_upper_cabinet_raw_mesh_path))
print("Upper door dimensions:", upper_cabinet_dims)
print("Scaled upper door dimensions:", scaled_upper_cabinet_dims)

# scale big upper cabinet door
mesh = trimesh.load(upper_cabinet_raw_mesh_path)
scale = round(depth / upper_cabinet_dims[1], 3)
mesh.apply_scale(scale)
base, ext = os.path.splitext(upper_cabinet_raw_mesh_path)
big_upper_cabinet_raw_mesh_path = f"{base}_scaled_{scale}{ext}"
mesh.export(big_upper_cabinet_raw_mesh_path)

# # Scale big drawer
# mesh = trimesh.load(big_drawer_raw_mesh_path)
# scale = round(depth / big_drawer_dims[1], 3)
# mesh.apply_scale(scale)
# base, ext = os.path.splitext(big_drawer_raw_mesh_path)
# scaled_big_drawer_raw_mesh_path = f"{base}_scaled_{scale}{ext}"
# mesh.export(scaled_big_drawer_raw_mesh_path)
# scaled_big_drawer_dims = sorted(extract_dimensions(scaled_big_drawer_raw_mesh_path))
# print("Big drawer dimensions:", big_drawer_dims)
# print("Scaled big drawer dimensions:", scaled_big_drawer_dims)

upper_cabinet_depth = 0.5 * depth # derived from ratio's of the bodies have issues with the scale of meshes
upper_cabinet_translation = 0.9 # hardcoded translation to place upper cabinets above lower cabinets
upper_cabinet_height = cabinet_height + drawer_height # round(np.max(scaled_upper_cabinet_dims), 3)
big_drawer_height = round(np.min(big_drawer_dims), 3)

s = synth.Scene()
assets = second_floor_kitchen(drawer_height=drawer_height, 
                          width=width, 
                          depth=depth, 
                          cabinet_height=cabinet_height, 
                          upper_cabinet_depth=upper_cabinet_depth,
                          upper_cabinet_height=upper_cabinet_height,
                          big_drawer_height=big_drawer_height,
                          upper_cabinet_translation=upper_cabinet_translation,
                          s=s)

output_dir = 'second_floor_kitchen_pipleine'
urdf_path = f'{output_dir}/second_floor_kitchen.urdf'
s.export(urdf_path)

for asset in assets:
    if "drawer" in asset and 'big' not in asset:
        urdf_link_name = asset + "_drawer_0_0"
        drawer = ARMDrawer(urdf_path, drawer_raw_mesh_path, urdf_link_name)
        drawer.set_urdf()
        drawer.set_mesh()
        drawer.extract_corners(sample_count=200000, weight_y_axis=100)
        drawer.warp()
        # drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)

        warped = drawer.get_warped_mesh().copy()
        warped_mesh_path = f"{output_dir}/{asset}.obj"
        warped.export(warped_mesh_path)

        drawer.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)
    if "lower_" in asset:
        urdf_link_name = asset + "_door_0_0"
        cabinet = ARMDrawer(urdf_path, door_raw_mesh_path, urdf_link_name)
        cabinet.set_urdf()
        cabinet.set_mesh()
        cabinet.extract_corners(weight_y_axis=0.5)
        cabinet.warp()
        # cabinet.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)

        warped = cabinet.get_warped_mesh().copy()
        warped_mesh_path = f"{output_dir}/{asset}.obj"
        warped.export(warped_mesh_path)

        cabinet.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)
    if "upper_" in asset and 'big' not in asset:
        urdf_link_name = asset + "_door_0_0"
        upper_cabinet = ARMDrawer(urdf_path, scaled_upper_cabinet_raw_mesh_path, urdf_link_name)
        upper_cabinet.set_urdf()
        upper_cabinet.set_mesh()
        # Reflection matrix across the y-axis
        R = np.array([
            [-1,  0,  0,  0],
            [ 0,  1,  0,  0],
            [ 0,  0,  -1,  0],
            [ 0,  0,  0,  1]
        ])
        upper_cabinet.extract_corners(sample_count=500000, weight_y_axis=0, manual_reflection=R)
        upper_cabinet.warp()
        # upper_cabinet.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)

        warped = upper_cabinet.get_warped_mesh().copy()
        warped_mesh_path = f"{output_dir}/{asset}.obj"
        warped.export(warped_mesh_path)

        upper_cabinet.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)

    if 'big_drawer' in asset:
        urdf_link_name = asset + "_drawer_0_0"
        big_drawer = ARMDrawer(urdf_path, big_drawer_raw_mesh_path, urdf_link_name)
        big_drawer.set_urdf()
        big_drawer.set_mesh()
        big_drawer.extract_corners(weight_y_axis=100)
        big_drawer.warp()
        # big_drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=False, show_points=True, show_aabb=False)

        warped = big_drawer.get_warped_mesh().copy()
        warped_mesh_path = f"{output_dir}/{asset}.obj"
        warped.export(warped_mesh_path)

        big_drawer.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)
    if 'big_upper' in asset:
        urdf_link_name = asset + "_door_0_0"
        big_upper_cabinet = ARMDrawer(urdf_path, big_upper_cabinet_raw_mesh_path, urdf_link_name)
        big_upper_cabinet.set_urdf()
        big_upper_cabinet.set_mesh()
        # Reflection matrix across the y-axis
        R = np.array([
            [-1,  0,  0,  0],
            [ 0,  1,  0,  0],
            [ 0,  0,  -1,  0],
            [ 0,  0,  0,  1]
        ])
        big_upper_cabinet.extract_corners(sample_count=500000, weight_y_axis=0, manual_reflection=R)
        big_upper_cabinet.warp()
        # big_upper_cabinet.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)

        warped = big_upper_cabinet.get_warped_mesh().copy()
        warped_mesh_path = f"{output_dir}/{asset}.obj"
        warped.export(warped_mesh_path)

        big_upper_cabinet.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)

# idea, scale the meshes first, then extract dimensions