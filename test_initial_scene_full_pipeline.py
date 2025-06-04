from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import numpy as np
from basement_kitchen import basement_kitchen
import trimesh


# Hardcoded/Estimated dimensions: translation up to wall cabinets, double drawers at the bottom, countertop, floor buffer
# Ideally get better scaled meshes from the scene

drawer_raw_mesh_path = 'basement_1_5_img/top_drawer_left/object_7_mesh.obj'
door_raw_mesh_path = 'basement_1_5_img/bottom_cabinet_right_door/object_4_mesh.obj'
upper_cabinet_raw_mesh_path = 'basement_1_5_img/top_cabinet_left_door/object_2_mesh.obj'
double_drawer_raw_mesh_path = 'double_drawer.obj'

drawer_dims = sorted(extract_dimensions(drawer_raw_mesh_path))
door_dims = sorted(extract_dimensions(door_raw_mesh_path))
upper_cabinet_dims = sorted(extract_dimensions(upper_cabinet_raw_mesh_path))
double_drawer_dims = sorted(extract_dimensions(double_drawer_raw_mesh_path))

print("Drawer dimensions:", drawer_dims)
print("Lower door dimensions:", door_dims)

# Assign to variables with rounding
drawer_height = round(drawer_dims[0], 3)
width = round(drawer_dims[1], 3)
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


# Scale double drawer
mesh = trimesh.load(double_drawer_raw_mesh_path)
scale = round(depth / double_drawer_dims[1], 3)
mesh.apply_scale(scale)
base, ext = os.path.splitext(double_drawer_raw_mesh_path)
scaled_double_drawer_raw_mesh_path = f"{base}_scaled_{scale}{ext}"
mesh.export(scaled_double_drawer_raw_mesh_path)
scaled_double_drawer_dims = sorted(extract_dimensions(scaled_double_drawer_raw_mesh_path))
print("Double drawer dimensions:", double_drawer_dims)
print("Scaled double drawer dimensions:", scaled_double_drawer_dims)

upper_cabinet_depth = 0.5 * depth # derived from ratio's of the bodies have issues with the scale of meshes
upper_cabinet_translation = 0.9 # hardcoded translation to place upper cabinets above lower cabinets
upper_cabinet_height = round(np.max(scaled_upper_cabinet_dims), 3)
double_drawer_height = round(np.min(scaled_double_drawer_dims) * width * 2 / np.max(scaled_double_drawer_dims), 3)

s = synth.Scene()
assets = basement_kitchen(drawer_height=drawer_height, 
                          width=width, 
                          depth=depth, 
                          cabinet_height=cabinet_height, 
                          upper_cabinet_depth=upper_cabinet_depth,
                          upper_cabinet_height=upper_cabinet_height,
                          double_drawer_height=double_drawer_height,
                          upper_cabinet_translation=upper_cabinet_translation,
                          s=s)

urdf_path = 'basement_kitchen_pipleine/initial_scene.urdf'
s.export(urdf_path)

for asset in assets:
    if "drawer" in asset and 'double' not in asset:
        urdf_link_name = asset + "_drawer_0_0"
        drawer = ARMDrawer(urdf_path, drawer_raw_mesh_path, urdf_link_name)
        drawer.set_urdf()
        drawer.set_mesh()
        drawer.extract_corners(weight_y_axis=100)
        drawer.warp()

        warped = drawer.get_warped_mesh().copy()
        warped_mesh_path = f"basement_kitchen_pipleine/{asset}.obj"
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
        warped_mesh_path = f"basement_kitchen_pipleine/{asset}.obj"
        warped.export(warped_mesh_path)

        cabinet.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)
    if "upper_" in asset:
        urdf_link_name = asset + "_door_0_0"
        upper_cabinet = ARMDrawer(urdf_path, scaled_upper_cabinet_raw_mesh_path, urdf_link_name)
        upper_cabinet.set_urdf()
        upper_cabinet.set_mesh()
        # Reflection matrix across the y-axis
        R = np.array([
            [1,  0,  0,  0],
            [ 0,  -1,  0,  0],
            [ 0,  0,  1,  0],
            [ 0,  0,  0,  1]
        ])
        upper_cabinet.extract_corners(weight_y_axis=0.5, manual_reflection=R)
        upper_cabinet.warp()
        # upper_cabinet.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)

        warped = upper_cabinet.get_warped_mesh().copy()
        warped_mesh_path = f"basement_kitchen_pipleine/{asset}.obj"
        warped.export(warped_mesh_path)

        upper_cabinet.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)

    if 'double_drawer' in asset:
        urdf_link_name = asset + "_drawer_0_0"
        double_drawer = ARMDrawer(urdf_path, scaled_double_drawer_raw_mesh_path, urdf_link_name)
        double_drawer.set_urdf()
        double_drawer.set_mesh()
        R = np.array([
            [1,  0,  0,  0],
            [ 0,  -1,  0,  0],
            [ 0,  0,  1,  0],
            [ 0,  0,  0,  1]
        ])
        double_drawer.extract_corners(weight_y_axis=100, manual_reflection=R)
        double_drawer.warp()
        # double_drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=False, show_points=True, show_aabb=False)

        warped = double_drawer.get_warped_mesh().copy()
        warped_mesh_path = f"basement_kitchen_pipleine/{asset}.obj"
        warped.export(warped_mesh_path)

        double_drawer.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)


# idea, scale the meshes first, then extract dimensions