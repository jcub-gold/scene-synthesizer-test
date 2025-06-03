from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import numpy as np
from basement_kitchen import basement_kitchen

# Hardcoded/Estimated dimensions: translation up to wall cabinets, double drawers at the bottom, countertop, floor buffer
# Ideally get better scaled meshes from the scene

drawer_raw_mesh_path = 'drawer_3.obj'
door_raw_mesh_path = 'door_4.obj'
upper_cabinet_raw_mesh_path = 'upper_cabinet.obj'
double_drawer_raw_mesh_path = 'double_drawer.obj'
drawer_dims = sorted(extract_dimensions(drawer_raw_mesh_path))
door_dims = sorted(extract_dimensions(door_raw_mesh_path))
upper_cabinet_dims = sorted(extract_dimensions(upper_cabinet_raw_mesh_path))
double_drawer_dims = sorted(extract_dimensions(double_drawer_raw_mesh_path))

print("Drawer dimensions:", drawer_dims)
print("Door dimensions:", door_dims)

# Assign to variables with rounding
drawer_height = round(drawer_dims[0], 3)
width = round(drawer_dims[1], 3)
depth = round(drawer_dims[2], 3)

cabinet_height = round(np.max(door_dims), 3)

upper_cabinet_depth = 0.65 # derived from ratio's of the bodies have issues with the scale of meshes
upper_cabinet_height = 2.103 # derived from ratio's of the bodies have issues with the scale of meshes

double_drawer_height = round(np.min(double_drawer_dims) * width * 2 / np.max(double_drawer_dims), 3) # proportional height to width of scene scale
upper_cabinet_translation = 0.9 # hardcoded translation to place upper cabinets above lower cabinets

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

urdf_path = 'full_pipeline_simple/initial_scene.urdf'
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
        warped_mesh_path = f"full_pipeline_simple/{asset}.obj"
        warped.export(warped_mesh_path)

        drawer.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)
    if "lower_" in asset:
        urdf_link_name = asset + "_door_0_0"
        cabinet = ARMDrawer(urdf_path, door_raw_mesh_path, urdf_link_name)
        cabinet.set_urdf()
        cabinet.set_mesh()
        cabinet.extract_corners(weight_y_axis=0.5)
        cabinet.warp()

        warped = cabinet.get_warped_mesh().copy()
        warped_mesh_path = f"full_pipeline_simple/{asset}.obj"
        warped.export(warped_mesh_path)

        cabinet.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)
    if "upper_" in asset:
        urdf_link_name = asset + "_door_0_0"
        upper_cabinet = ARMDrawer(urdf_path, upper_cabinet_raw_mesh_path, urdf_link_name)
        upper_cabinet.set_urdf()
        upper_cabinet.set_mesh()
        upper_cabinet.extract_corners(weight_y_axis=0.5)
        upper_cabinet.warp()

        warped = upper_cabinet.get_warped_mesh().copy()
        warped_mesh_path = f"full_pipeline_simple/{asset}.obj"
        warped.export(warped_mesh_path)

        upper_cabinet.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)
        
    if 'double_drawer' in asset:
        urdf_link_name = asset + "_drawer_0_0"
        double_drawer = ARMDrawer(urdf_path, double_drawer_raw_mesh_path, urdf_link_name)
        double_drawer.set_urdf()
        double_drawer.set_mesh()
        double_drawer.extract_corners(weight_y_axis=100)
        double_drawer.warp()

        warped = double_drawer.get_warped_mesh().copy()
        warped_mesh_path = f"full_pipeline_simple/{asset}.obj"
        warped.export(warped_mesh_path)

        double_drawer.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)




