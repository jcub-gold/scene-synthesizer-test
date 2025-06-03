from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import numpy as np

drawer_raw_mesh_path = 'drawer_3.obj'
door_raw_mesh_path = 'door_4.obj'
drawer_dims = sorted(extract_dimensions(drawer_raw_mesh_path))
door_dims = sorted(extract_dimensions(door_raw_mesh_path))

print("Drawer dimensions:", drawer_dims)
print("Door dimensions:", door_dims)

# Assign to variables with rounding
drawer_height = round(drawer_dims[0], 3)
width = round(drawer_dims[1], 3)
depth = round(drawer_dims[2], 3)

cabinet_height = round(np.max(door_dims), 3)

drawer = pa.BaseCabinetAsset(width=width, height=drawer_height, depth=depth, drawer_height=drawer_height, include_foot_panel=False, include_cabinet_doors=False, num_drawers_horizontal=1)
lower_left_cabinet = pa.BaseCabinetAsset(width=width, 
                                         height=cabinet_height, 
                                         depth=depth, 
                                         num_drawers_vertical=0,
                                         include_cabinet_doors=True,
                                         include_foot_panel=False,
                                         lower_compartment_types=("door_left",))
lower_right_cabinet = pa.BaseCabinetAsset(width=width, 
                                         height=cabinet_height, 
                                         depth=depth, 
                                         num_drawers_vertical=0,
                                         include_cabinet_doors=True,
                                         include_foot_panel=False,
                                         lower_compartment_types=("door_right",))
assets = [f'drawer_{drawer_height}_{width}_{depth}_0', f'drawer_{drawer_height}_{width}_{depth}_1', f'lower_left_cabinet_{cabinet_height}_{width}_{depth}_0', f'lower_right_cabinet_{cabinet_height}_{width}_{depth}_0']

s = synth.Scene()
s.add_object(drawer, assets[0])
s.add_object(
    drawer,
    assets[1],
    connect_parent_id=assets[0],
    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    lower_left_cabinet,
    assets[2],
    connect_parent_id=assets[1],
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'top'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    lower_right_cabinet,
    assets[3],
    connect_parent_id=assets[0],
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'top'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)

# s.show()
urdf_path = 'full_pipeline_simple/initial_scene.urdf'
s.export(urdf_path)

for asset in assets:
    if "drawer" in asset:
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
    if "cabinet" in asset:
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






