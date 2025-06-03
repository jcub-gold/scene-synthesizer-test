from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa

raw_mesh_path = 'scene/object_3_state_1_aligned_mesh.obj'
dims = extract_dimensions(raw_mesh_path)

# Sort and round dimensions
sorted_dims = sorted(dims)

# Assign to variables with rounding
drawer_height = round(sorted_dims[0], 3)
width = round(sorted_dims[1], 3)
depth = round(sorted_dims[2], 3)

cabinet_height = drawer_height + 0.39  # Not extracted yet

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
        drawer = ARMDrawer(urdf_path, raw_mesh_path, urdf_link_name)
        drawer.set_urdf()
        drawer.set_mesh()
        drawer.extract_corners()
        drawer.warp()

        warped = drawer.get_warped_mesh().copy()
        warped_mesh_path = f"full_pipeline_simple/{asset}.obj"
        warped.export(warped_mesh_path)

        drawer.replace_geometry(input_urdf=urdf_path, output_urdf=urdf_path, mesh_path=warped_mesh_path)







