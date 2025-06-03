from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import numpy as np
# # Assign to variables with rounding
# drawer_height = 0.333
# width = 0.958
# depth = 0.998

# cabinet_height = 1.003
# s = synth.Scene()

def basement_kitchen(drawer_height: float, width: float, depth: float, cabinet_height: float, s, show=False):    
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

    assets = [f'drawer_{drawer_height}_{width}_{depth}_0', 
            f'drawer_{drawer_height}_{width}_{depth}_1', 
            f'lower_left_cabinet_{cabinet_height}_{width}_{depth}_0', 
            f'lower_right_cabinet_{cabinet_height}_{width}_{depth}_0',
            f'drawer_{drawer_height}_{width}_{depth}_2', 
            f'drawer_{drawer_height}_{width}_{depth}_3', 
            f'lower_left_cabinet_{cabinet_height}_{width}_{depth}_1', 
            f'lower_right_cabinet_{cabinet_height}_{width}_{depth}_1',
            f'drawer_{drawer_height}_{width}_{depth}_4', 
            f'drawer_{drawer_height}_{width}_{depth}_5', 
            f'lower_left_cabinet_{cabinet_height}_{width}_{depth}_2', 
            f'lower_right_cabinet_{cabinet_height}_{width}_{depth}_2']

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

    s.add_object(
        drawer,
        assets[4],
        connect_parent_id=assets[1],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        drawer,
        assets[5],
        connect_parent_id=assets[4],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        lower_left_cabinet,
        assets[6],
        connect_parent_id=assets[5],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'top'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        lower_right_cabinet,
        assets[7],
        connect_parent_id=assets[4],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'top'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )

    rotation = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    translation = np.eye(4)
    translation[0, 3] = 0.998   # move 0.998 units in X
    translation[1, 3] = -0.998   # move +0.998 along Y
    transform = translation @ rotation

    s.add_object(
        drawer,
        assets[8],
        connect_parent_id=assets[5],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        transform=transform
    )
    s.add_object(
        drawer,
        assets[9],
        connect_parent_id=assets[8],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        lower_left_cabinet,
        assets[10],
        connect_parent_id=assets[9],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'top'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        lower_right_cabinet,
        assets[11],
        connect_parent_id=assets[8],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'top'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )

    if show:
        s.show()

    return assets

# basement_kitchen(drawer_height, width, depth, cabinet_height, s, show=True)