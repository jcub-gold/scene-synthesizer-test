from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import numpy as np
import math
# # Assign to variables with rounding
# drawer_height = 0.333
# width = 0.958
# depth = 0.998

# cabinet_height = 1.003

# upper_cabinet_depth = depth / 2
# upper_cabinet_height = 2.103 

# double_drawer_height = 1.5 * drawer_height

# upper_cabinet_translation = 0.9

# s = synth.Scene()

def basement_kitchen(drawer_height: float, width: float, depth: float, cabinet_height: float, upper_cabinet_depth:float, upper_cabinet_height:float, double_drawer_height:float, upper_cabinet_translation:float, s, show=False):    
    rotated_cabinet_width = (depth - upper_cabinet_depth) / math.sin(math.pi / 4)
    rotated_cabinet_depth = upper_cabinet_depth / math.cos(math.pi / 4)

    drawer = pa.BaseCabinetAsset(width=width, 
                                 height=drawer_height, 
                                 depth=depth, 
                                 drawer_height=drawer_height, 
                                 include_foot_panel=False, 
                                 include_cabinet_doors=False, 
                                 num_drawers_horizontal=1)
    lower_left_cabinet = pa.BaseCabinetAsset(width=width, 
                                            height=cabinet_height, 
                                            depth=depth, 
                                            num_drawers_vertical=0,
                                            include_cabinet_doors=True,
                                            include_foot_panel=False,
                                            lower_compartment_types=("door_left",),
                                            handle_offset=(cabinet_height * 0.35, width * 0.05))
    lower_right_cabinet = pa.BaseCabinetAsset(width=width, 
                                            height=cabinet_height, 
                                            depth=depth, 
                                            num_drawers_vertical=0,
                                            include_cabinet_doors=True,
                                            include_foot_panel=False,
                                            lower_compartment_types=("door_right",),
                                            handle_offset=(cabinet_height * 0.35, width * 0.05))
    upper_left_cabinet = pa.WallCabinetAsset(width=width, 
                                                    height=upper_cabinet_height, 
                                                    depth=upper_cabinet_depth, 
                                                    compartment_types=("door_left",),
                                                    handle_offset=(upper_cabinet_height * -0.4, width * 0.05))
    upper_right_cabinet = pa.WallCabinetAsset(width=width,
                                                    height=upper_cabinet_height, 
                                                    depth=upper_cabinet_depth, 
                                                    compartment_types=("door_right",),
                                                    handle_offset=(upper_cabinet_height * -0.4, width * 0.05))
    double_drawer = pa.drawer = pa.BaseCabinetAsset(width=width * 2, 
                                                    height=double_drawer_height, 
                                                    depth=depth, 
                                                    drawer_height=double_drawer_height, 
                                                    include_foot_panel=False, 
                                                    include_cabinet_doors=False, 
                                                    num_drawers_horizontal=1)
    
    tilted_upper_right_cabinet = pa.WallCabinetAsset(
        width=rotated_cabinet_width, 
        height=upper_cabinet_height, 
        depth=rotated_cabinet_depth, 
        compartment_types=("door_right",)
    )

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
            f'lower_right_cabinet_{cabinet_height}_{width}_{depth}_2',
            f'upper_left_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_0', 
            f'upper_right_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_0',
            f'upper_left_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_1', 
            f'upper_right_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_1',
            f'upper_left_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_2', 
            f'upper_right_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_2',
            f'upper_left_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_3', 
            f'upper_right_cabinet_{cabinet_height}_{width}_{upper_cabinet_depth}_3',
            f'double_drawer_{double_drawer_height}_{width * 2}_{depth}',
            f'titled_upper_right_cabinet_{upper_cabinet_height}_{rotated_cabinet_width}_{rotated_cabinet_depth}_0']

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
    translation[0, 3] = depth   # move 0.998 units in X
    translation[1, 3] = -depth   # move +0.998 along Y
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

    # upper cabinets
    s.add_object(
        upper_left_cabinet,
        assets[12],
        connect_parent_id=assets[5],
        connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, upper_cabinet_translation)
    )
    s.add_object(
        upper_right_cabinet,
        assets[13],
        connect_parent_id=assets[12],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0)
    )
    s.add_object(
        upper_left_cabinet,
        assets[14],
        connect_parent_id=assets[13],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0)
    )
    s.add_object(
        upper_right_cabinet,
        assets[15],
        connect_parent_id=assets[14],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0)
    )
    s.add_object(
        upper_left_cabinet,
        assets[16],
        connect_parent_id=assets[15],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0)
    )
    s.add_object(
        upper_right_cabinet,
        assets[17],
        connect_parent_id=assets[16],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0)
    )

    s.add_object(
        upper_left_cabinet,
        assets[18],
        connect_parent_id=assets[9],
        connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, upper_cabinet_translation)
    )
    s.add_object(
        upper_right_cabinet,
        assets[19],
        connect_parent_id=assets[18],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0)
    )

    # double drawer
    s.add_object(
        double_drawer,
        assets[20],
        connect_parent_id=assets[3],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    
    # tilted upper right cabinet
    theta = np.radians(45)  # Convert 45 degrees to radians
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    rotation = np.array([
        [ cos_t, sin_t, 0, 0],
        [-sin_t, cos_t, 0, 0],
        [     0,     0, 1, 0],
        [     0,     0, 0, 1]
    ])

    translation = np.eye(4)
    translation[0, 3] = upper_cabinet_depth   # move 0.998 units in X
    # translation[1, 3] = -(depth - upper_cabinet_depth)  # move +0.998 along Y
    transform = translation @ rotation

    s.add_object(
        tilted_upper_right_cabinet,
        assets[21],
        connect_parent_id=assets[12],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        transform=transform
    )

    if show:
        s.export('initial_scene/initial_scene_2.urdf')
        s.show()

    return assets

# basement_kitchen(drawer_height, width, depth, cabinet_height, upper_cabinet_depth, upper_cabinet_height, double_drawer_height, upper_cabinet_translation, s, show=True)
