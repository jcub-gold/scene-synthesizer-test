from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import numpy as np
import math
from scene_synthesizer.assets import BoxAsset

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

def second_floor_kitchen(drawer_height: float, width: float, depth: float, cabinet_height: float, 
                     upper_cabinet_depth:float, upper_cabinet_height:float, 
                     big_drawer_height:float, upper_cabinet_translation:float, s, show=False):    
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
    big_upper_cabinet = pa.WallCabinetAsset(width=depth,
                                                    height=upper_cabinet_height, 
                                                    depth=upper_cabinet_depth, 
                                                    compartment_types=("door_left",),
                                                    handle_offset=(upper_cabinet_height * -0.4, depth * 0.05))
    big_drawer = pa.drawer = pa.BaseCabinetAsset(width=width, 
                                                    height=big_drawer_height, 
                                                    depth=depth, 
                                                    drawer_height=big_drawer_height, 
                                                    include_foot_panel=False, 
                                                    include_cabinet_doors=False, 
                                                    num_drawers_horizontal=1)
    box = BoxAsset(extents=[0.95 * depth, depth, cabinet_height + drawer_height])
    long_drawer = pa.BaseCabinetAsset(
        width=width * 2, 
        height=drawer_height, 
        depth=depth, 
        drawer_height=drawer_height, 
        include_foot_panel=False, 
        include_cabinet_doors=False, 
        num_drawers_horizontal=1,
        handle_depth=0,
        handle_height=0,
        handle_width=0
    )

    assets = []
    for i in range(11):
        assets.append(f'drawer_{drawer_height}_{width}_{depth}_{i}')
    for i in range(6):
        assets.append(f'lower_right_cabinet_{cabinet_height}_{width}_{depth}_{i}')
    for i in range(7):
        assets.append(f'lower_left_cabinet_{upper_cabinet_height}_{width}_{upper_cabinet_depth}_{i}')
    for i in range(4):
        if (i != 3):
            assets.append(f'upper_left_cabinet_{upper_cabinet_height}_{width}_{upper_cabinet_depth}_{i}')
        else:
            assets.append(f'big_upper_cabinet_{upper_cabinet_height}_{width}_{upper_cabinet_depth}_{i}')
        assets.append(f'upper_right_cabinet_{upper_cabinet_height}_{width}_{upper_cabinet_depth}_{i}')
    assets.append(f'big_drawer_{big_drawer_height}_{width}_{depth}_0')
    assets.append(f'big_drawer_{big_drawer_height}_{width}_{depth}_1')

    s.add_object(drawer, assets[0])
    s.add_object(
        drawer,
        assets[1],
        connect_parent_id=assets[0],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
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
        assets[2],
        connect_parent_id=assets[1],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        transform=transform
    )
    s.add_object(
        drawer,
        assets[3],
        connect_parent_id=assets[2],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        long_drawer,
        'long_drawer',
        connect_parent_id=assets[3],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        drawer,
        assets[4],
        connect_parent_id='long_drawer',
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
        drawer,
        assets[6],
        connect_parent_id=assets[5],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(width * 2, 0, 0.0)
    )
    for i in range(7, 11):
        s.add_object(
            drawer,
            assets[i],
            connect_parent_id=assets[i - 1],
            connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
            connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
            translation=(0, 0, 0.0)
        )
    for i in range(10):
        if (i % 2) == 0:
            s.add_object(
                lower_right_cabinet,
                assets[int(i / 2) + 11],
                connect_parent_id=assets[i],
                connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
                connect_obj_anchor=('left', 'back', 'top'),         # bottom of the upper cabinet
                translation=(0, 0, 0.0)
            )
            s.add_object(
                lower_left_cabinet,
                assets[int(i / 2) + 17],
                connect_parent_id=assets[int(i / 2) + 11],
                connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
                connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
                translation=(0, 0, 0.0)
            )
            if (i ==2):
                s.add_object(
                    lower_right_cabinet,
                    assets[16],
                    connect_parent_id=assets[int(i / 2) + 17],
                    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
                    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
                    translation=(0, 0, 0.0)
                )
                s.add_object(
                    lower_left_cabinet,
                    assets[22],
                    connect_parent_id=assets[16],
                    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
                    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
                    translation=(0, 0, 0.0)
                )
    s.add_object(
        lower_left_cabinet,
        assets[23],
        connect_parent_id=assets[21],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
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
    transform = translation @ rotation

    s.add_object(
        box,
        'box',
        connect_parent_id=assets[23],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        transform=transform
    )

    s.add_object(
        upper_right_cabinet,
        assets[25],
        connect_parent_id='box',
        connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, upper_cabinet_translation)
    )
    s.add_object(
        upper_right_cabinet,
        assets[27],
        connect_parent_id=assets[25],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        upper_left_cabinet,
        assets[24],
        connect_parent_id=assets[27],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        upper_right_cabinet,
        assets[29],
        connect_parent_id=assets[24],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        upper_left_cabinet,
        assets[26],
        connect_parent_id=assets[29],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        upper_right_cabinet,
        assets[31],
        connect_parent_id=assets[0],
        connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, upper_cabinet_translation),
    )
    s.add_object(
        upper_left_cabinet,
        assets[28],
        connect_parent_id=assets[31],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        big_upper_cabinet,
        assets[30],
        connect_parent_id=assets[28],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        big_drawer,
        assets[32],
        connect_parent_id=assets[19],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet    
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet   
        translation=(0, 0, 0.0) 
    )
    s.add_object(
        big_drawer,
        assets[33],
        connect_parent_id=assets[32],
        connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet    
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet   
        translation=(0, 0, 0.0) 
    )

    counter_0 = BoxAsset(extents=[13 * width + depth, 1.05 * depth, 0.05 * (cabinet_height + drawer_height)])
    counter_1 = BoxAsset(extents=[2 * width + depth, 1.05 * depth, 0.05 * (cabinet_height + drawer_height)])
    microwave = pa.MicrowaveAsset(width=2 * width * 0.7, height=(cabinet_height + drawer_height - big_drawer_height) * 0.7, depth=depth * 0.7)
    refrigerator = pa.RefrigeratorAsset(width=2 * width, height=(cabinet_height + drawer_height + upper_cabinet_height), depth=depth )
    s.add_object(
        counter_0,
        'counter_0',
        connect_parent_id=assets[2],
        connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        counter_1,
        'counter_1',
        connect_parent_id=assets[0],
        connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0.0)
    )
    s.add_object(
        microwave,
        'microwave',
        connect_parent_id=assets[32],
        connect_parent_anchor=('left', 'center', 'top'),         # top of the lower cabinet
        connect_obj_anchor=('left', 'center', 'bottom'),         # bottom of the upper cabinet
        translation=(2 * width * 0.15, 0, 0)
    )
    s.add_object(
        refrigerator,
        'refrigerator',
        connect_parent_id=assets[11],
        connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
        connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
        translation=(0, 0, 0)
    )

    if show:
        s.export('initial_scene/initial_scene_2.urdf')
        s.show()

    return assets

# second_floor_kitchen(drawer_height, width, depth, cabinet_height, upper_cabinet_depth, upper_cabinet_height, double_drawer_height, upper_cabinet_translation, s, show=True)
