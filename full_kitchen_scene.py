import numpy as np
import open3d as o3d
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
from trimesh.scene.lighting import Light
import numpy as np
from scene_synthesizer.assets import BoxAsset

lower_large_double_cabinet = pa.BaseCabinetAsset(width=0.92, height = 0.88, depth=0.63, drawer_height=0.16, foot_panel_height=0.1)
lower_large_single_cabinet = pa.BaseCabinetAsset(width=0.46, height = 0.88, depth=0.63, drawer_height=0.16, foot_panel_height=0.1, num_drawers_horizontal=1)
lower_double_drawer = pa.BaseCabinetAsset(width=0.92, height = 0.38, depth=0.63, drawer_height=0.28, foot_panel_height=0.09)
sink = pa.SinkCabinetAsset(width=0.92, height = 0.92, depth=0.63, drawer_height=0.16, countertop_thickness=0.04, sink_width=0.63, sink_offset=(0,0.03))
left_box = BoxAsset(extents=[0.63, 0.63, 0.88])
lower_small_double_cabinet = pa.BaseCabinetAsset(width=0.69, height = 0.88, depth=0.63, drawer_height=0.16, foot_panel_height=0.1)
right_box = BoxAsset(extents=[0.67, 0.61, 0.88])
refrigerator = pa.RefrigeratorAsset(width=0.92, height = 2.12, depth=0.71)
upper_small_double_cabinet = pa.WallCabinetAsset(width=.86, height=.76, depth=.31)
upper_left_single_cabinet = pa.WallCabinetAsset(width=.46, height=.76, depth=.31, compartment_types=("door_left",))
upper_big_double_cabinet = pa.WallCabinetAsset(width=.92, height=.76, depth=.31)
upper_right_single_cabinet = pa.WallCabinetAsset(width=.46, height=.76, depth=.31, compartment_types=("door_right",))
right_counter = BoxAsset(extents=[4.81, 0.63, 0.04])
left_counter_left_of_sink = BoxAsset(extents=[1.55, 0.63, 0.04])
left_counter_right_of_refrigerator = BoxAsset(extents=[0.69, 0.63, 0.04])

s = synth.Scene()
s.add_object(lower_large_double_cabinet, 'lower_two_cabinet_right_of_sink')
s.add_object(
    lower_large_double_cabinet,
    'lower_two_middle_cabinet',
    connect_parent_id='lower_two_cabinet_right_of_sink',
    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    lower_large_single_cabinet,
    'lower_right_single_cabinet',
    connect_parent_id='lower_two_middle_cabinet',
    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    lower_double_drawer,
    'lower_two_drawers',
    connect_parent_id='lower_two_cabinet_right_of_sink',
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    lower_large_double_cabinet,
    'lower_two_cabinet_direct_right_of_sink',
    connect_parent_id='lower_two_drawers',
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    sink,
    'sink',
    connect_parent_id='lower_two_cabinet_direct_right_of_sink',
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    lower_large_double_cabinet,
    'lower_two_cabinet_direct_left_of_sink',
    connect_parent_id='sink',
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    left_box,
    'left_corner_box',
    connect_parent_id='lower_two_cabinet_direct_left_of_sink',
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
rotation = np.array([
    [0, -1, 0, 0],
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

s.add_object(
    lower_small_double_cabinet,
    'lower_small_double_cabinet',
    connect_parent_id='left_corner_box',
    connect_parent_anchor=('left', 'left', 'bottom'),
    connect_obj_anchor=('right', 'back', 'bottom'),
    transform=rotation
)
s.add_object(
    right_box,
    'right_corner_box',
    connect_parent_id='lower_right_single_cabinet',
    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    refrigerator,
    'refrigerator',
    connect_parent_id='lower_small_double_cabinet',
    connect_parent_anchor=('left', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    upper_small_double_cabinet,
    'upper_left_double_cabinet',
    connect_parent_id='lower_small_double_cabinet',
    connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.52)
)
s.add_object(
    upper_left_single_cabinet,
    'upper_left_single_cabinet',
    connect_parent_id='upper_left_double_cabinet',
    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
transform_with_translation = np.array([
    [0, 1, 0, 0],
    [-1, 0, 0, 0],
    [0, 0, 1, 0.52],
    [0, 0, 0, 1]
])

s.add_object(
    upper_right_single_cabinet,
    'upper_right_single_cabinet',
    connect_parent_id='right_corner_box',
    connect_parent_anchor=('right', 'back', 'top'),
    connect_obj_anchor=('left', 'back', 'bottom'),
    transform=transform_with_translation
)
s.add_object(
    upper_big_double_cabinet,
    'upper_right_big_double_cabinet',
    connect_parent_id='upper_right_single_cabinet',
    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    upper_small_double_cabinet,
    'upper_right_small_double_cabinet',
    connect_parent_id='upper_right_big_double_cabinet',
    connect_parent_anchor=('right', 'back', 'bottom'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    right_counter,
    'right_counter',
    connect_parent_id='lower_two_cabinet_direct_right_of_sink',
    connect_parent_anchor=('left', 'back', 'top'),         # top of the lower cabinet
    connect_obj_anchor=('left', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    left_counter_left_of_sink,
    'left_counter_left_of_sink',
    connect_parent_id='lower_two_cabinet_direct_left_of_sink',
    connect_parent_anchor=('right', 'back', 'top'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)
s.add_object(
    left_counter_right_of_refrigerator,
    'left_counter_right_of_refrigerator',
    connect_parent_id='lower_small_double_cabinet',
    connect_parent_anchor=('right', 'back', 'top'),         # top of the lower cabinet
    connect_obj_anchor=('right', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.0)
)

# s.show()

s.export('test/tested.urdf')