# Drawer Dimensions: Width: 0.37, Height: 0.17, Depth: 0.58
# Lower Cabinet Dimensions: Width: 0.42, Height: 0.59, Depth: 0.05
# Upper Cabinet Dimensions: Width: 0.40, Height: 0.88, Depth: 0.07
import numpy as np
import open3d as o3d
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
from trimesh.scene.lighting import Light
import numpy as np


# Obtained from aligned PCD's AABB
drawer_height = 0.17
lower_cabinet_height = 0.59
lower_cabinet_foot_panel_height = (drawer_height + lower_cabinet_height) * 0.045 # proportional to default setting
lower_cabinet_width_est = 0.4 * 2
lower_cabinet_height_est = drawer_height + lower_cabinet_height + lower_cabinet_foot_panel_height
lower_cabinet_depth_est = 0.58

print(lower_cabinet_height_est, lower_cabinet_width_est, lower_cabinet_depth_est)

upper_cabinet_width_est = 0.4 * 2
upper_cabinet_height_est = 0.88
upper_cabinet_depth_est = 0.4 * (upper_cabinet_width_est + upper_cabinet_height_est) / 2 # proportional to default setting

print(upper_cabinet_height_est, upper_cabinet_width_est, upper_cabinet_depth_est)

lower_cabinet = pa.BaseCabinetAsset(width=lower_cabinet_width_est, height=lower_cabinet_height_est, depth=lower_cabinet_depth_est, drawer_height=drawer_height, foot_panel_height=lower_cabinet_foot_panel_height)
upper_cabinet = pa.WallCabinetAsset(width=upper_cabinet_width_est, height=upper_cabinet_height_est, depth=upper_cabinet_depth_est)

s = synth.Scene()
s.add_object(lower_cabinet, 'lower_cabinet')
s.add_object(
    upper_cabinet,
    'upper_cabinet',
    connect_parent_id='lower_cabinet',
    connect_parent_anchor=('center', 'back', 'top'),         # top of the lower cabinet
    connect_obj_anchor=('center', 'back', 'bottom'),         # bottom of the upper cabinet
    translation=(0, 0, 0.2)
)
# s.show()

s.export('exports/initial_scene.urdf')

