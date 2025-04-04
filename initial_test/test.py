import numpy as np
import open3d as o3d
import scene_synthesize as synth
from scene_synthesize import procedural_assets as pa
from trimesh.scene.lighting import Light
import numpy as np

# pcd = o3d.io.read_point_cloud("synthetic_drawer.pcd")

# angle = np.pi / 9
# R = np.array([
#     [1, 0, 0],
#     [0, np.cos(angle), -np.sin(angle)],
#     [0, np.sin(angle), np.cos(angle)]
# ])

# pcd.rotate(R, center=(0,0,0))

# aabb = pcd.get_axis_aligned_bounding_box()
# aabb.color = (1, 0, 0)

# obb = pcd.get_oriented_bounding_box()
# obb.color = (0, 1, 0)

cabinet = pa.BaseCabinetAsset(width=1.0)
s = synth.Scene()
s.add_object(cabinet, 'cabinet_1')
s.add_object(
    cabinet,
    'cabinet_2',
    connect_parent_id='cabinet_1',
    connect_parent_anchor=('right', 'back', 'bottom'),
    connect_obj_anchor=('left', 'back', 'bottom')
)

s.colorize()
s.show()