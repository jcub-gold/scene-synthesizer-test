# # Drawer Dimensions: Width: 0.37, Height: 0.17, Depth: 0.58
# # Lower Cabinet Dimensions: Width: 0.42, Height: 0.59, Depth: 0.05
# # Upper Cabinet Dimensions: Width: 0.40, Height: 0.88, Depth: 0.07
# import numpy as np
# import open3d as o3d
# import scene_synthesizer as synth
# from scene_synthesizer import procedural_assets as pa
# from trimesh.scene.lighting import Light
# import numpy as np


# drawer = lower_large_double_cabinet = pa.BaseCabinetAsset(width=0.46, height = 0.16, depth=0.63, drawer_height=0.16, include_foot_panel=False, include_cabinet_doors=False, num_drawers_horizontal=1)
# s = synth.Scene()
# s.add_object(drawer, 'lower_cabinet')
# # s.show()

# s.export('exports/drawer.urdf')

import xml.etree.ElementTree as ET
import trimesh
import numpy as np

# 1) Parse URDF by hand
tree = ET.parse("exports/drawer.urdf")
root = tree.getroot()

# find the drawer link specifically
link = root.find("./link[@name='lower_cabinet_drawer_0_0']")

# collect all box‐visuals for the drawer
boxes = []
for visual in link.findall("visual"):
    geo = visual.find("geometry/box")
    if geo is None: 
        continue

    # extract size
    size = np.array(list(map(float, geo.attrib["size"].split())))
    # make the box
    box = trimesh.creation.box(extents=size)

    # extract the origin transform
    origin = visual.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if origin is not None:
        if "xyz" in origin.attrib:
            xyz = np.array(list(map(float, origin.attrib["xyz"].split())))
        if "rpy" in origin.attrib:
            rpy = np.array(list(map(float, origin.attrib["rpy"].split())))

    # build a 4×4 from rpy+xyz
    T = trimesh.transformations.euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")
    T[:3,3] = xyz
    box.apply_transform(T)
    boxes.append(box)

# Create the target drawer mesh
target_drawer = trimesh.util.concatenate(boxes)

# 2) load & align your scan
scan = trimesh.load("scene/object_3_state_1_aligned_mesh.obj")

# Get oriented bounding boxes
cb = scan.bounding_box_oriented
tb = target_drawer.bounding_box_oriented

# 1. Center scan at the origin
scan.apply_translation(-cb.centroid)

# 2. Align orientation using OBB rotation
R = tb.primitive.transform[:3, :3] @ np.linalg.inv(cb.primitive.transform[:3, :3])
T_rot = np.eye(4)
T_rot[:3, :3] = R
scan.apply_transform(T_rot)

# 3. Scale uniformly to fit
cb = scan.bounding_box_oriented  # recalculate after rotation
scale_factors = tb.extents / cb.extents
s = scale_factors.min()
S = np.eye(4) * s
S[3, 3] = 1
scan.apply_transform(S)

# 4. Move to target centroid
scan.apply_translation(tb.centroid)

# 3) visualize in one shot
scene = trimesh.Scene()

# Make the target drawer semi-transparent green
target_drawer.visual.vertex_colors[:] = [0,200,0,80]

# For the scan, convert TextureVisuals to VertexColors if needed
if isinstance(scan.visual, trimesh.visual.texture.TextureVisuals):
    scan.visual = trimesh.visual.ColorVisuals(mesh=scan, vertex_colors=[200,200,200,255])
else:
    scan.visual.vertex_colors[:] = [200,200,200,255]

scene.add_geometry(target_drawer, node_name="URDF_drawer")
scene.add_geometry(scan, node_name="aligned_scan")
scene.show()

# Save the aligned scan
scan.export("scene/aligned_drawer.obj")
