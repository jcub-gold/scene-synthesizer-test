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
from scipy.spatial import ConvexHull

# Helper: Extract the 4 corners of the front face from a mesh using the 2D face plane (exclude depth)
# Helper: Extract the 4 corners of the front face from a mesh using the 2D face plane at the front-most position (exclude depth)
def extract_front_face_corners(mesh):
    verts = mesh.vertices
    # Find the axis with the smallest spread (depth axis)
    spread = verts.max(axis=0) - verts.min(axis=0)
    depth_axis = np.argmin(spread)
    face_axes = [i for i in range(3) if i != depth_axis]
    # Find the value at the front-most position (max or min along depth axis)
    max_val = verts[:, depth_axis].max()
    min_val = verts[:, depth_axis].min()
    # Try both max and min, pick the one with more points (robust to orientation)
    mask_max = np.isclose(verts[:, depth_axis], max_val, atol=0.01 * spread[depth_axis])
    mask_min = np.isclose(verts[:, depth_axis], min_val, atol=0.01 * spread[depth_axis])
    face_points = verts[mask_max] if mask_max.sum() >= mask_min.sum() else verts[mask_min]
    verts_2d = face_points[:, face_axes]
    # Find the 4 extreme points: min/min, min/max, max/min, max/max
    min_x, max_x = verts_2d[:,0].min(), verts_2d[:,0].max()
    min_y, max_y = verts_2d[:,1].min(), verts_2d[:,1].max()
    corners_2d = np.array([
        [min_x, min_y],
        [max_x, min_y],
        [max_x, max_y],
        [min_x, max_y]
    ])
    # For each 2D corner, find the closest 3D vertex
    corners_3d = []
    for c2d in corners_2d:
        dists = np.linalg.norm(verts_2d - c2d, axis=1)
        idx = np.argmin(dists)
        corners_3d.append(face_points[idx])
    return np.array(corners_3d)


# Umeyama alignment (similarity transform)
def umeyama_alignment(src, dst):
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    U, S, Vt = np.linalg.svd(np.dot(dst_centered.T, src_centered))
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = np.dot(U, Vt)
    scale = np.trace(np.dot(np.dot(dst_centered.T, src_centered), R.T)) / np.sum(src_centered ** 2)
    t = dst_mean - scale * np.dot(R, src_mean)
    T = np.eye(4)
    T[:3, :3] = scale * R
    T[:3, 3] = t
    return T

# Helper: Extract the 4 corners of the front face from a URDF box mesh using box geometry
# For the scan mesh, use the previous logic

def extract_urdf_front_face_corners(boxes):
    all_faces = []
    for box in boxes:
        # Get box extents and transform
        size = box.extents
        T = box.primitive.transform if hasattr(box, 'primitive') else np.eye(4)
        # Find the depth axis (smallest dimension)
        depth_axis = np.argmin(size)
        face_axes = [i for i in range(3) if i != depth_axis]
        # Get 8 corners in local space
        signs = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                          [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]])
        corners_local = 0.5 * size * signs
        # Transform to world space
        corners_world = (T[:3,:3] @ corners_local.T).T + T[:3,3]
        # For both min and max along depth axis, get the face
        for which in [np.argmin, np.argmax]:
            val = which(corners_world[:, depth_axis])
            mask = np.isclose(corners_world[:, depth_axis], corners_world[val, depth_axis])
            face_pts = corners_world[mask]
            if len(face_pts) == 4:
                # Area for ranking
                area = np.linalg.norm(np.cross(face_pts[1] - face_pts[0], face_pts[3] - face_pts[0]))
                all_faces.append((area, face_pts))
    if not all_faces:
        raise RuntimeError("Could not find a suitable URDF front face.")
    all_faces.sort(reverse=True)
    return all_faces[0][1]

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
    size = np.array(list(map(float, geo.attrib["size"].split())))
    box = trimesh.creation.box(extents=size)
    origin = visual.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if origin is not None:
        if "xyz" in origin.attrib:
            xyz = np.array(list(map(float, origin.attrib["xyz"].split())))
        if "rpy" in origin.attrib:
            rpy = np.array(list(map(float, origin.attrib["rpy"].split())))
    T = trimesh.transformations.euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")
    T[:3,3] = xyz
    box.apply_transform(T)
    boxes.append(box)

target_drawer = trimesh.util.concatenate(boxes)
scan = trimesh.load("scene/object_3_state_1_aligned_mesh.obj")

# Extract the true front face corners for both meshes
corners_urdf = extract_urdf_front_face_corners(boxes)
corners_scan = extract_front_face_corners(scan)

print("URDF front face corners:\n", corners_urdf)
print("Scan front face corners:\n", corners_scan)

# Visualize the chosen corners for URDF mesh only
scene_urdf = trimesh.Scene()
scene_urdf.add_geometry(target_drawer, node_name="URDF_drawer")
for i, pt in enumerate(corners_urdf):
    sphere = trimesh.creation.icosphere(radius=0.01, subdivisions=2)
    sphere.apply_translation(pt)
    sphere.visual.vertex_colors = [0, 255, 0, 255]  # green for URDF
    scene_urdf.add_geometry(sphere, node_name=f"urdf_corner_{i}")
scene_urdf.show()

# Visualize the chosen corners for scan mesh only
scene_scan = trimesh.Scene()
scene_scan.add_geometry(scan.copy(), node_name="scan_mesh")
for i, pt in enumerate(corners_scan):
    sphere = trimesh.creation.icosphere(radius=0.01, subdivisions=2)
    sphere.apply_translation(pt)
    sphere.visual.vertex_colors = [255, 0, 0, 255]  # red for scan
    scene_scan.add_geometry(sphere, node_name=f"scan_corner_{i}")
scene_scan.show()

# Visualize both together (before alignment)
corner_spheres = []
for pt in corners_urdf:
    sphere = trimesh.creation.icosphere(radius=0.01, subdivisions=2)
    sphere.apply_translation(pt)
    sphere.visual.vertex_colors = [0, 255, 0, 255]  # green for URDF
    corner_spheres.append(sphere)
for pt in corners_scan:
    sphere = trimesh.creation.icosphere(radius=0.01, subdivisions=2)
    sphere.apply_translation(pt)
    sphere.visual.vertex_colors = [255, 0, 0, 255]  # red for scan
    corner_spheres.append(sphere)
scene_both = trimesh.Scene()
scene_both.add_geometry(target_drawer, node_name="URDF_drawer")
scene_both.add_geometry(scan.copy(), node_name="scan_mesh")
for i, sph in enumerate(corner_spheres):
    scene_both.add_geometry(sph, node_name=f"corner_{i}")
scene_both.show()

# Compute similarity transform
T = umeyama_alignment(corners_scan, corners_urdf)

# Apply to scan
scan.apply_transform(T)

# Visualize final alignment
scene = trimesh.Scene()
target_drawer.visual.vertex_colors[:] = [0,200,0,80]
if isinstance(scan.visual, trimesh.visual.texture.TextureVisuals):
    scan.visual = trimesh.visual.ColorVisuals(mesh=scan, vertex_colors=[200,200,200,255])
else:
    scan.visual.vertex_colors[:] = [200,200,200,255]
scene.add_geometry(target_drawer, node_name="URDF_drawer")
scene.add_geometry(scan, node_name="aligned_scan")
for i, sph in enumerate(corner_spheres):
    scene.add_geometry(sph, node_name=f"corner_{i}")
scene.show()

scan.export("scene/aligned_drawer.obj")
