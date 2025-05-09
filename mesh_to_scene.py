import trimesh
import numpy as np
from scipy.optimize import minimize
import copy
import xml.etree.ElementTree as ET
from corner_extract_obj import load_and_visualize_obj, rotate_mesh_to_face_forward, align_mesh
from corner_extract_urdf import extract_urdf_front_face_corners, load_and_visualize, get_boxes


def match_corners(obj_corners, urdf_corners):
    """Match corners by sorting both sets in the same order."""
    obj_corners = np.array(obj_corners)
    urdf_corners = np.array(urdf_corners)
    obj_sorted = obj_corners[np.lexsort((obj_corners[:,2], obj_corners[:,0]))]
    urdf_sorted = urdf_corners[np.lexsort((urdf_corners[:,2], urdf_corners[:,0]))]
    return obj_sorted, urdf_sorted


def make_frame(corners):
    """Compute a local frame (origin, u, v, normal) from four 3D corner points."""
    O = corners.mean(axis=0)
    u = corners[1] - corners[0]
    u /= np.linalg.norm(u)
    n = np.cross(corners[2] - corners[1], corners[0] - corners[1])
    n /= np.linalg.norm(n)
    v = np.cross(n, u)
    return O, u, v, n


def warp_mesh_to_urdf(obj_mesh, obj_corners, urdf_corners):
    """
    Align the front-facing quad of obj_mesh (yellow corners) to urdf_corners (green corners) with an
    in-plane affine warp (u-v plane) while preserving depth (normal) dimension.
    """
    # sort correspondences
    src_pts, tgt_pts = match_corners(obj_corners, urdf_corners)

    # build local frames
    O_src, u_src, v_src, n_src = make_frame(src_pts)
    O_tgt, u_tgt, v_tgt, n_tgt = make_frame(tgt_pts)

    # project into uv plane
    def to_uv(c, O, u, v): return np.stack([(c - O).dot(u), (c - O).dot(v)], axis=1)
    uv_src = to_uv(src_pts, O_src, u_src, v_src)
    uv_tgt = to_uv(tgt_pts, O_tgt, u_tgt, v_tgt)

    # build affine solve
    N = len(uv_src)
    G = np.zeros((2*N,6))
    b = np.zeros(2*N)
    for i, (x,y) in enumerate(uv_src):
        G[2*i]   = [x, y, 1, 0, 0, 0]
        G[2*i+1] = [0, 0, 0, x, y, 1]
        b[2*i]   = uv_tgt[i,0]
        b[2*i+1] = uv_tgt[i,1]
    params, *_ = np.linalg.lstsq(G, b, rcond=None)
    A = np.array([[params[0], params[1]], [params[3], params[4]]])
    t2 = np.array([params[2], params[5]])

    # warp each vertex
    verts = obj_mesh.vertices.copy()
    rel = verts - O_src
    coords = np.stack([rel.dot(u_src), rel.dot(v_src), rel.dot(n_src)], axis=1)
    uv = coords[:,:2]
    z  = coords[:,2:]
    uv_w = uv.dot(A.T) + t2
    warped_coords = np.concatenate([uv_w, z], axis=1)
    new_verts = (O_tgt + 
                 warped_coords[:,0:1]*u_tgt +
                 warped_coords[:,1:2]*v_tgt +
                 warped_coords[:,2:3]*n_tgt)

    warped = obj_mesh.copy()
    warped.vertices = new_verts
    return warped

def replace_drawer_geometry(input_urdf, output_urdf, mesh_path):
    """Replace drawer geometry with realistic mesh."""
    # Parse the URDF
    tree = ET.parse(input_urdf)
    root = tree.getroot()
    
    # Find the drawer link
    drawer_link = root.find(".//link[@name='lower_cabinet_drawer_0_0']")
    
    # Store the original inertial properties
    inertial = drawer_link.find('inertial')
    
    # Clear existing visual and collision elements
    for child in drawer_link.findall('visual'):
        drawer_link.remove(child)
    for child in drawer_link.findall('collision'):
        drawer_link.remove(child)
        
    # Add new mesh visual
    visual = ET.SubElement(drawer_link, 'visual')
    geometry = ET.SubElement(visual, 'geometry')
    mesh = ET.SubElement(geometry, 'mesh')
    mesh.set('filename', mesh_path)
    
    # Add material
    material = ET.SubElement(visual, 'material')
    material.set('name', 'drawer_material')
    color = ET.SubElement(material, 'color')
    color.set('rgba', '0.8 0.8 0.8 1.0')
    
    # Add simplified collision (box)
    collision = ET.SubElement(drawer_link, 'collision')
    geometry = ET.SubElement(collision, 'geometry')
    box = ET.SubElement(geometry, 'box')
    box.set('size', '0.456 0.544 0.156')  # Use original drawer dimensions
    
    # Write modified URDF
    tree.write(output_urdf, encoding='ASCII', xml_declaration=True)


def visualize_meshes_with_corners(obj_path, urdf_path, side_by_side=False, save_warped=False):
    """Display the original, the target, and the warped result together."""
    # load and align OBJ
    obj = trimesh.load(obj_path)
    obj, _ = align_mesh(obj)
    obj = rotate_mesh_to_face_forward(obj)
    obj_corners = load_and_visualize_obj(obj_path, debug_viz=False, scene_viz=False)

    # load URDF geometry & corners
    boxes = get_boxes(urdf_path)
    urdf = trimesh.util.concatenate(boxes)
    urdf_corners = extract_urdf_front_face_corners(boxes)
    R = trimesh.transformations.rotation_matrix(-np.pi/2, [1,0,0])
    urdf.apply_transform(R)
    urdf_corners = trimesh.transform_points(urdf_corners, R)

    # warp
    warped = warp_mesh_to_urdf(obj, obj_corners, urdf_corners)

    if save_warped:
        # Save the warped mesh
        _R = trimesh.transformations.rotation_matrix(np.pi/2, [1,0,0])
        warped.apply_transform(_R)
        warped.export('drawers/warped_drawer.obj')
        warped.apply_transform(R)
    
    # prepare scene
    scene = trimesh.Scene()
    
    # warped in yellow-green overlay
    warped.visual.face_colors = [255,255,0,100]
    # target URDF in red
    urdf.visual.face_colors = [255,0,0,100]

    if side_by_side:
        # Offset the warped mesh to the right
        offset = np.array([urdf.bounding_box.extents[0] * 1.5, 0, 0])
        warped.apply_translation(offset)

    # Only add warped and urdf meshes
    for m in (warped, urdf):
        scene.add_geometry(m)

    # Only show urdf corners in red
    for c in urdf_corners:
        s = trimesh.creation.icosphere(radius=0.01)
        s.apply_translation(c)
        s.visual.vertex_colors = [255,0,0,255]
        scene.add_geometry(s)

    # axes
    L = max(obj.extents)*1.3
    scene.add_geometry(trimesh.creation.axis(origin_size=0.01, axis_length=L))

    scene.show()


if __name__ == '__main__':
    obj_p  = 'scene/object_3_state_1_aligned_mesh.obj'
    urdf_p = 'exports/drawer.urdf'
    # visualize_meshes_with_corners(obj_p, urdf_p, save_warped=True)
    replace_drawer_geometry('exports/drawer.urdf', 
                       'drawers/drawer_realistic.urdf',
                       'drawers/warped_drawer.obj')