import trimesh
import numpy as np
from scipy.optimize import minimize
import copy
import xml.etree.ElementTree as ET
from corner_extract_obj import load_and_visualize_obj, rotate_mesh_to_face_forward, align_mesh
from corner_extract_urdf import extract_urdf_front_face_corners, load_and_visualize, get_boxes

def match_corners(obj_corners, urdf_corners):
    """Match corners by sorting both sets in the same order."""
    # Convert inputs to numpy arrays if they aren't already
    obj_corners = np.array(obj_corners)
    urdf_corners = np.array(urdf_corners)
    
    # First sort corners by x coordinate, then by z coordinate
    obj_sorted = obj_corners[np.lexsort((obj_corners[:,2], obj_corners[:,0]))]
    urdf_sorted = urdf_corners[np.lexsort((urdf_corners[:,2], urdf_corners[:,0]))]
    
    # Now we have corners sorted in same order: left to right, bottom to top
    return obj_sorted, urdf_sorted

def scale_mesh_to_urdf(obj_mesh, obj_corners, urdf_corners):
    """Scale and transform mesh to align corners while preserving depth."""
    # Get full 3D bounding boxes
    obj_bounds = obj_mesh.bounds
    obj_3d_dims = obj_bounds[1] - obj_bounds[0]
    
    # Get URDF bounding box
    urdf_bounds = np.array([
        np.min(urdf_corners, axis=0),
        np.max(urdf_corners, axis=0)
    ])
    urdf_3d_dims = urdf_bounds[1] - urdf_bounds[0]
    
    # Calculate scale factors for XY plane (width and height)
    scale_xy = np.array([
        urdf_3d_dims[0] / obj_3d_dims[0],  # x scale
        1.0,                                # y scale (depth) - preserve original
        urdf_3d_dims[2] / obj_3d_dims[2]   # z scale
    ])
    
    # Create scaling matrix
    S = np.diag([scale_xy[0], scale_xy[1], scale_xy[2], 1.0])
    
    # Calculate centroids for alignment
    obj_centroid = np.mean(obj_corners, axis=0)
    urdf_centroid = np.mean(urdf_corners, axis=0)
    
    # Create translation matrices
    T1 = np.eye(4)  # Move to origin
    T1[:3, 3] = -obj_centroid
    
    T2 = np.eye(4)  # Move to target position
    T2[:3, 3] = urdf_centroid
    
    # Combine transformations
    transform = T2 @ S @ T1
    
    # Apply transform to mesh
    warped_mesh = obj_mesh.copy()
    warped_mesh.apply_transform(transform)
    
    return warped_mesh, transform

def center_mesh_and_corners(mesh, corners):
    """Center mesh and corners at origin."""
    # Get centroid of mesh
    centroid = mesh.centroid
    
    # Create translation matrix to move to origin
    T = np.eye(4)
    T[:3, 3] = -centroid
    
    # Center mesh
    centered_mesh = mesh.copy()
    centered_mesh.apply_transform(T)
    
    # Center corners
    centered_corners = corners - centroid
    
    return centered_mesh, centered_corners

def visualize_meshes_with_corners(obj_path, urdf_path):
    """Display both meshes and their corners with proper alignment."""
    # Load and process OBJ
    obj_mesh = trimesh.load(obj_path)
    obj_mesh, optimal_angles = align_mesh(obj_mesh)
    obj_mesh = rotate_mesh_to_face_forward(obj_mesh)
    obj_corners = load_and_visualize_obj(obj_path, debug_viz=False, scene_viz=False)
    
    # Load URDF
    boxes = get_boxes(urdf_path)
    urdf_mesh = trimesh.util.concatenate(boxes)
    urdf_corners = extract_urdf_front_face_corners(boxes)
    
    # Rotate URDF mesh and corners 90 degrees around X axis
    R = trimesh.transformations.rotation_matrix(-np.pi/2, [1, 0, 0])
    urdf_mesh.apply_transform(R)
    urdf_corners = trimesh.transform_points(urdf_corners, R)
    
    # Create visualization scene
    scene = trimesh.Scene()
    
    # Add OBJ mesh in semi-transparent blue
    obj_mesh.visual.face_colors = [0, 0, 255, 100]
    scene.add_geometry(obj_mesh)
    
    # Add URDF mesh in semi-transparent red
    urdf_mesh.visual.face_colors = [255, 0, 0, 100]
    scene.add_geometry(urdf_mesh)
    
    # Add OBJ corner points in yellow
    for corner in obj_corners:
        sphere = trimesh.creation.icosphere(radius=0.01)
        sphere.apply_translation(corner)
        sphere.visual.vertex_colors = [255, 255, 0, 255]
        scene.add_geometry(sphere)
    
    # Add URDF corner points in green
    for corner in urdf_corners:
        sphere = trimesh.creation.icosphere(radius=0.01)
        sphere.apply_translation(corner)
        sphere.visual.vertex_colors = [0, 255, 0, 255]
        scene.add_geometry(sphere)
    
    # Add coordinate axes for reference
    axis_length = max(obj_mesh.extents) * 1.2
    axes = trimesh.creation.axis(origin_size=0.01, axis_length=axis_length)
    scene.add_geometry(axes)
    
    scene.show()

if __name__ == "__main__":
    obj_path = "scene/object_3_state_1_aligned_mesh.obj"
    urdf_path = "exports/drawer.urdf"
    visualize_meshes_with_corners(obj_path, urdf_path)