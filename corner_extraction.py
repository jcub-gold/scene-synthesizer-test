import trimesh
import numpy as np
from scipy.optimize import minimize
import copy

def compute_aabb_volume(angles, mesh):
    # Create rotation matrix for all three angles (x, y, z)
    R = trimesh.transformations.euler_matrix(angles[0], angles[1], angles[2])
    rotated_mesh = copy.deepcopy(mesh)
    rotated_mesh.apply_transform(R)
    aabb = rotated_mesh.bounding_box
    return aabb.volume

def align_mesh(mesh):
    # Try multiple starting points to avoid local minima
    starting_angles = [
        [0, 0, 0],        # No rotation
        [np.pi/2, 0, 0],  # 90° around x
        [0, np.pi/2, 0],  # 90° around y
        [0, 0, np.pi/2],  # 90° around z
        [np.pi/4, np.pi/4, np.pi/4]  # 45° all axes
    ]
    
    best_volume = float('inf')
    best_angles = None
    best_mesh = None
    
    for start in starting_angles:
        # Optimize rotation to minimize AABB volume
        result = minimize(
            lambda x: compute_aabb_volume(x, mesh),
            x0=start,
            method='Powell',
            options={'maxiter': 1000}
        )
        
        volume = compute_aabb_volume(result.x, mesh)
        if volume < best_volume:
            best_volume = volume
            best_angles = result.x
            
            # Create the best rotated mesh
            R = trimesh.transformations.euler_matrix(best_angles[0], best_angles[1], best_angles[2])
            best_mesh = copy.deepcopy(mesh)
            best_mesh.apply_transform(R)
    
    return best_mesh, best_angles

def find_front_face_corners(vertices):
    # Project vertices onto XY plane
    xy = vertices[:, :2]
    # Find min/max X and Y
    min_x_idx = np.argmin(xy[:, 0])
    max_x_idx = np.argmax(xy[:, 0])
    min_y_idx = np.argmin(xy[:, 1])
    max_y_idx = np.argmax(xy[:, 1])
    # Get the corresponding 3D points
    corners = [
        vertices[min_x_idx],
        vertices[max_x_idx],
        vertices[min_y_idx],
        vertices[max_y_idx],
    ]
    # Remove duplicates (in case min/max overlap)
    unique_corners = []
    for c in corners:
        if not any(np.allclose(c, uc) for uc in unique_corners):
            unique_corners.append(c)
    return unique_corners

def rotate_mesh_to_face_forward(mesh, target_axis=np.array([0, 1, 0])):
    # Find the normal of the front face (largest area face)
    face_areas = mesh.area_faces
    largest_face_idx = np.argmax(face_areas)
    normal = mesh.face_normals[largest_face_idx]
    normal = normal / np.linalg.norm(normal)
    target_axis = target_axis / np.linalg.norm(target_axis)
    # Compute rotation axis and angle
    axis = np.cross(normal, target_axis)
    if np.linalg.norm(axis) < 1e-8:
        mesh_rot = mesh.copy()
    else:
        axis = axis / np.linalg.norm(axis)
        angle = np.arccos(np.clip(np.dot(normal, target_axis), -1.0, 1.0))
        # Create rotation matrix
        R = trimesh.transformations.rotation_matrix(angle, axis)
        mesh_rot = mesh.copy()
        mesh_rot.apply_transform(R)
    # Apply 90 degree rotation around Y axis (in positive direction)
    R_y90 = trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0])
    mesh_rot.apply_transform(R_y90)
    
    # Apply 180 degree rotation around X axis to flip upright
    R_x180 = trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])
    mesh_rot.apply_transform(R_x180)
    
    return mesh_rot

def load_and_visualize_obj(obj_path):
    # Load the mesh
    mesh = trimesh.load(obj_path)
    
    # Align the mesh
    aligned_mesh, optimal_angles = align_mesh(mesh)
    print(f"Optimal rotation angles (x,y,z): [{optimal_angles[0]:.2f}, {optimal_angles[1]:.2f}, {optimal_angles[2]:.2f}] radians")
    
    # Rotate aligned mesh so its front face is facing +Y
    aligned_mesh = rotate_mesh_to_face_forward(aligned_mesh, target_axis=np.array([0, 1, 0]))
    
    # Get corners from aligned mesh
    aligned_corners = find_front_face_corners(aligned_mesh.vertices)
    
    # Create visualization scene
    scene = trimesh.Scene()
    
    # Add aligned mesh and its corners (green)
    scene.add_geometry(aligned_mesh)
    for point in aligned_corners:
        sphere = trimesh.creation.uv_sphere(radius=0.01)
        sphere.vertices += point
        sphere.visual.face_colors = [0, 255, 0, 255]  # Green
        scene.add_geometry(sphere)

    # Show the scene
    scene.show()
    return aligned_corners

if __name__ == "__main__":
    obj_path = "scene/object_3_state_1_aligned_mesh.obj"
    aligned_corners = load_and_visualize_obj(obj_path)
    if aligned_corners is not None:
        print("\nAligned mesh corner points coordinates:")
        for i, corner in enumerate(aligned_corners):
            print(f"Corner {i+1}: {corner}")