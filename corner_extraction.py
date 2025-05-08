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

def find_front_face_corners(vertices, debug_viz=False):
    print("Starting corner detection...")
    try:
        # Project vertices onto XY plane
        xy = vertices[:, :2]
        print(f"Projected {len(xy)} vertices to XY plane")
        
        # Split points into quadrants and find furthest point in each
        corners = []
        
        # Calculate distances from origin for each point
        distances = np.linalg.norm(xy, axis=1)
        
        # For each quadrant, find point furthest from origin
        # Quadrant 1 (++): x > 0, y > 0
        q1_mask = (xy[:, 0] >= 0) & (xy[:, 1] >= 0)
        if np.any(q1_mask):
            q1_idx = np.argmax(distances * q1_mask)
            corners.append(vertices[q1_idx])
            
        # Quadrant 2 (-+): x < 0, y > 0
        q2_mask = (xy[:, 0] < 0) & (xy[:, 1] >= 0)
        if np.any(q2_mask):
            q2_idx = np.argmax(distances * q2_mask)
            corners.append(vertices[q2_idx])
            
        # Quadrant 3 (--): x < 0, y < 0
        q3_mask = (xy[:, 0] < 0) & (xy[:, 1] < 0)
        if np.any(q3_mask):
            q3_idx = np.argmax(distances * q3_mask)
            corners.append(vertices[q3_idx])
            
        # Quadrant 4 (+-): x > 0, y < 0
        q4_mask = (xy[:, 0] >= 0) & (xy[:, 1] < 0)
        if np.any(q4_mask):
            q4_idx = np.argmax(distances * q4_mask)
            corners.append(vertices[q4_idx])
        
        if debug_viz:
            # Create debug visualization
            debug_scene = trimesh.Scene()
            
            # Add all projected points in gray
            projected_points = np.column_stack((xy, np.zeros(len(xy))))
            for point in projected_points:
                sphere = trimesh.creation.uv_sphere(radius=0.005)
                sphere.vertices += point
                sphere.visual.face_colors = [100, 100, 100, 100]  # Semi-transparent gray
                debug_scene.add_geometry(sphere)
            
            # Add corner points in red
            for corner in corners:
                sphere = trimesh.creation.uv_sphere(radius=0.01)
                sphere.vertices += [corner[0], corner[1], 0]  # Project to z=0
                sphere.visual.face_colors = [255, 0, 0, 255]  # Red
                debug_scene.add_geometry(sphere)
            
            # Show the debug visualization
            debug_scene.show()
        
        print(f"Found {len(corners)} corners by quadrant distance")
        return corners
        
    except Exception as e:
        print(f"Error in find_front_face_corners: {str(e)}")
        return []
    return current_mesh

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
    
    # Center mesh in X and Z axes
    vertices = mesh_rot.vertices
    
    # Center X
    x_coords = vertices[:, 0]
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    x_center = (x_max + x_min) / 2
    
    # Center Y
    y_coords = vertices[:, 1]
    y_min, y_max = np.min(y_coords), np.max(y_coords)
    y_center = (y_max + y_min) / 2
    
    # Create translation matrix for X and Y centering
    T = np.eye(4)
    T[0, 3] = -x_center  # X offset
    T[1, 3] = -y_center  # Y offset
    mesh_rot.apply_transform(T)
    
    return mesh_rot

def load_and_visualize_obj(obj_path):
    # Load the mesh
    mesh = trimesh.load(obj_path)
    
    # Align the mesh
    aligned_mesh, optimal_angles = align_mesh(mesh)
    print(f"Optimal rotation angles (x,y,z): [{optimal_angles[0]:.2f}, {optimal_angles[1]:.2f}, {optimal_angles[2]:.2f}] radians")
    
    # Rotate aligned mesh so its front face is facing +Y
    aligned_mesh = rotate_mesh_to_face_forward(aligned_mesh, target_axis=np.array([0, 1, 0]))
    
    # Create debug visualization of projected points
    debug_scene = trimesh.Scene()
    
    # Add the 3D mesh first
    debug_scene.add_geometry(aligned_mesh)
    
    # Project vertices onto XY plane and sample points (take every 10th point)
    sampled_vertices = aligned_mesh.vertices[::10]  # Sample every 10th point
    xy = sampled_vertices[:, :2]
    z = sampled_vertices[:, 2]  # Get Z coordinates for depth coloring
    
    # Normalize Z values to [0, 1] range for coloring
    z_normalized = (z - z.min()) / (z.max() - z.min())
    
    # Create colors based on depth (blue->red gradient)
    colors = np.zeros((len(z), 4))
    colors[:, 0] = z_normalized * 255  # Red channel increases with depth
    colors[:, 2] = (1 - z_normalized) * 255  # Blue channel decreases with depth
    colors[:, 3] = 255  # Alpha channel
    
    projected_points = np.column_stack((xy, np.zeros(len(xy))))
    
    # Create a single point cloud for projected points
    point_cloud = trimesh.PointCloud(projected_points)
    point_cloud.colors = colors.astype(np.uint8)
    debug_scene.add_geometry(point_cloud)
    
    
    # Add coordinate axes for reference
    axis_length = max(aligned_mesh.extents) * 1.2  # Scale axes to mesh size
    axes = trimesh.creation.axis(origin_size=0.01, axis_length=axis_length)
    debug_scene.add_geometry(axes)
    
    print(f"Showing mesh and projected points visualization ({len(xy)} points) - close window to continue")
    # debug_scene.show()
    
    # Get corners from aligned mesh
    aligned_corners = find_front_face_corners(aligned_mesh.vertices)
    
    # Create final visualization scene
    scene = trimesh.Scene()
    scene.add_geometry(aligned_mesh)
    for point in aligned_corners:
        sphere = trimesh.creation.uv_sphere(radius=0.01)
        sphere.vertices += point
        sphere.visual.face_colors = [0, 255, 0, 255]
        scene.add_geometry(sphere)

    print("Showing final mesh with corners")
    scene.show()
    return aligned_corners

if __name__ == "__main__":
    obj_path = "scene/object_3_state_1_aligned_mesh.obj"
    aligned_corners = load_and_visualize_obj(obj_path)
    if aligned_corners is not None:
        print("\nAligned mesh corner points coordinates:")
        for i, corner in enumerate(aligned_corners):
            print(f"Corner {i+1}: {corner}")