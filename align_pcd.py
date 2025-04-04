import open3d as o3d
import numpy as np
from scipy.optimize import minimize
import glob
import os
import copy

def compute_aabb_volume(angles, pcd):
    # Create rotation matrix for all three angles (x, y, z)
    R = pcd.get_rotation_matrix_from_xyz(angles)
    rotated_pcd = copy.deepcopy(pcd)
    rotated_pcd.rotate(R)
    aabb = rotated_pcd.get_axis_aligned_bounding_box()
    return aabb.volume()

def align_pcd(pcd):
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
    best_pcd = None
    
    for start in starting_angles:
        # Optimize rotation to minimize AABB volume
        result = minimize(
            lambda x: compute_aabb_volume(x, pcd),
            x0=start,
            method='Powell',  # Powell method works well for this type of optimization
            options={'maxiter': 1000}
        )
        
        volume = compute_aabb_volume(result.x, pcd)
        if volume < best_volume:
            best_volume = volume
            best_angles = result.x
            
            # Create the best rotated point cloud
            R = pcd.get_rotation_matrix_from_xyz(best_angles)
            best_pcd = copy.deepcopy(pcd)
            best_pcd.rotate(R)
    
    return best_pcd, best_angles

def process_point_cloud(pcd_path):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    # Get the filename without extension for display
    filename = os.path.basename(pcd_path)
    print(f"\nProcessing: {filename}")
    
    # Align the point cloud
    aligned_pcd, optimal_angle = align_pcd(pcd)
    # Convert numpy array to list for printing
    angle_list = optimal_angle.tolist()
    print(f"Optimal rotation angles (x,y,z): [{angle_list[0]:.2f}, {angle_list[1]:.2f}, {angle_list[2]:.2f}] radians")
    
    # Calculate AABB and OBB for aligned point cloud
    aabb = aligned_pcd.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)  # Red for AABB
    
    obb = aligned_pcd.get_oriented_bounding_box()
    obb.color = (0, 1, 0)  # Green for OBB
    
    # Print bounding box information with proper array handling
    print(f"Original AABB volume: {pcd.get_axis_aligned_bounding_box().volume():.6f}")
    print(f"Aligned AABB volume: {aabb.volume():.6f}")
    print(f"AABB min bound: [{', '.join(f'{x:.4f}' for x in aabb.min_bound)}]")
    print(f"AABB max bound: [{', '.join(f'{x:.4f}' for x in aabb.max_bound)}]")
    
    print(f"OBB center: [{', '.join(f'{x:.4f}' for x in obb.center)}]")
    print(f"OBB extent: [{', '.join(f'{x:.4f}' for x in obb.extent)}]")
    print(f"OBB volume: {obb.volume():.6f}")
    
    # Save aligned point cloud
    aligned_path = pcd_path.replace('.pcd', '_aligned.pcd')
    o3d.io.write_point_cloud(aligned_path, aligned_pcd)
    print(f"Saved aligned point cloud to: {aligned_path}")
    
    # Visualize both original and aligned point clouds with bounding boxes
    orig_aabb = pcd.get_axis_aligned_bounding_box()
    orig_aabb.color = (0, 0, 1)  # Blue for original AABB
    
    o3d.visualization.draw_geometries([
        pcd, orig_aabb,  # Original point cloud and AABB
        aligned_pcd, aabb, obb  # Aligned point cloud with both boxes
    ])
    
    return aabb, obb, aligned_pcd

def main():
    # Get the script's directory
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Find all PCD files in the scene directory
    pcd_files = glob.glob(os.path.join(base_dir, "scene", "*.pcd"))
    
    if not pcd_files:
        print("No PCD files found in the scene directory!")
        return
    
    # Process each PCD file
    for pcd_path in pcd_files:
        try:
            aabb, obb, aligned_pcd = process_point_cloud(pcd_path)
        except Exception as e:
            print(f"Error processing {pcd_path}: {str(e)}")

if __name__ == "__main__":
    main()