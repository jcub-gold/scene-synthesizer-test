import open3d as o3d
import numpy as np
import glob
import os

def get_aabb_dimensions(pcd_path):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    # Get AABB
    aabb = pcd.get_axis_aligned_bounding_box()
    
    # Calculate dimensions
    dimensions = aabb.max_bound - aabb.min_bound
    
    # Get filename for display
    filename = os.path.basename(pcd_path)
    
    print(f"\nFile: {filename}")
    print(f"Dimensions (x, y, z): [{', '.join(f'{x:.4f}' for x in dimensions)}]")
    print(f"Width (x): {dimensions[0]:.4f}")
    print(f"Height (y): {dimensions[1]:.4f}")
    print(f"Depth (z): {dimensions[2]:.4f}")
    
    return dimensions

def main():
    # Get the script's directory
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Find all aligned PCD files
    aligned_pcds = glob.glob(os.path.join(base_dir, "scene", "*_aligned.pcd"))
    
    if not aligned_pcds:
        print("No aligned PCD files found in the scene directory!")
        return
    
    # Store results
    results = {}
    
    # Process each aligned PCD file
    for pcd_path in aligned_pcds:
        try:
            dimensions = get_aabb_dimensions(pcd_path)
            results[os.path.basename(pcd_path)] = dimensions
        except Exception as e:
            print(f"Error processing {pcd_path}: {str(e)}")
    
    # Print summary
    print("\nSummary of all objects:")
    print("-" * 50)
    for filename, dims in results.items():
        print(f"{filename}: {dims[0]:.4f} x {dims[1]:.4f} x {dims[2]:.4f}")

if __name__ == "__main__":
    main()