import open3d as o3d
import os
import glob

# Get all .obj files in the scene folder
obj_files = glob.glob("scene/*.obj")

for obj_file in obj_files:
    try:
        # Load the mesh
        mesh = o3d.io.read_triangle_mesh(obj_file)
        mesh.compute_vertex_normals()

        # Uniformly sample points from mesh surface
        pcd = mesh.sample_points_uniformly(number_of_points=10000)

        # Create output filename by replacing .obj with .pcd
        pcd_file = obj_file.replace('.obj', '.pcd')
        
        # Save the point cloud
        o3d.io.write_point_cloud(pcd_file, pcd)
        print(f"Processed {obj_file} -> {pcd_file}")
        
        # Optional: visualize each point cloud
        o3d.visualization.draw_geometries([pcd])
        
    except Exception as e:
        print(f"Error processing {obj_file}: {str(e)}")