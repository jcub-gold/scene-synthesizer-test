import open3d as o3d

# Load OBJ mesh
mesh = o3d.io.read_triangle_mesh("your_model.obj")
mesh.compute_vertex_normals()

# Load PCD
pcd = o3d.io.read_point_cloud("your_point_cloud.pcd")

# Visualize both
o3d.visualization.draw_geometries([mesh, pcd])
