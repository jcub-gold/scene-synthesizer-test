import open3d as o3d
import os

folder_path = "scene"
files = [f for f in os.listdir(folder_path) if f.endswith(('.ply', '.obj'))]

geometries = []
for file in files:
    mesh = o3d.io.read_triangle_mesh(os.path.join(folder_path, file))
    mesh.compute_vertex_normals()
    geometries.append(mesh)

o3d.visualization.draw_geometries(geometries)
