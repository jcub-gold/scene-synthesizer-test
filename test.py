import numpy as np
import open3d as o3d
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
from trimesh.scene.lighting import Light
import numpy as np
from scene_synthesizer.assets import BoxAsset
from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
import open3d as o3d

# raw_mesh_path = 'scene/object_2_state_1_aligned_mesh.obj'
# dims = extract_dimensions(raw_mesh_path)

# drawer_path = 'drawer_3.obj'
door_path = 'door_4.obj'
# drawer_dims = extract_dimensions(drawer_path)
# door_dims = extract_dimensions(door_path)
# print(drawer_dims, door_dims)

# upper_cabinet_path = 'second_floor_1_5_img/top_cabinet_left_door/object_9_mesh.obj'
# dims = extract_dimensions(upper_cabinet_path)
# print("Upper cabinet dimensions:", dims)

double_drawer_path = 'basement_1_5_img/top_cabinet_body/object_8_mesh.obj'
dims = extract_dimensions(double_drawer_path)
print("Upper cabinet dimensions:", dims)
double_drawer_path = 'basement_1_5_img/bottom_cabinet_body/object_6_mesh.obj'
dims = extract_dimensions(double_drawer_path)
print("Upper cabinet dimensions:", dims)


# # Load both meshes
# drawer_mesh = o3d.io.read_triangle_mesh(double_drawer_path)
# door_mesh = o3d.io.read_triangle_mesh(door_path)

# # Add vertex colors to distinguish the meshes (optional)
# drawer_mesh.paint_uniform_color([1, 0, 0])  # Red for drawer
# door_mesh.paint_uniform_color([0, 0, 1])    # Blue for door

# # Compute vertex normals for better visualization
# drawer_mesh.compute_vertex_normals()
# door_mesh.compute_vertex_normals()

# rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz([0, np.pi/2, 0])
# # Apply rotation to door mesh
# door_mesh.rotate(rotation_matrix, center=door_mesh.get_center())

# # Create visualizer and add both meshes
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(drawer_mesh)
# vis.add_geometry(door_mesh)

# # Optional: Set better default camera view
# opt = vis.get_render_option()
# opt.background_color = np.asarray([0.5, 0.5, 0.5])  # Gray background
# opt.show_coordinate_frame = True

# # Run the visualizer
# vis.run()
# vis.destroy_window()

# lower_large_double_cabinet = pa.BaseCabinetAsset(
#     width=0.958, 
#     height=0.333, 
#     depth=0.998, 
#     num_drawers_horizontal=1,
#     include_cabinet_doors=False,
#     include_foot_panel=False # Explicitly specify door types
# )
# s = synth.Scene()
# s.add_object(lower_large_double_cabinet, 'cabinet')

# # # s.show()

# s.export('extracted_drawer/drawer_extracted.urdf')