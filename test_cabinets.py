from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os
from extract_dimensions import extract_dimensions

urdf_path = 'extracted_drawer/drawer_extracted.urdf'
mesh_path = 'drawer_3.obj'
# mesh_path = 'scene/object_2_state_1_aligned_mesh.obj'
urdf_link_name = 'cabinet_drawer_0_0'

drawer = ARMDrawer(urdf_path, mesh_path, urdf_link_name)
drawer.set_urdf()
drawer.set_mesh()
# drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=False, show_points=False, show_aabb=False)

# drawer._simple_align_mesh()

# aabb = drawer.get_mesh().bounding_box
# mesh_dims = aabb.extents
# aabb = drawer.get_urdf().bounding_box
# urdf_dims = aabb.extents
# print("Mesh AABB dimensions (x, y, z):", mesh_dims)
# print("URDF AABB dimensions (x, y, z):", urdf_dims)
# drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=False, show_points=False, show_aabb=False)

drawer.extract_corners(weight_y_axis=100)
# drawer.debug_visualize(show_obj=True, show_urdf=False, show_warped=False, show_points=True, show_aabb=False)

drawer.warp(visualize_corners = False,
             visualize_uv = False,
             visualize_frames = False,
             visualize_warped_corners = False)
drawer.debug_visualize(show_obj=False, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)

warped = drawer.get_warped_mesh().copy()
dir_path = 'extracted_drawer/'
if not os.path.isdir(dir_path):
    os.makedirs(dir_path)
warped.export('extracted_drawer/warped_drawer.obj')

drawer.replace_geometry(input_urdf='extracted_drawer/drawer_extracted.urdf', output_urdf='extracted_drawer/drawer_extracted.urdf', mesh_path='extracted_drawer/warped_drawer.obj')




