from AddRealisticMesh import AddRealisticMesh
from ARMDrawer import ARMDrawer
import os

# urdf_path = 'exports/drawer_extracted.urdf'
# mesh_path = 'scene/object_3_state_1_aligned_mesh.obj'
# urdf_link_name = 'drawer_drawer_0_0'

# drawer = ARMDrawer(urdf_path, mesh_path, urdf_link_name)
# drawer.set_urdf()
# drawer.set_mesh()

# # drawer._simple_align_mesh()
# # print(drawer.get_urdf_transformations())
# # drawer._refine_rotation_all_90_axes(sample_count=10000)

# # aabb = drawer.get_mesh().bounding_box
# # dims = aabb.extents
# # print("AABB dimensions (x, y, z):", dims)
# # aabb = drawer.get_urdf().bounding_box
# # dims = aabb.extents
# # print("AABB dimensions (x, y, z):", dims)
# drawer.extract_corners()
# drawer.warp()
# # drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)

# warped = drawer.get_warped_mesh().copy()
# dir_path = 'drawers_extracted/'
# if not os.path.isdir(dir_path):
#     os.makedirs(dir_path)
# warped.export('drawers_extracted/warped_drawer.obj')

# drawer.replace_geometry(input_urdf='exports/drawer_extracted.urdf', output_urdf='drawers_extracted/warped_drawer.urdf', mesh_path='drawers_extracted/warped_drawer.obj')




urdf_path = 'exports/initial_scene.urdf'
mesh_path = 'scene/object_3_state_1_aligned_mesh.obj'
urdf_link_names = ['lower_cabinet_drawer_0_0', 'lower_cabinet_drawer_1_0']

for urdf_link_name in urdf_link_names:
    drawer = ARMDrawer(urdf_path, mesh_path, urdf_link_name)
    drawer.set_urdf()
    drawer.set_mesh()
    # drawer._simple_align_mesh()
    # drawer._refine_rotation_all_90_axes(sample_count=100000)
    drawer.extract_corners()
    drawer.warp()
    drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)
    drawer.extract_corners()
    drawer.warp()
    # drawer.debug_visualize(show_obj=True, show_urdf=True, show_warped=True, show_points=True, show_aabb=False)
    warped = drawer.get_warped_mesh().copy()
    dir_path = 'initial_scene/'
    if not os.path.isdir(dir_path):
        os.makedirs(dir_path)
    warped.export(f'{dir_path + urdf_link_name}.obj')
    drawer.replace_geometry(input_urdf=urdf_path, output_urdf=f'{dir_path}initial_scene.urdf', mesh_path=f'{dir_path + urdf_link_name}.obj')



# TODO: do cabinets then make an entire scene and test


