import numpy as np
import open3d as o3d
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
from trimesh.scene.lighting import Light
import numpy as np
from scene_synthesizer.assets import BoxAsset

box = BoxAsset(extents=[0.3, 0.3, 0.3])

s = synth.Scene()
s.add_object(box, 'lower_two_cabinet_right_of_sink')

# s.show()

s.export('test/test4.5.urdf')