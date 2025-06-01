import numpy as np
import open3d as o3d
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
from trimesh.scene.lighting import Light
import numpy as np
from scene_synthesizer.assets import BoxAsset

drawer = pa.BaseCabinetAsset(width=0.361, height = 0.178, depth=0.593, drawer_height=0.178, include_foot_panel=False, include_cabinet_doors=False, num_drawers_horizontal=1)

s = synth.Scene()
s.add_object(drawer, 'drawer')

# s.show()

s.export('exports/drawer_extracted.urdf')