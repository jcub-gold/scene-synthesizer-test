import numpy as np
import open3d as o3d
import scene_synthesizer as synth
from scene_synthesizer import procedural_assets as pa
from trimesh.scene.lighting import Light
import numpy as np
from scene_synthesizer.assets import BoxAsset

lower_large_double_cabinet = pa.BaseCabinetAsset(
    width=0.92, 
    height=0.78, 
    depth=0.63, 
    num_drawers_vertical=0,
    include_cabinet_doors=True,
    include_foot_panel=False,
    compartment_types=("door_left", "door_right")  # Explicitly specify door types
)
s = synth.Scene()
s.add_object(lower_large_double_cabinet, 'drawer')

s.show()

# s.export('exports/drawer_extracted.urdf')