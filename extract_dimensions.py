import trimesh
import numpy as np
from corner_extract_obj import align_mesh


def extract_dimensions(mesh_path):
    mesh = trimesh.load(mesh_path)

    """
    1) AABB‐minimize self.mesh via align_mesh(…).
    2) Center that aligned mesh on the world origin by shifting its bbox center to (0,0,0).
    3) Center the URDF box‐mesh on the world origin in the same way.
    """

    # --- 1) AABB‐minimize the mesh ---
    mesh_aligned, _ = align_mesh(mesh)

    # --- 2) Center the aligned mesh on origin ---
    mesh_centered, T = center_on_origin(mesh_aligned)
    mesh = mesh_centered.copy()

    aabb = mesh.bounding_box
    dims = aabb.extents
    return dims


def center_on_origin(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """
    Translate `mesh` so that its AABB center sits at (0,0,0).
    We compute the axis‐aligned bounding box, take (min + max) / 2 for each axis,
    and subtract that vector from every vertex. Returns a new Trimesh.
    """
    # 1) Compute axis‐aligned bounding box corners
    aabb = mesh.bounds   # shape (2,3): [ [min_x, min_y, min_z], [max_x, max_y, max_z] ]
    min_corner = aabb[0]
    max_corner = aabb[1]

    # 2) Compute center = (min + max) / 2
    center = (min_corner + max_corner) * 0.5

    # 3) Translate every vertex by –center
    T = np.eye(4)
    T[:3, 3] = -center

    recentred = mesh.copy()
    recentred.apply_transform(T)
    return recentred, T