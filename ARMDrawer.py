# ARMDrawer.py

import numpy as np
import xml.etree.ElementTree as ET
import trimesh
from collections import defaultdict
from corner_extract_obj import align_mesh  # for AABB alignment
from trimesh.transformations import rotation_matrix
from trimesh.boolean import intersection as boolean_intersection
from AddRealisticMesh import AddRealisticMesh
from itertools import permutations, product


class ARMDrawer(AddRealisticMesh):
    def extract_corners(self, sample_count: int = 100000, weight_y_axis: float = 5.0):
        self._simple_align_mesh()
        self._refine_rotation_all_90_axes(sample_count=sample_count)
        # MESH corners extraction
        if self.mesh is None:
            raise RuntimeError("Call set_mesh() first.")
        try:
            # Project vertices onto XZ plane
            vertices = self.get_mesh().vertices
            xz = vertices[:, [0, 2]]  # columns 0 (x) and 2 (z)
            y = vertices[:, 1]       # Get Y coordinates for scoring
            print(f"Projected {len(xz)} vertices to XZ plane")

            # Split points into quadrants and find furthest point in each
            corners = []

            # Calculate distances from origin for each point in XZ plane
            xz_distances = np.linalg.norm(xz, axis=1)
            # Normalize Y coordinates to [0,1] range for scoring
            y_min, y_max = np.min(y), np.max(y)
            y_normalized = (y_max - y) / (y_max - y_min)
            scores = xz_distances * (1 + weight_y_axis * y_normalized)

            # For each quadrant in XZ plane, find point with highest score
            quadrants = [
                ((xz[:, 0] >= 0) & (xz[:, 1] >= 0)),  # Q1 (x>=0, z>=0)
                ((xz[:, 0] <  0) & (xz[:, 1] >= 0)),  # Q2 (x<0,  z>=0)
                ((xz[:, 0] <  0) & (xz[:, 1] <  0)),  # Q3 (x<0,  z<0)
                ((xz[:, 0] >= 0) & (xz[:, 1] <  0))   # Q4 (x>=0, z<0)
            ]

            for quadrant_mask in quadrants:
                if np.any(quadrant_mask):
                    # Find corner with highest score in this quadrant
                    quadrant_scores = scores * quadrant_mask
                    corner_idx = np.argmax(quadrant_scores)
                    corners.append(vertices[corner_idx])

            print(f"Found {len(corners)} corners by quadrant distance")
            self.mesh_corners = np.array(corners)

        except Exception as e:
            print(f"Error in extract_corners: {e}")
            self.mesh_corners = None


        # URDF corners extraction
        if self.urdf is None:
            raise RuntimeError("Call set_urdf() first.")
        vertices = self.get_urdf().vertices
        
        # Get extremal points
        min_x = np.min(vertices[:, 0])
        max_x = np.max(vertices[:, 0])
        max_y = np.min(vertices[:, 1])  # Get the maximum y-value for the front face
        min_z = np.min(vertices[:, 2])
        max_z = np.max(vertices[:, 2])
        
        # Find actual vertices closest to the extremal combinations
        target_points = np.array([
            [max_x, max_y, max_z],  # Top right
            [min_x, max_y, max_z],  # Top left
            [min_x, max_y, min_z],  # Bottom left
            [max_x, max_y, min_z],  # Bottom right
        ])
        
        corners = []
        for target in target_points:
            # Calculate distances to all vertices
            distances = np.linalg.norm(vertices - target[None, :], axis=1)
            # Get the closest vertex
            closest_idx = np.argmin(distances)
            corners.append(vertices[closest_idx])
        
        self.urdf_corners = np.array(corners)

    def _refine_rotation_all_90_axes(self, sample_count: int = 100000):
        """
        Instead of brute‐forcing all 24 axis swaps, we:
        1) Look at each mesh’s AABB.extents (dx, dy, dz) after AABB‐alignment & centering.
        2) Find the permutation of those three dims that best matches two of the three URDF dims—
            i.e. we compute abs(permuted_mesh_extents - urdf_extents), sort those three differences,
            and take the sum of the two smallest differences as our “score.” This discards the single
            largest‐mismatch dimension as the “outlier.”
        3) Build the corresponding permutation matrix P.
        4) Loop over the 4 possible sign‐flip matrices S = diag(±1,±1,±1) with det(S·P)=+1.
        5) For each R = S·P, apply to the OBJ, sample sample_count points, compute
            sum of nearest‐point distances to URDF, and pick the best among the 4.
        """
        if self.mesh is None or self.urdf is None:
            raise RuntimeError("Call set_mesh(), set_urdf() and simple_align_mesh() first.")

        # --- 1) Get each mesh’s AABB extents (already AABB‐aligned + centered) ---
        mesh_dims = self.mesh.bounding_box.extents   # array([dx, dy, dz])
        urdf_dims = self.urdf.bounding_box.extents   # array([Dx, Dy, Dz])

        # --- 2) Find the permutation of (dx,dy,dz) that best matches (Dx,Dy,Dz) on two dims ---
        best_perm = None
        best_score = np.inf

        # We will measure score(permutation) = sum of the two smallest entries of abs(permuted - urdf_dims).
        for perm in permutations([0, 1, 2], 3):
            permuted = np.array([
                mesh_dims[perm[0]],
                mesh_dims[perm[1]],
                mesh_dims[perm[2]]
            ])
            diffs = np.abs(permuted - urdf_dims)         # length‐3 array of absolute differences
            diffs_sorted = np.sort(diffs)                # sort ascending
            score = diffs_sorted[0] + diffs_sorted[1]    # sum of the two smallest diffs
            if score < best_score:
                best_score = score
                best_perm = perm

        # Build the 3×3 permutation matrix P so that [x';y';z'] = P @ [x;y;z]
        P = np.zeros((3, 3), dtype=np.float64)
        for row in range(3):
            col = best_perm[row]
            P[row, col] = 1.0

        print(f"Chosen permutation (mesh→URDF dims): {best_perm}")
        print(f"Mesh dims permuted: {[mesh_dims[i] for i in best_perm]}, URDF dims: {urdf_dims}")

        # --- 3) Build the 4 sign‐flip rotations R = S·P with det(R)=+1 ---
        candidate_Rs = []
        for signs in product([1.0, -1.0], repeat=3):
            S = np.diag(signs)
            R = S.dot(P)
            candidate_Rs.append(R)

        # At this point, candidate_Rs has exactly 4 matrices (each 3×3 with ±1 entries).

        best_total = np.inf
        best_mesh_aligned = None
        best_R = None

        base_mesh = self.mesh.copy()  # should already be AABB‐aligned + centered

        # --- 4) Test each of the 4 R candidates by sampling and measuring distance to URDF ---
        for idx, R3 in enumerate(candidate_Rs):
            R_hom = np.eye(4, dtype=np.float64)
            R_hom[:3, :3] = R3

            candidate = base_mesh.copy()
            candidate.apply_transform(R_hom)

            pts = candidate.sample(sample_count)            # sample `sample_count` points
            total_dist = cumulative_distance_to_mesh(pts, self.urdf)

            print(f"  [flip {idx+1}/4] total_dist = {total_dist:.6f}")

            if total_dist < best_total:
                best_total = total_dist
                best_mesh_aligned = candidate.copy()
                best_R = R3.copy()

        if best_mesh_aligned is None:
            raise RuntimeError("No valid rotation found among the 4 candidates.")

        # --- 5) Store and report the result ---
        print(f"refine_rotation_all_90_axes: best total_dist = {best_total:.6f}")
        print(f"Best 3×3 rotation matrix:\n{best_R}")

        self.mesh = best_mesh_aligned.copy()

    def _simple_align_mesh(self):
        """
        1) AABB‐minimize self.mesh via align_mesh(…).
        2) Center that aligned mesh on the world origin by shifting its bbox center to (0,0,0).
        3) Center the URDF box‐mesh on the world origin in the same way.
        """
        if self.mesh is None:
            raise RuntimeError("Call set_mesh() first.")
        if self.urdf is None:
            raise RuntimeError("Call set_urdf() first.")

        # --- 1) AABB‐minimize the mesh ---
        mesh_aligned, _ = align_mesh(self.mesh)

        # --- 2) Center the aligned mesh on origin ---
        mesh_centered, T = center_on_origin(mesh_aligned)
        self.mesh = mesh_centered.copy()

        # --- 3) Center the URDF mesh on origin the same way ---
        self.urdf, T = center_on_origin(self.urdf.copy())
        self.urdf_transformations = T

    def replace_geometry(self, input_urdf: str, output_urdf: str, mesh_path: str):
        """
        Replace the geometry of the drawer link named by self.urdf_link_name
        with a single realistic mesh, and rebuild a simplified collision box
        whose dimensions are computed from the existing collision boxes under that link.
        """
        # Parse the URDF
        tree = ET.parse(input_urdf)
        root = tree.getroot()

        # Find the drawer link dynamically
        drawer_link = root.find(f".//link[@name='{self.urdf_link_name}']")
        if drawer_link is None:
            raise RuntimeError(f"Could not find link '{self.urdf_link_name}' in URDF.")

        # Gather all <collision><geometry><box size="..."> elements under this link
        collisions = drawer_link.findall("collision")
        if not collisions:
            raise RuntimeError(f"No <collision> elements found under link '{self.urdf_link_name}'.")

        # Compute the axis-aligned bounding box (AABB) that encloses all existing boxes
        all_mins = []
        all_maxs = []

        for col in collisions:
            geom = col.find("geometry")
            if geom is None:
                continue
            box = geom.find("box")
            if box is None or "size" not in box.attrib:
                continue

            # Parse the size "sx sy sz"
            sx, sy, sz = map(float, box.get("size").split())

            # Read the <origin xyz="ox oy oz"/> if present
            origin_tag = col.find("origin")
            if origin_tag is not None and "xyz" in origin_tag.attrib:
                ox, oy, oz = map(float, origin_tag.get("xyz").split())
            else:
                ox, oy, oz = 0.0, 0.0, 0.0

            # Compute that box’s min & max corners in the link’s frame
            half = np.array([sx / 2.0, sy / 2.0, sz / 2.0])
            origin = np.array([ox, oy, oz])
            min_corner = origin - half
            max_corner = origin + half

            all_mins.append(min_corner)
            all_maxs.append(max_corner)

        if not all_mins:
            raise RuntimeError(f"Could not parse any <box size='...'> under collisions of '{self.urdf_link_name}'.")

        # Find global min/max across x,y,z
        all_mins = np.vstack(all_mins)
        all_maxs = np.vstack(all_maxs)
        global_min = all_mins.min(axis=0)
        global_max = all_maxs.max(axis=0)

        # Compute total size of the combined AABB
        total_size = global_max - global_min  # [dx, dy, dz]
        box_size_str = f"{total_size[0]:.6f} {total_size[1]:.6f} {total_size[2]:.6f}"

        # Remove all existing <visual> and <collision> nodes under this link
        for child in list(drawer_link.findall("visual")):
            drawer_link.remove(child)
        for child in list(drawer_link.findall("collision")):
            drawer_link.remove(child)

        # 1) Add new <visual> for the realistic mesh
        visual = ET.SubElement(drawer_link, "visual")
        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        mesh.set("filename", mesh_path)

        # (Optional) attach a simple gray material
        material = ET.SubElement(visual, "material")
        material.set("name", f"{self.urdf_link_name}_material")
        color = ET.SubElement(material, "color")
        color.set("rgba", "0.8 0.8 0.8 1.0")

        # 2) Add a single <collision> with a <box> of the computed size
        collision = ET.SubElement(drawer_link, "collision")
        geometry = ET.SubElement(collision, "geometry")
        box = ET.SubElement(geometry, "box")
        box.set("size", box_size_str)

        # Center the new collision box at the AABB center
        center = (global_min + global_max) / 2.0  # [cx, cy, cz]
        origin = ET.SubElement(collision, "origin")
        origin.set("xyz", f"{center[0]:.6f} {center[1]:.6f} {center[2]:.6f}")
        origin.set("rpy", "0 0 0")

        # Write updated URDF
        tree.write(output_urdf, encoding="ASCII", xml_declaration=True)


# ——————————————————————————————
# Helper functions (inside the same file)
# ——————————————————————————————
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

def cumulative_distance_to_mesh(pts: np.ndarray,
                                urdf_mesh: trimesh.Trimesh) -> float:
    closest_pts, distances, face_id = urdf_mesh.nearest.on_surface(pts)
    return float(np.sum(distances))