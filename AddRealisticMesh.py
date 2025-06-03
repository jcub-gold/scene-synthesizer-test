import trimesh
import numpy as np
import xml.etree.ElementTree as ET
from matplotlib import pyplot as plt

class AddRealisticMesh:
    def __init__(self, urdf_path, mesh_path, urdf_link_name):
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path
        self.urdf_link_name = urdf_link_name

        self.urdf = None
        self.mesh = None
        self.urdf_corners = None
        self.mesh_corners = None
        self.warped_mesh = None
        self.warped_mesh_corners = None
        self.urdf_transformations = np.eye(4)

    def set_urdf(self):
        assert (self.urdf_path != None and self.urdf_link_name != None), "initialize urdf path or link name"

        # Parse URDF
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        link = root.find(f"./link[@name='{self.urdf_link_name}']")

        # Collect all box-visuals
        boxes = []
        for visual in link.findall("visual"):
            geo = visual.find("geometry/box")
            if geo is None: 
                continue
            size = np.array(list(map(float, geo.attrib["size"].split())))
            box = trimesh.creation.box(extents=size)
            
            # Get transform from URDF
            origin = visual.find("origin")
            xyz = np.zeros(3)
            rpy = np.zeros(3)
            if origin is not None:
                if "xyz" in origin.attrib:
                    xyz = np.array(list(map(float, origin.attrib["xyz"].split())))
                if "rpy" in origin.attrib:
                    rpy = np.array(list(map(float, origin.attrib["rpy"].split())))
            T = trimesh.transformations.euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")
            T[:3,3] = xyz
            box.apply_transform(T)
            boxes.append(box)

        self.urdf = trimesh.util.concatenate(boxes)

    def set_mesh(self):
        self.mesh = obj = trimesh.load(self.mesh_path)

    def extract_corners(self):
        pass

    def warp(self,
             visualize_corners: bool = False,
             visualize_uv: bool = False,
             visualize_frames: bool = False,
             visualize_warped_corners: bool = False):
        """
        Extended warp() that can draw intermediate steps.
        
        Flags:
          - visualize_corners:   shows 3D scene with red/green spheres + thick yellow tubes
          - visualize_uv:        shows a 2D UV‐scatter (OBJ corners in red, URDF corners in blue)
          - visualize_frames:    shows the local (u,v,n) axes at O_src and O_tgt
          - visualize_warped_corners: plots where the four warped corners land, in 3D, versus the URDF corners 
        """
        assert (
            self.urdf is not None and
            self.mesh is not None and
            self.urdf_corners is not None and
            self.mesh_corners is not None
        ), "Must call set_urdf(), set_mesh(), extract_corners() first."

        def make_frame(corners: np.ndarray):
            """
            Given 4 points (4×3), compute:
              O = centroid,
              u = normalized (corner1 – corner0),
              n = normalized cross((corner2 – corner1), (corner0 – corner1)),
              v = cross(n, u).
            Return O, u, v, n (each shape (3,)).
            """
            O = corners.mean(axis=0)
            u = corners[1] - corners[0]
            u = u / np.linalg.norm(u)
            n = np.cross(corners[2] - corners[1], corners[0] - corners[1])
            n = n / np.linalg.norm(n)
            v = np.cross(n, u)
            return O, u, v, n

        # 1) We assume mesh_corners & urdf_corners are already exactly [BL,BR,TL,TR]
        src_pts = np.array(self.mesh_corners)   # (4×3) array of OBJ’s front‐face corners
        tgt_pts = np.array(self.urdf_corners)   # (4×3) array of URDF’s front‐face corners

        # 2) Visualize the raw corner‐pairs in 3D, if desired
        if visualize_corners:
            scene = trimesh.Scene()

            # a) Draw the URDF box in translucent blue
            urdf_copy = self.urdf.copy()
            urdf_copy.visual.face_colors = [0, 0, 255, 50]
            scene.add_geometry(urdf_copy)

            # b) Draw the OBJ mesh in solid gray
            mesh_copy = self.mesh.copy()
            mesh_copy.visual.face_colors = [200, 200, 200, 255]
            scene.add_geometry(mesh_copy)

            # c) Red spheres at URDF corners
            for c in tgt_pts:
                sph = trimesh.creation.icosphere(radius=0.01)
                sph.apply_translation(c)
                sph.visual.vertex_colors = [255, 0, 0, 255]
                scene.add_geometry(sph)

            # d) Green spheres at OBJ corners
            for c in src_pts:
                sph = trimesh.creation.icosphere(radius=0.01)
                sph.apply_translation(c)
                sph.visual.vertex_colors = [0, 255, 0, 255]
                scene.add_geometry(sph)

            # e) Thick yellow tubes for each matched pair
            for i in range(4):
                p0 = src_pts[i]
                p1 = tgt_pts[i]
                vec = p1 - p0
                length = np.linalg.norm(vec)
                if length < 1e-8:
                    continue

                # Create a cylinder of height=length, radius=0.005
                cyl = trimesh.creation.cylinder(
                    radius=0.005, 
                    height=length, 
                    sections=16
                )

                # Rotate its local +Z to align with the direction (vec/length)
                direction = vec / length
                R = trimesh.geometry.align_vectors([0.0, 0.0, 1.0], direction)
                cyl.apply_transform(R)

                # Translate the cylinder so its midpoint is (p0 + p1)/2
                midpoint = (p0 + p1) * 0.5
                cyl.apply_translation(midpoint)

                # Color it bright yellow
                cyl.visual.vertex_colors = [255, 255, 0, 255]
                scene.add_geometry(cyl)

            # f) Add coordinate axes at origin for reference
            axes = trimesh.creation.axis(origin_size=0.01, axis_length=0.2)
            scene.add_geometry(axes)

            # g) Show the scene and return from visualization (pause here)
            scene.show()
            # If you want the code to pause until the window is closed, you can:
            # input("Press Enter to continue...")

        # 3) Build local frames for the four corners
        O_src, u_src, v_src, n_src = make_frame(src_pts)
        O_tgt, u_tgt, v_tgt, n_tgt = make_frame(tgt_pts)

        # 4) Visualize the local frames if requested
        if visualize_frames:
            scene = trimesh.Scene()

            # Show the URDF box lightly
            urdf_copy = self.urdf.copy()
            urdf_copy.visual.face_colors = [0, 0, 255, 30]
            scene.add_geometry(urdf_copy)

            # Show the OBJ mesh lightly
            mesh_copy = self.mesh.copy()
            mesh_copy.visual.face_colors = [200, 200, 200, 30]
            scene.add_geometry(mesh_copy)

            # (a) Draw a small 3D coordinate triad at O_src
            tri_src = trimesh.creation.axis(origin_size=0.005, axis_length=0.1)
            # But we need to rotate that triad so its x-axis= u_src, y-axis= v_src, z-axis= n_src.
            M_src = np.eye(4)
            M_src[:3,:3] = np.stack([u_src, v_src, n_src], axis=1)  # columns = [u,v,n]
            M_src[:3,3] = O_src
            tri_src.apply_transform(M_src)
            scene.add_geometry(tri_src)

            # (b) Draw a small 3D coordinate triad at O_tgt
            tri_tgt = trimesh.creation.axis(origin_size=0.005, axis_length=0.1)
            M_tgt = np.eye(4)
            M_tgt[:3,:3] = np.stack([u_tgt, v_tgt, n_tgt], axis=1)
            M_tgt[:3,3] = O_tgt
            tri_tgt.apply_transform(M_tgt)
            scene.add_geometry(tri_tgt)

            # (c) Also draw the four corner spheres again so you can see O_src/O_tgt in context
            for c in src_pts:
                s = trimesh.creation.icosphere(radius=0.008)
                s.apply_translation(c)
                s.visual.vertex_colors = [0, 255, 0, 255]
                scene.add_geometry(s)

            for c in tgt_pts:
                s = trimesh.creation.icosphere(radius=0.008)
                s.apply_translation(c)
                s.visual.vertex_colors = [255, 0, 0, 255]
                scene.add_geometry(s)

            # (d) Finally show
            scene.show()
            # input("Press Enter to continue…")

        # 5) Project the four corners into their respective 2D UV‐spaces
        def to_uv(pts, O, u, v):
            rel = pts - O
            return np.stack([rel.dot(u), rel.dot(v)], axis=1)

        uv_src = to_uv(src_pts, O_src, u_src, v_src)  # shape (4×2)
        uv_tgt = to_uv(tgt_pts, O_tgt, u_tgt, v_tgt)  # shape (4×2)

        # 6) Visualize the 2D UV correspondences if requested
        if visualize_uv:
            plt.figure(figsize=(5,5))
            plt.scatter(uv_src[:,0], uv_src[:,1], c='r', s=50, label='OBJ corners')
            plt.scatter(uv_tgt[:,0], uv_tgt[:,1], c='b', s=50, label='URDF corners')

            for i in range(4):
                x0, y0 = uv_src[i]
                x1, y1 = uv_tgt[i]
                plt.arrow(x0, y0, x1 - x0, y1 - y0,
                          head_width=0.005, head_length=0.01,
                          fc='orange', ec='orange', length_includes_head=True)

            plt.legend()
            plt.title("2D UV Corner Correspondence")
            plt.xlabel("u")
            plt.ylabel("v")
            plt.axis('equal')
            plt.grid(True)
            plt.show()
            # input("Press Enter to continue…")

        # 7) Solve the 2D‐affine A, t so that: UV_tgt = A ⋅ UV_src + t
        N = 4
        G = np.zeros((2*N, 6))
        b = np.zeros(2*N)
        for i in range(N):
            xs, ys = uv_src[i]
            xt, yt = uv_tgt[i]
            G[2*i]   = [xs, ys, 1,  0,   0,  0]
            G[2*i+1] = [0,   0,  0, xs, ys, 1]
            b[2*i]   = xt
            b[2*i+1] = yt

        params, *_ = np.linalg.lstsq(G, b, rcond=None)
        A = np.array([[params[0], params[1]],
                      [params[3], params[4]]])
        t2 = np.array([params[2], params[5]])

        # 8) Now compute the warped positions of the four source corners (just the corners)
        rel_corners = src_pts - O_src                      # (4×3)
        coords_c = np.stack([rel_corners.dot(u_src),
                              rel_corners.dot(v_src),
                              rel_corners.dot(n_src)], axis=1)  # (4×3): [u,v,w]
        uv_corners = coords_c[:, :2]                       # (4×2)
        w_depth   = coords_c[:, 2:]                         # (4×1)
        
        uv_corners_warped = uv_corners.dot(A.T) + t2       # (4×2)
        warped_corners_3d = (O_tgt
                             + uv_corners_warped[:,0:1]*u_tgt
                             + uv_corners_warped[:,1:2]*v_tgt
                             + w_depth * n_tgt)           # (4×3)

        # 9) If requested, visualize those warped corners versus the URDF corners
        if visualize_warped_corners:
            scene = trimesh.Scene()

            # (a) Show URDF lightly so we see it
            urdf_copy = self.urdf.copy()
            urdf_copy.visual.face_colors = [0, 0, 255, 30]
            scene.add_geometry(urdf_copy)

            # (b) Show OBJ mesh lightly
            mesh_copy = self.mesh.copy()
            mesh_copy.visual.face_colors = [200, 200, 200, 30]
            scene.add_geometry(mesh_copy)

            # (c) Draw red spheres at the *target* corners (URDF)
            for c in tgt_pts:
                s = trimesh.creation.icosphere(radius=0.008)
                s.apply_translation(c)
                s.visual.vertex_colors = [255, 0, 0, 255]
                scene.add_geometry(s)

            # (d) Draw bright green spheres at the *warped* corner positions
            for c in warped_corners_3d:
                s = trimesh.creation.icosphere(radius=0.008)
                s.apply_translation(c)
                s.visual.vertex_colors = [0, 255, 0, 255]
                scene.add_geometry(s)

            # (e) Draw small lines connecting each warped corner → its URDF target
            for i in range(4):
                seg = np.vstack([warped_corners_3d[i], tgt_pts[i]])
                path = trimesh.load_path(seg)
                path.colors = np.array([[255, 255, 0, 255]])
                scene.add_geometry(path)

            # (f) Show
            scene.show()
            # input("Press Enter to continue…")

        # 10) Finally, apply the full warp to every vertex of the mesh
        all_verts = self.mesh.vertices.copy()               # (M×3)
        rel_all = all_verts - O_src                         # (M×3)
        coords_all = np.stack([rel_all.dot(u_src),
                               rel_all.dot(v_src),
                               rel_all.dot(n_src)], axis=1)  # (M×3)
        uv_all = coords_all[:, :2]                          # (M×2)
        depth_all = coords_all[:, 2:]                       # (M×1)

        uv_all_warped = uv_all.dot(A.T) + t2                # (M×2)
        warped_coords = np.concatenate([uv_all_warped, depth_all], axis=1)  # (M×3)

        new_verts = (O_tgt
                     + warped_coords[:, 0:1]*u_tgt
                     + warped_coords[:, 1:2]*v_tgt
                     + warped_coords[:, 2:3]*n_tgt)       # (M×3)

        # 11) Undo any prior “center‐on‐origin” translation
        inv_T = np.linalg.inv(self.urdf_transformations)
        warped = self.mesh.copy()
        warped.vertices = new_verts
        warped.apply_transform(inv_T)
        self.warped_mesh = warped

    def replace_geometry(self):
        # Code to render the mesh in the scene
        pass

    def debug_visualize(self,
                        show_urdf: bool,
                        show_obj: bool,
                        show_warped: bool,
                        show_points: bool,
                        show_aabb: bool = False):
        """
        Pop up a trimesh.Scene showing, depending on the flags:
        - If show_urdf=True and self.urdf exists: add self.urdf (semi-transparent blue).
        - If show_obj=True  and self.mesh exists: add self.mesh (solid gray).
        - If show_warped=True and self.warped_mesh exists: add self.warped_mesh (solid green).
        - If show_points=True and self.urdf_corners/self.mesh_corners/self.warped_mesh_corners exist:
            add spheres at each of those corner points 
            (red for URDF corners, green for mesh corners, yellow for warped_mesh_corners).
        - If show_aabb=True, draw the axis-aligned bounding box around any displayed
            geometry (URDF, OBJ, and/or WARPED) by fetching `geom.bounding_box`.
        """
        scene = trimesh.Scene()

        # Keep track of which geometries to include in AABB
        aabb_targets = []

        # 1) URDF geometry
        if show_urdf:
            if self.urdf is None:
                raise RuntimeError("URDF mesh not set. Call set_urdf() first.")
            urdf_copy = self.urdf.copy()
            urdf_copy.visual.face_colors = [0, 0, 255, 50]  # translucent blue
            scene.add_geometry(urdf_copy)
            if show_aabb:
                aabb_targets.append(urdf_copy)

        # 2) OBJ geometry
        if show_obj:
            if self.mesh is None:
                raise RuntimeError("OBJ mesh not set. Call set_mesh() first.")
            mesh_copy = self.mesh.copy()
            mesh_copy.visual.face_colors = [200, 200, 200, 255]  # solid gray
            scene.add_geometry(mesh_copy)
            if show_aabb:
                aabb_targets.append(mesh_copy)

        # 3) WARPED geometry
        if show_warped:
            if getattr(self, "warped_mesh", None) is None:
                raise RuntimeError("Warped mesh not set. Run warp() first.")
            warped_copy = self.warped_mesh.copy()
            warped_copy.visual.face_colors = [0, 200, 0, 255]  # solid green
            scene.add_geometry(warped_copy)
            if show_aabb:
                aabb_targets.append(warped_copy)

        # 4) Corner points overlay
        if show_points:
            # URDF corners (red)
            if getattr(self, "urdf_corners", None) is not None:
                for corner in self.urdf_corners:
                    sphere = trimesh.creation.icosphere(radius=0.01)
                    sphere.apply_translation(corner)
                    sphere.visual.vertex_colors = [255, 0, 0, 255]
                    scene.add_geometry(sphere)
            # Mesh corners (green)
            if getattr(self, "mesh_corners", None) is not None:
                for corner in self.mesh_corners:
                    sphere = trimesh.creation.icosphere(radius=0.01)
                    sphere.apply_translation(corner)
                    sphere.visual.vertex_colors = [0, 255, 0, 255]
                    scene.add_geometry(sphere)
            # Warped mesh corners (yellow)
            if getattr(self, "warped_mesh_corners", None) is not None:
                for corner in self.warped_mesh_corners:
                    sphere = trimesh.creation.icosphere(radius=0.01)
                    sphere.apply_translation(corner)
                    sphere.visual.vertex_colors = [255, 255, 0, 255]
                    scene.add_geometry(sphere)

        # 5) Draw AABB wireframes
        if show_aabb and aabb_targets:
            for geom in aabb_targets:
                bb = geom.bounding_box
                # Make faces transparent, edges white
                bb.visual.face_colors = [255, 255, 255, 0]  # faces invisible
                bb.visual.vertex_colors = [255, 255, 255, 255]  # edges white
                scene.add_geometry(bb)

        # 6) Draw coordinate axes if any geometry is shown
        if ((show_urdf   and self.urdf is not None) or
            (show_obj    and self.mesh is not None) or
            (show_warped and getattr(self, "warped_mesh", None) is not None)):
            largest_extent = 0.0
            if show_urdf and self.urdf is not None:
                largest_extent = max(largest_extent, max(self.urdf.extents))
            if show_obj and self.mesh is not None:
                largest_extent = max(largest_extent, max(self.mesh.extents))
            if show_warped and getattr(self, "warped_mesh", None) is not None:
                largest_extent = max(largest_extent, max(self.warped_mesh.extents))
            if largest_extent > 0:
                axes = trimesh.creation.axis(
                    origin_size=0.01,
                    axis_length=largest_extent * 1.2
                )
                scene.add_geometry(axes)

        scene.show()
    
    # Get Methods
    def get_urdf(self):
        return self.urdf
    def get_mesh(self):
        return self.mesh
    def get_urdf_corners(self):
        return self.urdf_corners
    def get_mesh_corners(self): 
        return self.mesh_corners
    def get_warped_mesh(self):  
        return self.warped_mesh
    def get_warped_mesh_corners(self):
        return self.warped_mesh_corners
    def get_urdf_transformations(self):
        return self.urdf_transformations

