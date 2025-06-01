import trimesh
import numpy as np
import xml.etree.ElementTree as ET

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

    def warp(self):
        assert (self.urdf is not None and
                self.mesh is not None and
                self.urdf_corners is not None and
                self.mesh_corners is not None), "run previous stages to get meshes and corners"
        
        def match_corners(obj_corners, urdf_corners):
            """Match corners by sorting both sets in the same order."""
            obj_corners = np.array(obj_corners)
            urdf_corners = np.array(urdf_corners)
            obj_sorted = obj_corners[np.lexsort((obj_corners[:,2], obj_corners[:,0]))]
            urdf_sorted = urdf_corners[np.lexsort((urdf_corners[:,2], urdf_corners[:,0]))]
            return obj_sorted, urdf_sorted


        def make_frame(corners):
            """Compute a local frame (origin, u, v, normal) from four 3D corner points."""
            O = corners.mean(axis=0)
            u = corners[1] - corners[0]
            u /= np.linalg.norm(u)
            n = np.cross(corners[2] - corners[1], corners[0] - corners[1])
            n /= np.linalg.norm(n)
            v = np.cross(n, u)
            return O, u, v, n


        src_pts, tgt_pts = match_corners(self.mesh_corners, self.urdf_corners)

        # build local frames
        O_src, u_src, v_src, n_src = make_frame(src_pts)
        O_tgt, u_tgt, v_tgt, n_tgt = make_frame(tgt_pts)

        # project into uv plane
        def to_uv(c, O, u, v): return np.stack([(c - O).dot(u), (c - O).dot(v)], axis=1)
        uv_src = to_uv(src_pts, O_src, u_src, v_src)
        uv_tgt = to_uv(tgt_pts, O_tgt, u_tgt, v_tgt)

        # build affine solve
        N = len(uv_src)
        G = np.zeros((2*N,6))
        b = np.zeros(2*N)
        for i, (x,y) in enumerate(uv_src):
            G[2*i]   = [x, y, 1, 0, 0, 0]
            G[2*i+1] = [0, 0, 0, x, y, 1]
            b[2*i]   = uv_tgt[i,0]
            b[2*i+1] = uv_tgt[i,1]
        params, *_ = np.linalg.lstsq(G, b, rcond=None)
        A = np.array([[params[0], params[1]], [params[3], params[4]]])
        t2 = np.array([params[2], params[5]])

        # warp each vertex
        verts = self.mesh.vertices.copy()
        rel = verts - O_src
        coords = np.stack([rel.dot(u_src), rel.dot(v_src), rel.dot(n_src)], axis=1)
        uv = coords[:,:2]
        z  = coords[:,2:]
        uv_w = uv.dot(A.T) + t2
        warped_coords = np.concatenate([uv_w, z], axis=1)
        new_verts = (O_tgt + 
                    warped_coords[:,0:1]*u_tgt +
                    warped_coords[:,1:2]*v_tgt +
                    warped_coords[:,2:3]*n_tgt)

        inv_T = np.linalg.inv(self.urdf_transformations)

        warped = self.mesh.copy()
        warped.vertices = new_verts          # these are in the centered‐origin frame
        warped.apply_transform(inv_T)        # now apply the inverse‐translate back
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

