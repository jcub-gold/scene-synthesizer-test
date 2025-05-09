import xml.etree.ElementTree as ET
import trimesh
import numpy as np

def extract_urdf_front_face_corners(boxes):
    # Concatenate all boxes
    drawer = trimesh.util.concatenate(boxes)
    vertices = drawer.vertices
    
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
    
    return np.array(corners)

def get_boxes(urdf_path):
    # Parse URDF
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    link = root.find("./link[@name='lower_cabinet_drawer_0_0']")

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
    return boxes

def load_and_visualize(urdf_path):
    boxes = get_boxes(urdf_path)
    # Get corners and visualize
    corners = extract_urdf_front_face_corners(boxes)
    print("URDF front face corners:\n", corners)

    # Visualize to verify
    scene = trimesh.Scene()
    drawer = trimesh.util.concatenate(boxes)
    scene.add_geometry(drawer)
    for corner in corners:
        sphere = trimesh.creation.icosphere(radius=0.01)
        sphere.apply_translation(corner)
        sphere.visual.vertex_colors = [0, 255, 0, 255]  # Green markers
        scene.add_geometry(sphere)
    scene.show()

if __name__ == "__main__":
    urdf_path = "exports/drawer.urdf"
    load_and_visualize(urdf_path)
    