import numpy as np
import open3d as o3d

def generate_drawer_point_cloud():
    width, height, depth = 1.0, 0.3, 0.5
    thickness = 0.02
    
    inner_width = width - 2 * thickness
    inner_height = height - thickness
    inner_depth = depth - thickness
    
    inner_offset_x = thickness
    inner_offset_y = thickness
    inner_offset_z = 0
    
    num_points_per_edge = 100

    def line_points(p1, p2, num_points=num_points_per_edge):
        return np.linspace(p1, p2, num_points)

    corners = np.array([
        [0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0],
        [0, 0, depth], [width, 0, depth], [width, height, depth], [0, height, depth]
    ])

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    edge_points = np.vstack([line_points(corners[start], corners[end]) for start, end in edges])
    
    num_points_face = 200
    
    x_front = np.random.uniform(0, width, num_points_face)
    y_front = np.random.uniform(0, height, num_points_face)
    z_front = np.full(num_points_face, depth)
    front_points = np.vstack([x_front, y_front, z_front]).T

    x_back = np.random.uniform(0, width, num_points_face)
    y_back = np.random.uniform(0, height, num_points_face)
    z_back = np.zeros(num_points_face)
    back_points = np.vstack([x_back, y_back, z_back]).T

    x_left = np.zeros(num_points_face)
    y_left = np.random.uniform(0, height, num_points_face)
    z_left = np.random.uniform(0, depth, num_points_face)
    left_points = np.vstack([x_left, y_left, z_left]).T

    x_right = np.full(num_points_face, width)
    y_right = np.random.uniform(0, height, num_points_face)
    z_right = np.random.uniform(0, depth, num_points_face)
    right_points = np.vstack([x_right, y_right, z_right]).T

    x_bottom = np.random.uniform(0, width, num_points_face)
    y_bottom = np.zeros(num_points_face)
    z_bottom = np.random.uniform(0, depth, num_points_face)
    bottom_points = np.vstack([x_bottom, y_bottom, z_bottom]).T

    handle_center = np.array([-0.02, height / 2, depth / 2])
    num_circles = 10
    num_points_per_circle = 30
    handle_radius = 0.01
    handle_depth = 0.1

    handle_points = []
    
    for depth_offset in np.linspace(-handle_depth/2, handle_depth/2, num_circles):
        theta = np.linspace(0, 2 * np.pi, num_points_per_circle)
        for t in theta:
            x = handle_center[0] + handle_radius * np.cos(t)
            y = handle_center[1] + handle_radius * np.sin(t)
            z = handle_center[2] + depth_offset
            handle_points.append([x, y, z])
            
    handle_points = np.array(handle_points)

    num_points_inner = 150
    
    x_inner_front = np.random.uniform(inner_offset_x, inner_offset_x + inner_width, num_points_inner)
    y_inner_front = np.random.uniform(inner_offset_y, inner_offset_y + inner_height, num_points_inner)
    z_inner_front = np.full(num_points_inner, depth - thickness)
    inner_front_points = np.vstack([x_inner_front, y_inner_front, z_inner_front]).T

    x_inner_back = np.random.uniform(inner_offset_x, inner_offset_x + inner_width, num_points_inner)
    y_inner_back = np.random.uniform(inner_offset_y, inner_offset_y + inner_height, num_points_inner)
    z_inner_back = np.full(num_points_inner, thickness)
    inner_back_points = np.vstack([x_inner_back, y_inner_back, z_inner_back]).T

    x_inner_left = np.full(num_points_inner, inner_offset_x)
    y_inner_left = np.random.uniform(inner_offset_y, inner_offset_y + inner_height, num_points_inner)
    z_inner_left = np.random.uniform(thickness, depth - thickness, num_points_inner)
    inner_left_points = np.vstack([x_inner_left, y_inner_left, z_inner_left]).T

    x_inner_right = np.full(num_points_inner, inner_offset_x + inner_width)
    y_inner_right = np.random.uniform(inner_offset_y, inner_offset_y + inner_height, num_points_inner)
    z_inner_right = np.random.uniform(thickness, depth - thickness, num_points_inner)
    inner_right_points = np.vstack([x_inner_right, y_inner_right, z_inner_right]).T

    x_inner_bottom = np.random.uniform(inner_offset_x, inner_offset_x + inner_width, num_points_inner)
    y_inner_bottom = np.full(num_points_inner, inner_offset_y)
    z_inner_bottom = np.random.uniform(thickness, depth - thickness, num_points_inner)
    inner_bottom_points = np.vstack([x_inner_bottom, y_inner_bottom, z_inner_bottom]).T

    num_points_rim = 50
    
    x_front_rim = np.random.uniform(0, width, num_points_rim)
    y_front_rim = np.random.uniform(height - thickness, height, num_points_rim)
    z_front_rim = np.full(num_points_rim, depth)
    front_rim_points = np.vstack([x_front_rim, y_front_rim, z_front_rim]).T

    x_back_rim = np.random.uniform(0, width, num_points_rim)
    y_back_rim = np.random.uniform(height - thickness, height, num_points_rim)
    z_back_rim = np.zeros(num_points_rim)
    back_rim_points = np.vstack([x_back_rim, y_back_rim, z_back_rim]).T

    x_left_rim = np.zeros(num_points_rim)
    y_left_rim = np.random.uniform(height - thickness, height, num_points_rim)
    z_left_rim = np.random.uniform(0, depth, num_points_rim)
    left_rim_points = np.vstack([x_left_rim, y_left_rim, z_left_rim]).T

    x_right_rim = np.full(num_points_rim, width)
    y_right_rim = np.random.uniform(height - thickness, height, num_points_rim)
    z_right_rim = np.random.uniform(0, depth, num_points_rim)
    right_rim_points = np.vstack([x_right_rim, y_right_rim, z_right_rim]).T

    all_points = np.vstack([
        edge_points, 
        front_points, 
        back_points,
        left_points,
        right_points,
        bottom_points,
        inner_front_points,
        inner_back_points,
        inner_left_points,
        inner_right_points,
        inner_bottom_points,
        front_rim_points,
        back_rim_points,
        left_rim_points,
        right_rim_points,
        handle_points
    ])

    noise = np.random.normal(scale=0.002, size=all_points.shape)
    all_points += noise

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)

    return pcd

drawer_pcd = generate_drawer_point_cloud()

o3d.io.write_point_cloud("synthetic_drawer.pcd", drawer_pcd)