import open3d as o3d
import numpy as np

def create_grid_from_point_cloud(pc, margin=0.1, step=0.1):
    bbox = pc.get_axis_aligned_bounding_box()
    min_bound = bbox.min_bound - margin
    max_bound = bbox.max_bound + margin

    grid_lines = []
    for i in np.arange(min_bound[0], max_bound[0] + step, step):
        grid_lines.append([[i, min_bound[1], 0], [i, max_bound[1], 0]])  # X lines
    for i in np.arange(min_bound[1], max_bound[1] + step, step):
        grid_lines.append([[min_bound[0], i, 0], [max_bound[0], i, 0]])  # Y lines
    for j in np.arange(min_bound[2], max_bound[2] + step, step):
        grid_lines.append([[min_bound[0], min_bound[1], j], [max_bound[0], min_bound[1], j]])  # Z lines
        grid_lines.append([[min_bound[0], max_bound[1], j], [max_bound[0], max_bound[1], j]])  # Z lines
        grid_lines.append([[min_bound[0], min_bound[1], j], [min_bound[0], max_bound[1], j]])  # Y lines
        grid_lines.append([[max_bound[0], min_bound[1], j], [max_bound[0], max_bound[1], j]])  # Y lines

    lines = o3d.geometry.LineSet()
    lines.points = o3d.utility.Vector3dVector(np.array([point for line in grid_lines for point in line]))
    lines.lines = o3d.utility.Vector2iVector(np.array([[i, i + 1] for i in range(0, len(lines.points), 2)]))
    lines.paint_uniform_color([0.0, 0.0, 1.0])  # Color for grid lines

    return lines

def create_sensor_representation(origin):
    cube = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
    cube.translate(origin)  # Center the cube at the sensor position
    cube.paint_uniform_color([1.0, 0.0, 0.0])  # Color the cube red
    return cube

if __name__ == '__main__':
    pc = o3d.io.read_point_cloud("data/Road.ply")
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    origin = np.array([0.0, 0.0, 0.0])  # Sensor position
    pc.translate(np.array([0, 3, 0]))

    print(f"Number of points before voxel: {len(pc.points)}")
    pc = pc.voxel_down_sample(voxel_size=0.05)
    print(f"Number of points after voxel downsample: {len(pc.points)}")

    print(f"Number of points before removal: {len(pc.points)}")
    pc_rad, ind_rad = pc.remove_radius_outlier(nb_points=16, radius=0.1)
    outlier_rad_pc = pc.select_by_index(ind_rad, invert=True)
    outlier_rad_pc.paint_uniform_color([1.0, 0.0, 1.0])
    print(f"Number of points after radius outlier removal: {len(pc_rad.points)}")

    # Set the color of the nearest point to yellow
    distances = np.linalg.norm(np.asarray(pc_rad.points) - origin, axis=1)
    nearest_point_index = np.argmin(distances)

    # Initialize colors for the point cloud
    colors = np.tile([0.5, 0.5, 0.5], (len(pc_rad.points), 1))  # Default to gray
    colors[nearest_point_index] = [1.0, 0.0, 0.0]  # Set nearest point color to yellow

    pc_rad.colors = o3d.utility.Vector3dVector(colors)
    pc_rad.estimate_normals()
    

    sensor_cube = create_sensor_representation(origin)

    print(f"Minimum distance from sensor to nearest point: {distances[nearest_point_index]} meters")
    print(f"Nearest point has coordinates: {np.asarray(pc_rad.points)[nearest_point_index]}")

    o3d.visualization.draw_geometries([pc_rad, create_grid_from_point_cloud(pc_rad), mesh, sensor_cube])
