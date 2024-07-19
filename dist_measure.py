from open3d import *
import open3d as o3d
import numpy as np

def create_grid_from_point_cloud(pc, margin=0.1):
    # Get the bounding box of the point cloud
    bbox = pc.get_axis_aligned_bounding_box()
    min_bound = bbox.min_bound - margin
    max_bound = bbox.max_bound + margin

    # Create grid lines
    grid_lines = []
    step = 0.1  # Adjust the step size for the grid

    for i in np.arange(min_bound[0], max_bound[0] + step, step):
        grid_lines.append([[i, min_bound[1], 0], [i, max_bound[1], 0]])  # Vertical lines
    for i in np.arange(min_bound[1], max_bound[1] + step, step):
        grid_lines.append([[min_bound[0], i, 0], [max_bound[0], i, 0]])  # Horizontal lines

    # Create LineSet for the grid
    lines = o3d.geometry.LineSet()
    lines.points = o3d.utility.Vector3dVector(np.array([point for line in grid_lines for point in line]))
    lines.lines = o3d.utility.Vector2iVector(np.array([[i, i + 1] for i in range(0, len(lines.points), 2)]))
    lines.paint_uniform_color([0.0, 0.0, 1.0])  # Color for grid lines

    return lines

if __name__ == '__main__':
    # Read point cloud
    pc = o3d.io.read_point_cloud("data/Road.ply")
    # Create 3D coordinate system
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    print(f"Number of points before voxel: {len(pc.points)}")
    # Downsampling
    pc = pc.voxel_down_sample(voxel_size=0.05)
    print(f"Number of points after voxel downsample: {len(pc.points)}")

    print(f"Number of points before removal: {len(pc.points)}")
    # Radius outlier removal
    pc_rad, ind_rad = pc.remove_radius_outlier(nb_points=16, radius=0.1)
    outlier_rad_pc = pc.select_by_index(ind_rad, invert=True)
    outlier_rad_pc.paint_uniform_color([1.0, 0.0, 1.0])
    print(f"Number of points after radius outlier removal: {len(pc_rad.points)}")

    # Create grid from point cloud
    grid = create_grid_from_point_cloud(pc_rad)

    # Sensor in origin
    sensor_pos = np.array([0.0, 0.0, 0.0])

    # Distance measuring
    distances = np.linalg.norm(np.asarray(pc.points) - sensor_pos, axis=1)
    min_distance = np.min(distances)
    nearest_point_index = np.argmin(distances)
    nearest_coord = np.asarray(pc_rad.points)[nearest_point_index]

    print(f"Minimum distance from sensor to nearest point: {min_distance} meters")
    print(f"Nearest point has coordinates: {nearest_coord}")

    # Visualize point cloud, grid, and coordinate frame
    o3d.visualization.draw_geometries([pc_rad, grid, mesh], zoom=1, front=[0.1, 0.1, 0.1], lookat=[0, 0, 0], up=[0, 0, 1])
