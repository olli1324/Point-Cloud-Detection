import open3d as o3d
import numpy as np

if __name__ == '__main__':
    # Read point cloud:
    pcd = o3d.io.read_point_cloud("data/Road.ply")

    # Random down-sampling:
    random_pcd = pcd.random_down_sample(sampling_ratio=0.005)

    # Uniform down-sampling:
    uniform_pcd = pcd.uniform_down_sample(every_k_points=200)

    # Voxel down-sampling:
    voxel_pcd = pcd.voxel_down_sample(voxel_size=0.4)

    # Translating point clouds:
    #left
    points = np.asarray(random_pcd.points)
    points += [10, 0, 0]
    random_pcd.points = o3d.utility.Vector3dVector(points)

    #middle
    points = np.asarray(uniform_pcd.points)
    points += [20, 0, 0]
    uniform_pcd.points = o3d.utility.Vector3dVector(points)

    #right
    points = np.asarray(voxel_pcd.points)
    points += [30, 0, 0]
    voxel_pcd.points = o3d.utility.Vector3dVector(points)

    # Display:
    o3d.visualization.draw_geometries([pcd, random_pcd, uniform_pcd, voxel_pcd])