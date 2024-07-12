import open3d as o3d
import numpy as np

# Read point cloud:
pcd = o3d.io.read_point_cloud("data/Road.ply")
# Down sampling to reduce the running time:
pcd = pcd.voxel_down_sample(voxel_size=0.02)

# Radius outlier removal:
pcd_rad, ind_rad = pcd.remove_radius_outlier(nb_points=16, radius=0.05)
outlier_rad_pcd = pcd.select_by_index(ind_rad, invert=True)
outlier_rad_pcd.paint_uniform_color([1., 0., 1.])

# Statistical outlier removal:
pcd_stat, ind_stat = pcd.remove_statistical_outlier(nb_neighbors=20,
                                                std_ratio=2.0)
outlier_stat_pcd = pcd.select_by_index(ind_stat, invert=True)
outlier_stat_pcd.paint_uniform_color([0., 0., 1.])

# Translate to visualize:
points = np.asarray(pcd_stat.points)
points += [0, 0, 0]
pcd_stat.points = o3d.utility.Vector3dVector(points)

points = np.asarray(outlier_stat_pcd.points)
points += [10, 0, 0]
outlier_stat_pcd.points = o3d.utility.Vector3dVector(points)

points = np.asarray(pcd_rad.points)
points += [20, 0, 0]
pcd_rad.points = o3d.utility.Vector3dVector(points)

points = np.asarray(outlier_rad_pcd.points)
points += [30, 0, 0]
outlier_rad_pcd.points = o3d.utility.Vector3dVector(points)

# Display:
o3d.visualization.draw_geometries([pcd_stat, outlier_stat_pcd, pcd_rad, outlier_rad_pcd])