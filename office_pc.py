import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os

print("Load a ply point cloud, print and render")
pcd = o3d.io.read_point_cloud("data/ulmatec.ply")
print(pcd)
print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])

# Downsampling
# print("Downsample using voxel of 0.05")
# downpcd = pcd.voxel_down_sample(voxel_size=0.1)
# print(downpcd)
# print(np.asarray(downpcd.points))
# o3d.visualization.draw_geometries([downpcd])

# Compute Normals
print("Recompute the normal of downpcd")
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([downpcd],point_show_normal=True) 


# Surface Reconstruction
# Alpha Shapes
alpha = 0.5
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

#Poisson Surface Reconstruction
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth = 9)
    
print(mesh)
o3d.visualization.draw_geometries([mesh], 
zoom = 0.664,
front = [-0.4761, -0.4698, -0.7434],
lookat = [1.89, 3.25, 0.92],    
up = [0.23, -0.88, 0.41])

pcd.orient_normals_consistent_tangent_plane(100)
o3d.visualization.draw_geometries([pcd], point_show_normal=True)