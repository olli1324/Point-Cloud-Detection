import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import imageio.v3 as iio

#Point in coordinate system
number_points = 5
pcd = np.random.rand(number_points, 3)  # uniform distribution over [0, 1)
print(pcd)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.scatter3D(pcd[:, 0], pcd[:, 1], pcd[:, 2])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Random Point Cloud")
# plt.show()




# Bunny
bunny = o3d.data.BunnyMesh()
mesh = o3d.io.read_triangle_mesh(bunny.path)
mesh.compute_vertex_normals() # compute normals for vertices or faces
#o3d.visualization.draw_geometries([mesh])



# Bunny points:
pcd = mesh.sample_points_uniformly(number_of_points=1000)
#o3d.visualization.draw_geometries([pcd])
# Save into ply file:
#o3d.io.write_point_cloud("output/bunny_pcd.ply", pcd)




# Create numpy pointcloud:
number_points = 2000
pcd_np = np.random.rand(number_points, 3)
# Convert to Open3D.PointCLoud:
pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
pcd_o3d.points = o3d.utility.Vector3dVector(pcd_np)  # set pcd_np as the point cloud points
#o3d.visualization.draw_geometries([pcd_o3d])




# Read the bunny point cloud file:
pcd_o3d = o3d.io.read_point_cloud("output/bunny_pcd.ply")
# Convert the open3d object to numpy:
pcd1_np = np.asarray(pcd_o3d.points)
# Display using matplotlib:
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.scatter3D(pcd1_np[:, 0], pcd1_np[:, 2], pcd1_np[:, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Bunny Point Cloud")
plt.show()