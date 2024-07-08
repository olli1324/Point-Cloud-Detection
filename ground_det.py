import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import open3d as o3d
import imageio.v3 as iio

# Read point cloud:
pcd = o3d.io.read_point_cloud("data/Road.ply")
# Create a 3D coordinate system:
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
# geometries to draw:
geometries = [pcd, origin]

# Get max and min points of each axis x, y and z:
x_max = max(pcd.points, key=lambda x: x[0])
y_max = max(pcd.points, key=lambda x: x[1])
z_max = max(pcd.points, key=lambda x: x[2])
x_min = min(pcd.points, key=lambda x: x[0])
y_min = min(pcd.points, key=lambda x: x[1])
z_min = min(pcd.points, key=lambda x: x[2])


# Colors:
RED = [1., 0., 0.]
GREEN = [0., 1., 0.]
BLUE = [0., 0., 1.]
YELLOW = [1., 1., 0.]
MAGENTA = [1., 0., 1.]
CYAN = [0., 1., 1.]
GRAY = [0.5, 0.5, 0.5]

positions = [x_max, y_max, z_max, x_min, y_min, z_min]
colors = [RED, GREEN, BLUE, MAGENTA, YELLOW, CYAN]
for i in range(len(positions)):
   # Create a sphere mesh:
   sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
   # move to the point position:
   sphere.translate(np.asarray(positions[i]))
   # add color:
   sphere.paint_uniform_color(np.asarray(colors[i]))
   # compute normals for vertices or faces:
   sphere.compute_vertex_normals()
   # add to geometry list to display later:
   geometries.append(sphere)

# Define a threshold:
THRESHOLD = 0.075

# Get the max value along the y-axis:
y_max = max(pcd.points, key=lambda x: x[1])[1]

# Get the original points color to be updated:
# pcd_colors = np.asarray(pcd.colors)
pcd_colors = np.full((len(pcd.points), 3), GRAY)

# Number of points:
n_points = pcd_colors.shape[0]

# update color:
for i in range(n_points):
    # if the current point is aground point:
    if pcd.points[i][1] >= y_max - THRESHOLD:
        pcd_colors[i] = GREEN  # color it green

pcd.colors = o3d.utility.Vector3dVector(pcd_colors)

# Display:
o3d.visualization.draw_geometries([pcd, origin])