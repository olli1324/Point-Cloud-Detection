import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import imageio.v3 as iio

# Read point cloud:
pcd = o3d.io.read_point_cloud("data/depth_3.ply")
# Create a 3D coordinate system:
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
# geometries to draw:
geometries = [pcd, origin]
# Visualize:
#o3d.visualization.draw_geometries(geometries)

# Get max and min points of each axis x, y and z:
x_max = max(pcd.points, key=lambda x: x[0])
y_max = max(pcd.points, key=lambda x: x[1])
z_max = max(pcd.points, key=lambda x: x[2])
x_min = min(pcd.points, key=lambda x: x[0])
y_min = min(pcd.points, key=lambda x: x[1])
z_min = min(pcd.points, key=lambda x: x[2])

# Visualize with axis. Blue, red, green arrow represent Z-axis, X-axis, Y-axis.
"""
viewer = o3d.visualization.Visualizer()
viewer.create_window()
for geometry in geometries:
    viewer.add_geometry(geometry)
opt = viewer.get_render_option()
opt.show_coordinate_frame = True
opt.background_color = np.asarray([0.5, 0.5, 0.5])
viewer.run()
viewer.destroy_window()
"""

# Colors:
RED = [1., 0., 0.]
GREEN = [0., 1., 0.]
BLUE = [0., 0., 1.]
YELLOW = [1., 1., 0.]
MAGENTA = [1., 0., 1.]
CYAN = [0., 1., 1.]

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

# Display:
o3d.visualization.draw_geometries(geometries)