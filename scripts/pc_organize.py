import imageio.v3 as iio
import numpy as np
import matplotlib.pyplot as plt
import ground_detection as g
import open3d as o3d

# Camera parameters:
FX_DEPTH = 5.8262448167737955e+02
FY_DEPTH = 5.8269103270988637e+02
CX_DEPTH = 3.1304475870804731e+02
CY_DEPTH = 2.3844389626620386e+02

# Read depth image:
depth_image = iio.imread('DepthCap.png')
# Compute the grayscale image:
depth_grayscale = np.array(256 * depth_image / 0x0fff, dtype=np.uint8)
# Convert a grayscale image to a 3-channel image:
depth_grayscale = np.stack((depth_grayscale,) * 3, axis=-1)

# # Visualize grayscale image
# plt.subplot(1, 2, 2)
# plt.title("Grayscale Depth Image")
# plt.imshow(depth_grayscale)
# plt.show()

# get depth image resolution:
height, width = depth_image.shape
# compute indices and reshape it to have the same shape as the depth image:
jj = np.tile(range(width), height).reshape((height, width))
ii = np.repeat(range(height), width).reshape((height, width))
# Compute constants:
xx = (jj - CX_DEPTH) / FX_DEPTH
yy = (ii - CY_DEPTH) / FY_DEPTH
# compute organised point cloud:
organized_pcd = np.dstack((xx * depth_image, yy * depth_image, depth_image))

# Print some statistics for debugging
print("Organized Point Cloud Shape:", organized_pcd.shape)
print("Max value along y-axis (y_max):", max(organized_pcd.reshape((height * width, 3)), key=lambda x: x[1])[1])

# Ground_detection:
THRESHOLD = 0.075 * 1000  # Define a threshold
y_max = max(organized_pcd.reshape((height * width, 3)), key=lambda x: x[1])[
    1]  # Get the max value along the y-axis

# Set the ground pixels to green:
for i in range(height):
    for j in range(width):
        if organized_pcd[i][j][1] >= y_max - THRESHOLD:
            depth_grayscale[i][j] = [0, 255, 0]  # Update the depth image


#UNDER PRODUCTION

# color x_min
colors = [g.RED, g.GREEN, g.BLUE, g.MAGENTA, g.YELLOW, g.CYAN]

org_pcd_colors = np.asarray(organized_pcd.colors)
n_points = org_pcd_colors.shape[0]
for i in range(n_points):
    if organized_pcd.points[i][1] <= g.x_min - THRESHOLD:
        org_pcd_colors[i] = g.GRAY

organized_pcd.colors = o3d.utility.Vector3dVector(org_pcd_colors)
o3d.visualization.draw_geometries(organized_pcd)


# Display depth_grayscale:
plt.imshow(depth_grayscale)
plt.show()