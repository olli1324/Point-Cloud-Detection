import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import imageio.v3 as iio

# Read depth image as grayscale to ensure single channel:
depth_image = iio.imread('../images/grayscale.png', mode='I')

# Print properties:
print(f"Image resolution: {depth_image.shape}")
print(f"Data type: {depth_image.dtype}")
print(f"Min value: {np.min(depth_image)}")
print(f"Max value: {np.max(depth_image)}")

# Optionally, you can visualize the depth image using matplotlib
plt.imshow(depth_image, cmap='gray')
plt.title('Depth Image')
plt.colorbar()
#plt.show()



# Depth camera parameters:
FX_DEPTH = 5.8262448167737955e+02
FY_DEPTH = 5.8269103270988637e+02
CX_DEPTH = 3.1304475870804731e+02
CY_DEPTH = 2.3844389626620386e+02  


# Transform from 2D image coord to 3D camera coord.
pcd = []
height, width = depth_image.shape
for i in range(height):
    for j in range(width):
        z = depth_image[i][j]
        x = (j - CX_DEPTH) * z / FX_DEPTH
        y = (i - CY_DEPTH) * z / FY_DEPTH
        pcd.append([x, y, z])

# Visualize the data with Open3D    
pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
pcd_o3d.points = o3d.utility.Vector3dVector(pcd)  # set pcd_np as the point cloud points
# Visualize:
#o3d.visualization.draw_geometries([pcd_o3d])



# Read the rgb image:
rgb_image = iio.imread('images/rgb.png')
# Display depth and grayscale image:
fig, axs = plt.subplots(1, 2)
axs[0].imshow(depth_image, cmap="gray")
axs[0].set_title('Depth image')
axs[1].imshow(rgb_image)
axs[1].set_title('RGB image')
#plt.show()

# Rotation matrix:
R = -np.array([[9.9997798940829263e-01, 5.0518419386157446e-03, 4.3011152014118693e-03],
                   [-5.0359919480810989e-03, 9.9998051861143999e-01, -3.6879781309514218e-03],
                   [- 4.3196624923060242e-03, 3.6662365748484798e-03, 9.9998394948385538e-01]])
# Translation vector:
T = np.array([2.5031875059141302e-02, -2.9342312935846411e-04, 6.6238747008330102e-04])


# Convert point from depth sensor 3D coord to rgb camera coord. 
[x_RGB, y_RGB, z_RGB] = np.linalg.inv(R).dot([x, y, z]) - np.linalg.inv(R).dot(T)

# Use intrinsic param. of the RGB cam to map it to color img coord.
# RGB camera intrinsic Parameters:
FX_RGB = 5.1885790117450188e+02
FY_RGB = 5.1946961112127485e+02
CX_RGB = 3.2558244941119034e+0
CY_RGB = 2.5373616633400465e+02
# Indicies of corresponding pixel:
j_rgb = int((x_RGB * FX_RGB) / z_RGB + CX_RGB + width / 2)
i_rgb = int((y_RGB * FY_RGB) / z_RGB + CY_RGB)



# Put everything together and display it
colors = []
pcd = []
for i in range(height):
    for j in range(width):
        # Convert pixel from depth coord to depth sensor 3D coord.
        z = depth_image[i][j]
        x = (j - CX_DEPTH) * z / FX_DEPTH
        y = (i - CY_DEPTH) * z / FY_DEPTH

        # Convert point from depth sensor 3D coord. to rgb cam coord.
        [x_RGB, y_RGB, z_RGB] = np.linalg.inv(R).dot([x, y, z]) - np.linalg.inv(R).dot(T)

        # Convert from rgb cam coord to rgb img coord.
        j_rgb = int((x_RGB * FX_RGB) / z_RGB + CX_RGB + width / 2)
        i_rgb = int((y_RGB * FY_RGB) / z_RGB + CY_RGB)

        # Add point to point cloud:
        pcd.append([x, y, z])

        # Add the color of the pixel if it exists:
        if 0 <= j_rgb < width and 0 <= i_rgb < height:
            colors.append(rgb_image[i_rgb][j_rgb] / 255)
        else:
            colors.append([0., 0., 0.])
            
# Convert to Open3D.PointCLoud:
pcd_o3d = o3d.geometry.PointCloud()  # create a point cloud object
pcd_o3d.points = o3d.utility.Vector3dVector(pcd)
pcd_o3d.colors = o3d.utility.Vector3dVector(colors)
# Visualize:
o3d.visualization.draw_geometries([pcd_o3d])

#i_rgb and j_rgb goes out of bounds. ERROR



#OPTIMIZATION IF WANTED:
""""
    # get depth resolution:
    height, width = depth_im.shape
    length = height * width
    # compute indices:
    jj = np.tile(range(width), height)
    ii = np.repeat(range(height), width)
    # reshape depth image
    z = depth_im.reshape(length)
    # compute pcd:
    pcd = np.dstack([(ii - CX_DEPTH) * z / FX_DEPTH, (jj - CY_DEPTH) * z / FY_DEPTH, z]).reshape((length, 3))


    # compute indices:
    jj = np.tile(range(width), height)
    ii = np.repeat(range(height), width)
    # Compute constants:
    xx = (jj - CX_DEPTH) / FX_DEPTH
    yy = (ii - CY_DEPTH) / FY_DEPTH
    # transform depth image to vector of z:
    length = height * width
    z = depth_image.reshape(height * width)
    # compute point cloud
    pcd = np.dstack((xx * z, yy * z, z)).reshape((length, 3))
"""

