from open3d import *
import math
import numpy as np
import open3d as o3d
import itertools


#Finally, the input point cloud is cropped using the created bounding box object
if __name__ == '__main__':
    # Read point cloud:
    pcd = o3d.io.read_point_cloud("data/Road.ply")

    # Create bounding box:
    bounds = [[-math.inf, math.inf], [-math.inf, math.inf], [0.8, 2]]  # set the bounds
    bounding_box_points = list(itertools.product(*bounds))  # create limit points
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object

    # Crop the point cloud using the bounding box:
    pcd_cropped = pcd.crop(bounding_box)

    # Display the cropped point cloud:
    o3d.visualization.draw_geometries([pcd_cropped])