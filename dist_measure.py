from open3d import *
import math
import numpy as np
import itertools
import matplotlib.pyplot as plt
import open3d as o3d
import imageio.v3 as iio

if __name__ == '__main__':
    # Read point cloud:
    pc = o3d.io.read_point_cloud("data/Road.ply")
    # Create 3D coord. system:
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    
    # Down sampling to reduce the running time: Fjerner støy
    pc = pc.voxel_down_sample(voxel_size=0.02)
    
    # Radius outlier removal: #Jevner ut
    pc_rad, ind_rad = pc.remove_radius_outlier(nb_points=16, radius=0.05)
    outlier_rad_pc = pc.select_by_index(ind_rad, invert=True)
    outlier_rad_pc.paint_uniform_color([1., 0., 1.])


    # Sensor in origin
    sensor_pos = np.array([0.0,0.0,0.0])


    # Distance measuring
    distances = np.linalg.norm(np.asarray(pc.points) - sensor_pos, axis= 1)
    min_distance = np.min(distances)

    nearest_point_index = np.argmin(distances)
    nearest_coord = np.asarray(pc.points)[nearest_point_index]


    # Create bounding box:
    bounds = [[0, 1], [-2, 2], [-math.inf, math.inf]]  # set the bounds
    bounding_box_points = list(itertools.product(*bounds))  # create limit points
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(bounding_box_points))  # create bounding box object

    # Crop the point cloud using the bounding box:
    pc_cropped = pc_rad.crop(bounding_box)



    print(f"Minimum distance from sensor to nearest point: {min_distance} meters")
    print(f"Nearest point has coordinates: {nearest_coord}")


    def custom_draw_geometry_with_rotation(pc_cropped):

        def rotate_view(vis):
            ctr = vis.get_view_control()
            #print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
            ctr.change_field_of_view(step=0) #zoom 5-90deg
            #print("Field of view (after changing) %.2f" % ctr.get_field_of_view())
            ctr.rotate(2.0, 0.0)
            ctr.set_lookat([0.00568425, 0.01403132, -0.00640959])
            ctr.set_front([0, 1, 0])
            ctr.set_up([0, 0, 1])
            ctr.set_zoom(0.9)
            return False
        o3d.visualization.draw_geometries_with_animation_callback([pc], rotate_view)


    # o3d.visualization.draw_geometries([pc_cropped])
    # o3d.visualization.draw_geometries([pc], zoom=0.1, front= [0.1, 0.1, 0.1], lookat=[1, 1, 1], up=[-0.0694, -0.9768, 0.2024])
    custom_draw_geometry_with_rotation(pc)