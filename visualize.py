import os.path

import numpy as np
import open3d
from colour import Color


def read_pose(path: str) -> np.ndarray:
    pose = np.eye(4)
    with open(path) as file:
        lines = file.readlines()
        for ind, line in enumerate(lines):
            pose[ind] = list(map(float, line.replace("\n", "").split(" ")))

    return pose


initial_clouds = "hilti/basement1/clouds"
fast_lio_poses = "hilti/basement1/poses"

optimised_clouds = "output/hilti/optimisation/clouds"
optimised_poses = "output/hilti/optimisation/poses"

undistorted_optimised_clouds = "output/hilti/optimisation_undistortion_400_500/clouds"
undistorted_optimised_poses = "output/hilti/optimisation_undistortion_400_500/poses"

point_cloud = open3d.geometry.PointCloud(open3d.utility.Vector3dVector())
red = Color("red").rgb
blue = Color("blue").rgb
green = Color("green").rgb

for ind in range(400, 499, 4):
    # initial_pose = np.eye(4)
    # initial_cloud = open3d.io.read_point_cloud(os.path.join(initial_clouds, f"{ind}.pcd"))
    # initial_cloud.paint_uniform_color(red)
    # point_cloud += initial_cloud
    #

    fast_lio_pose = read_pose(os.path.join(fast_lio_poses, f"{ind}.txt"))
    fast_lio_cloud = open3d.io.read_point_cloud(os.path.join(initial_clouds, f"{ind}.pcd")).transform(fast_lio_pose)
    fast_lio_cloud.paint_uniform_color(blue)
    point_cloud += fast_lio_cloud

    optimised_pose = read_pose(os.path.join(optimised_poses, f"{ind}.txt"))
    optimised_cloud = open3d.io.read_point_cloud(os.path.join(optimised_clouds, f"{ind}.pcd")).transform(optimised_pose).translate([0, 8, 0])
    optimised_cloud.paint_uniform_color(green)
    point_cloud += optimised_cloud

    undistorted_optimised_pose = read_pose(os.path.join(undistorted_optimised_poses, f"{ind}.txt"))
    undistorted_optimised_cloud = open3d.io.read_point_cloud(os.path.join(undistorted_optimised_clouds, f"{ind}.pcd")).transform(undistorted_optimised_pose).translate([0, 16, 0])
    undistorted_optimised_cloud.paint_uniform_color(red)
    point_cloud += undistorted_optimised_cloud

open3d.visualization.draw(point_cloud)
