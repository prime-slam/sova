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


clouds = "hilti/basement1/clouds"
initial_poses = "hilti/basement1/poses"
optimised_poses = "output/hilti/basement1/optimisation/poses"

blue = Color("blue").rgb
green = Color("green").rgb

point_cloud = open3d.geometry.PointCloud(open3d.utility.Vector3dVector())

for ind in range(400, 410):
    initial_cloud = open3d.io.read_point_cloud(os.path.join(clouds, f"{ind}.pcd"))
    initial_pose = read_pose(os.path.join(initial_poses, f"{ind}.txt"))
    initial_cloud.transform(initial_pose)
    initial_cloud.paint_uniform_color(blue)
    point_cloud += initial_cloud

    optimised_cloud = open3d.io.read_point_cloud(os.path.join(clouds, f"{ind}.pcd"))
    optimised_pose = read_pose(os.path.join(optimised_poses, f"{ind}.txt"))
    optimised_cloud.transform(optimised_pose).translate([0, 10, 0])
    optimised_cloud.paint_uniform_color(green)
    point_cloud += optimised_cloud

open3d.visualization.draw(point_cloud)
