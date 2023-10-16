import numpy as np
import open3d as o3d
from octreelib.grid import StaticGrid, StaticGridConfig
from octreelib.octree import Octree, OctreeConfig

import argparse
import os
import sys
from typing import List

from slam.segmenter import RansacSegmenter
from slam.utils import Visualiser

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if True:
    from slam.subdivider import CountSubdivider
    from slam.typing import ArrayNx4x4


def read_point_clouds(
    point_clouds_directory: str, first_number: int, last_number: int
) -> List[o3d.geometry.PointCloud]:
    point_clouds = []

    for i in range(first_number, last_number):
        point_cloud_path = os.path.join(point_clouds_directory, str(i) + ".pcd")
        point_cloud = o3d.io.read_point_cloud(point_cloud_path)

        point_clouds.append(point_cloud)

    return point_clouds


def read_poses(
    poses_directory: str, first_number: int, last_number: int
) -> ArrayNx4x4[float]:
    poses = []

    for i in range(first_number, last_number):
        pose_path = os.path.join(poses_directory, str(i) + ".txt")
        pose = np.eye(4)
        with open(pose_path) as file:
            lines = file.readlines()
            for ind, line in enumerate(lines):
                pose[ind] = list(map(float, line.replace("\n", "").split(" ")))

        poses.append(pose)

    return poses


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="StaticPipeline")
    parser.add_argument("--data_directory", type=str, required=True)
    args = parser.parse_args()

    point_clouds_directory = os.path.join(args.data_directory, "clouds")
    poses_directory = os.path.join(args.data_directory, "poses")

    pose = read_poses(
        poses_directory=poses_directory,
        first_number=0,
        last_number=1,
    )[0]

    point_cloud = read_point_clouds(
        point_clouds_directory=point_clouds_directory,
        first_number=0,
        last_number=1,
    )[0].transform(pose)

    subdividers = [
        CountSubdivider(
            count=800,
        ),
    ]

    segmenter = RansacSegmenter(
        threshold=0.01,
        initial_points=3,
        iterations=5000,
    )

    grid = StaticGrid(StaticGridConfig(Octree, OctreeConfig()))
    grid.insert_points(0, point_cloud.points)
    grid.subdivide(subdividers)
    grid.map_leaf_points(segmenter)
    Visualiser.draw(grid=grid)
