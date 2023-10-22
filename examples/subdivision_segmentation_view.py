import open3d as o3d
from octreelib.grid import StaticGrid, StaticGridConfig
from octreelib.octree import Octree, OctreeConfig

import argparse
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if True:
    from slam.segmenter import RansacSegmenter
    from slam.subdivider import CountSubdivider
    from slam.utils import HiltiReader, Visualiser

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="StaticPipeline")
    parser.add_argument("--data_directory", type=str, required=True)
    parser.add_argument("--filename", type=str, required=True)
    parser.add_argument("--frame", type=int, required=True)
    parser.add_argument("--diff", type=bool, required=False, default=False)
    args = parser.parse_args()

    point_cloud_path = os.path.join(
        args.data_directory, "clouds", str(args.frame) + ".pcd"
    )
    pose_path = os.path.join(args.data_directory, "poses", str(args.frame) + ".txt")

    point_cloud = HiltiReader.read_point_cloud(filename=point_cloud_path)
    pose = HiltiReader.read_pose(filename=pose_path)
    point_cloud = point_cloud.transform(pose)

    subdividers = [
        CountSubdivider(
            count=800,
        ),
    ]

    segmenter = RansacSegmenter(
        threshold=0.1,
        initial_points=3,
        iterations=5000,
    )

    grid = StaticGrid(StaticGridConfig(Octree, OctreeConfig()))
    grid.insert_points(
        pose_number=0,
        points=point_cloud.points,
    )
    grid.subdivide(subdividers)
    grid.map_leaf_points(segmenter)
    Visualiser.draw(grid=grid, filename=args.filename)

    if args.diff:
        random.seed(42)
        segmented_points_cloud = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(grid.get_points(0))
        )
        segmented_points_cloud.paint_uniform_color(
            [random.random(), random.random(), random.random()]
        )
        o3d.visualization.draw(point_cloud + segmented_points_cloud)
