import numpy as np
import open3d as o3d

import argparse
import copy
import os
import random
import sys
from typing import List

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if True:
    from slam.backend import EigenFactorBackend
    from slam.pipeline import StaticPipeline
    from slam.segmenter.ransac import RansacSegmenter
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
    parser.add_argument("--first_point_cloud", type=int, required=True)
    parser.add_argument("--last_point_cloud", type=int, required=True)
    parser.add_argument("--step", type=int, required=True)
    args = parser.parse_args()

    point_clouds_directory = os.path.join(args.data_directory, "clouds")
    poses_directory = os.path.join(args.data_directory, "poses")

    subdividers = [
        CountSubdivider(
            count=800,
        ),
    ]

    filters = []

    optimised_point_clouds = []
    random.seed(42)

    for ind in range(args.first_point_cloud, args.last_point_cloud, args.step):
        print(f"Processing {ind} to {ind + args.step}...")
        point_clouds = read_point_clouds(
            point_clouds_directory=point_clouds_directory,
            first_number=ind,
            last_number=ind + args.step,
        )

        poses = read_poses(
            poses_directory=poses_directory,
            first_number=ind,
            last_number=ind + args.step,
        )

        backend = EigenFactorBackend(
            poses_number=args.step,
            iterations_number=5000,
        )

        pipeline = StaticPipeline(
            point_clouds=point_clouds,
            poses=poses,
            subdividers=subdividers,
            segmenter=RansacSegmenter(
                threshold=0.01,
                initial_points=3,
                iterations=5000,
            ),
            filters=filters,
            backend=backend,
        )

        result = pipeline.run()
        print(f"Processing {ind} to {ind + args.step} result:\n{result}")

        optimised_point_clouds_temp = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector()
        )
        for point_cloud, initial_pose, optimised_pose in zip(
            point_clouds, poses, result.poses
        ):
            point_cloud_to_insert = (
                copy.deepcopy(point_cloud)
                .transform(initial_pose)
                .transform(optimised_pose)
            )
            optimised_point_clouds_temp += point_cloud_to_insert

        optimised_point_clouds.append(optimised_point_clouds_temp)

    print(f"Processing all {args.step}'lets...")

    backend = EigenFactorBackend(
        poses_number=((args.last_point_cloud - args.first_point_cloud) // args.step),
        iterations_number=5000,
    )

    pipeline = StaticPipeline(
        point_clouds=optimised_point_clouds,
        poses=[np.eye(4) for _ in range(len(optimised_point_clouds))],
        subdividers=subdividers,
        segmenter=RansacSegmenter(
            threshold=1,
            initial_points=3,
            iterations=5000,
        ),
        filters=filters,
        backend=backend,
    )

    result = pipeline.run()
    print(f"Processing all {args.step}'lets result:\n{result}")

    random.seed(42)
    optimised_point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector())
    for point_cloud, optimised_pose in zip(optimised_point_clouds, result.poses):
        point_cloud_to_insert = copy.deepcopy(point_cloud).transform(optimised_pose)
        point_cloud_to_insert.paint_uniform_color(
            [random.random(), random.random(), random.random()]
        )
        optimised_point_cloud += point_cloud_to_insert

    o3d.visualization.draw(optimised_point_cloud)
