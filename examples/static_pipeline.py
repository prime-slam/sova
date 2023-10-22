import numpy as np
import open3d as o3d

import argparse
import copy
import os
import random
import sys

from slam.utils import HiltiReader

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if True:
    from slam.backend import EigenFactorBackend
    from slam.pipeline import StaticPipeline
    from slam.segmenter.ransac import RansacSegmenter
    from slam.subdivider import CountSubdivider


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
        point_clouds = []
        poses = []
        for s in range(
            args.first_point_cloud * ind, args.first_point_cloud * ind + args.step
        ):
            point_cloud_path = os.path.join(point_clouds_directory, str(s) + ".pcd")
            pose_path = os.path.join(poses_directory, str(s) + ".txt")

            point_cloud = HiltiReader.read_point_cloud(filename=point_cloud_path)
            pose = HiltiReader.read_pose(filename=pose_path)

            point_clouds.append(point_cloud)
            poses.append(pose)

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
