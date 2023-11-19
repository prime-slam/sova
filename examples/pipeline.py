"""
This script contains example of running pipeline.
Options could be changed in the appropriate place of this script, and that's only thing that user may change (and also type of backend).

Flow of pipeline:
1. Inserts initial point cloud into Grid and subdivides it
2. Inserts remaining point clouds into Grid
3. Runs Segmenters criteria into Grid
4. Runs Filter functions to delete unnecessary voxels/point clouds
5. Runs chosen backend and produces BackendOutput result with all necessary information
6. Saves visualization to specified directory

Arguments of CLI:
data_directory: str
    Path to dataset directory which must has structure like this:
        dataset_name
            /clouds
                0.pcd
                ...
            /poses
                0.txt
                ...
first_point_cloud_number: int
    Number of first point cloud to optimise
last_point_cloud_number: int
    Number of last point cloud to optimise
step: int
    Step between optimising patches
visualizations_directory: str
    Represents name of directory where segmented point clouds would be stored
diff: bool
    Represents parameter for visualization before/after state

How can I run it?
```
python3 examples/pipeline.py \
    --data_directory evaluation/hilti \
    --first_point_cloud_number 0 \
    --last_point_cloud_number 27 \
    --step 4 \
    --visualizations_directory visualizations \
    --diff True
```
"""
import open3d as o3d
from octreelib.grid import GridConfig, VisualizationConfig

import argparse
import copy
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from slam.backend import EigenFactorBackend
from slam.pipeline import SequentialPipeline, SequentialPipelineRuntimeParameters
from slam.segmenter import RansacSegmenter
from slam.subdivider import SizeSubdivider
from slam.utils import HiltiReader, KittiReader, NuscenesReader, Reader, OptimisedPoseReadWriter

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="Pipeline")
    parser.add_argument("--data_directory", type=str, required=True)
    parser.add_argument("--first_point_cloud_number", type=int, required=True)
    parser.add_argument("--last_point_cloud_number", type=int, required=True)
    parser.add_argument("--step", type=int, required=True)
    parser.add_argument("--visualizations_directory", type=str, required=True)
    parser.add_argument("--diff", type=bool)
    args = parser.parse_args()

    reader = Reader
    if "hilti" in args.data_directory.lower():
        reader = HiltiReader()
    elif "kitti" in args.data_directory.lower():
        reader = KittiReader()
    elif "nuscenes" in args.data_directory.lower():
        reader = NuscenesReader()
    else:
        raise ValueError("Unrecognisable type of dataset")
    posesWriter = OptimisedPoseReadWriter()

    # Pipeline configuration
    # TODO(user): You can manipulate configuration specification below as you want
    iterations_count = 2

    subdividers = [
        SizeSubdivider(
            size=4,
        ),
    ]

    segmenters = [
        RansacSegmenter(
            threshold=0.01,
            initial_points=6,
            iterations=5000,
        ),
    ]

    filters = []

    grid_configuration = GridConfig(
        voxel_edge_length=8,
    )

    optimised_poses_dir = "./optimised"
    # End of pipeline specification section
    # Do not touch code below, just run it :)

    for ind in range(
        args.first_point_cloud_number, args.last_point_cloud_number, args.step
    ):
        start = ind
        end = min(args.last_point_cloud_number, ind + args.step)

        print(f"Processing {start} to {end-1}...")

        point_clouds = []
        initial_poses = []
        for s in range(start, end):
            point_cloud_path = os.path.join(
                args.data_directory, "clouds", str(s) + ".pcd"
            )
            pose_path = os.path.join(args.data_directory, "poses", str(s) + ".txt")

            point_cloud = reader.read_point_cloud(filename=point_cloud_path)
            initial_pose = reader.read_pose(filename=pose_path)

            point_clouds.append(point_cloud)
            initial_poses.append(initial_pose)

        poses = copy.deepcopy(initial_poses)
        for iteration_ind in range(iterations_count):
            # TODO(user): You can also change Backend type
            backend = EigenFactorBackend(
                poses_number=(end - start),
                iterations_number=5000,
            )

            pipeline = SequentialPipeline(
                point_clouds=point_clouds,
                poses=poses,
                subdividers=subdividers,
                segmenters=segmenters,
                filters=filters,
                backend=backend,
            )

            output = pipeline.run(
                SequentialPipelineRuntimeParameters(
                    grid_configuration=grid_configuration,
                    visualization_config=VisualizationConfig(
                        filepath=f"{args.visualizations_directory}/{start}-{end-1}_{iteration_ind}.html"
                    ),
                    initial_point_cloud_number=(end - start) // 2,
                )
            )
            print(f"Iteration: {iteration_ind}:\n{output}")

            for pose_ind in range(len(initial_poses)):
                poses[pose_ind] = output.poses[pose_ind] @ poses[pose_ind]

            if args.diff:
                random.seed(42)
                initial_point_cloud = o3d.geometry.PointCloud(
                    o3d.utility.Vector3dVector()
                )
                optimised_point_cloud = o3d.geometry.PointCloud(
                    o3d.utility.Vector3dVector()
                )
                for point_cloud, initial_pose, optimised_pose in zip(
                    point_clouds, initial_poses, poses
                ):
                    color = [random.random(), random.random(), random.random()]
                    before = copy.deepcopy(point_cloud).transform(initial_pose)
                    before.paint_uniform_color(color)
                    initial_point_cloud += before

                    after = copy.deepcopy(point_cloud).transform(optimised_pose)
                    after.paint_uniform_color(color)
                    optimised_point_cloud += after

                print("Initial point clouds is going to be printed")
                o3d.visualization.draw(initial_point_cloud)
                print("Optimised point clouds is going to be printed")
                o3d.visualization.draw(optimised_point_cloud)

        for optimised_pose_number in range(start, end):
            posesWriter.write(
                os.path.join(optimised_poses_dir, f"{optimised_pose_number}.txt"),
                poses[optimised_pose_number - start],
            )
