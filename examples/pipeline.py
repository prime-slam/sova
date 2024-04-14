"""
This script contains example of running pipeline.
Options could be changed in the appropriate place of YAML file.

Flow of pipeline:
1. Inserts initial point cloud into Grid and subdivides it
2. Inserts remaining point clouds into Grid
3. Runs Segmenters criteria into Grid
4. Runs Filter functions to delete unnecessary voxels/point clouds
5. Runs chosen backend and produces BackendOutput result with all necessary information
6. Saves visualization to specified directory

Arguments of CLI:
configuration_path: str
    Represents path to YAML configuration of Pipeline

How can I run it?
```
python3 examples/pipeline.py --configuration_path examples/configurations/hilti.yaml
```
"""
from typing import Tuple

import open3d as o3d
from octreelib.grid import VisualizationConfig

import argparse
import copy
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sova.pipeline import (
    SequentialPipeline,
    SequentialPipelineRuntimeParameters,
    YAMLConfigurationReader,
)
from sova.utils import (
    DatasetReader,
    HiltiReader,
    KittiReader,
    NuscenesReader,
    OptimisedPoseReadWriter,
)


def prepare_output_directories(
    configuration: YAMLConfigurationReader,
) -> Tuple[str, str]:
    if not os.path.exists(configuration.output_directory):
        os.makedirs(configuration.output_directory)

    poses_dir = os.path.join(configuration.output_directory, "poses")
    if not os.path.exists(poses_dir):
        os.makedirs(poses_dir)

    visualization_dir = os.path.join(configuration.output_directory, "visualization")
    if configuration.debug:
        if not os.path.join(visualization_dir):
            os.makedirs(visualization_dir)

    return poses_dir, visualization_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="Pipeline")
    parser.add_argument("--configuration_path", type=str, required=True)
    args = parser.parse_args()

    configuration_reader = YAMLConfigurationReader(args.configuration_path)

    dataset_reader = DatasetReader
    if "hilti" in configuration_reader.dataset_type.lower():
        dataset_reader = HiltiReader()
    elif "kitti" in configuration_reader.dataset_type.lower():
        dataset_reader = KittiReader()
    elif "nuscenes" in configuration_reader.dataset_type.lower():
        dataset_reader = NuscenesReader()
    else:
        raise ValueError("Unrecognisable type of dataset")
    posesWriter = OptimisedPoseReadWriter()

    poses_dir, visualization_dir = prepare_output_directories(configuration_reader)

    for ind in range(
        configuration_reader.patches_start,
        configuration_reader.patches_end,
        configuration_reader.patches_step,
    ):
        start = ind
        end = min(
            configuration_reader.patches_end, ind + configuration_reader.patches_step
        )

        print(f"Processing {start} to {end-1}...")

        point_clouds = []
        initial_poses = []
        for s in range(start, end):
            point_cloud_path = os.path.join(
                configuration_reader.dataset_path, "clouds", str(s) + ".pcd"
            )
            pose_path = os.path.join(
                configuration_reader.dataset_path, "poses", str(s) + ".txt"
            )

            point_cloud = dataset_reader.read_point_cloud(filename=point_cloud_path)
            initial_pose = dataset_reader.read_pose(filename=pose_path)

            point_clouds.append(point_cloud)
            initial_poses.append(initial_pose)

        poses = copy.deepcopy(initial_poses)
        for iteration_ind in range(configuration_reader.patches_iterations):
            pipeline = SequentialPipeline(
                point_clouds=point_clouds,
                poses=poses,
                subdividers=configuration_reader.subdividers,
                segmenters=configuration_reader.segmenters,
                filters=configuration_reader.filters,
                backend=configuration_reader.backend(start, end),
                debug=configuration_reader.debug,
            )

            output = pipeline.run(
                SequentialPipelineRuntimeParameters(
                    grid_configuration=configuration_reader.grid_configuration,
                    visualization_config=VisualizationConfig(
                        filepath=f"{visualization_dir}/{start}-{end - 1}_{iteration_ind}.html"
                    ),
                    initial_point_cloud_number=(end - start) // 2,
                )
            )
            print(f"Iteration: {iteration_ind}:\n{output}")

            for pose_ind in range(len(initial_poses)):
                poses[pose_ind] = output.poses[pose_ind] @ poses[pose_ind]

            if configuration_reader.debug:
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
                os.path.join(poses_dir, f"{optimised_pose_number}.txt"),
                poses[optimised_pose_number - start],
            )
