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
import numpy as np
import open3d
import open3d as o3d
from octreelib.grid import VisualizationConfig

import argparse
import copy
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from slam.pipeline import (
    SequentialPipeline,
    SequentialPipelineRuntimeParameters,
    YAMLConfigurationReader,
)
from slam.utils import (
    DatasetReader,
    HiltiReader,
    KittiReader,
    NuscenesReader,
    OptimisationsReadWriter, Undistortioner,
)


def prepare_output_directories(configuration: YAMLConfigurationReader) -> None:
    visualization_dir = configuration.visualization_dir
    if not os.path.exists(visualization_dir):
        os.makedirs(visualization_dir)

    optimisation_dir = configuration.optimisation_dir
    if not os.path.exists(optimisation_dir):
        os.makedirs(os.path.join(optimisation_dir, "clouds"))
        os.makedirs(os.path.join(optimisation_dir, "poses"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="Pipeline")
    parser.add_argument("--configuration_path", type=str, required=True)
    args = parser.parse_args()

    configuration_reader = YAMLConfigurationReader(args.configuration_path)

    dataset_reader = DatasetReader
    if "hilti" in configuration_reader.dataset_path.lower():
        dataset_reader = HiltiReader()
    elif "kitti" in configuration_reader.dataset_path.lower():
        dataset_reader = KittiReader()
    elif "nuscenes" in configuration_reader.dataset_path.lower():
        dataset_reader = NuscenesReader()
    else:
        raise ValueError("Unrecognisable type of dataset")
    optimisationsWriter = OptimisationsReadWriter()

    prepare_output_directories(configuration_reader)

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

        initial_point_clouds = []
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

            initial_point_clouds.append(point_cloud)
            initial_poses.append(initial_pose)

        poses = copy.deepcopy(initial_poses)
        point_clouds = copy.deepcopy(initial_point_clouds)

        # Beginning of pipeline iterations
        for pipeline_iteration in range(configuration_reader.pipeline_iterations):
            # Beginning of patch iterations
            for patch_iteration in range(configuration_reader.patches_iterations):
                # Undistortion section
                if configuration_reader.undistortion_segments is not None:
                    pc1 = open3d.geometry.PointCloud(open3d.utility.Vector3dVector())
                    pc2 = open3d.geometry.PointCloud(open3d.utility.Vector3dVector())

                    undistortioner = Undistortioner(configuration_reader.undistortion_segments)
                    for undistortion_ind in range(end - start - 1):
                        point_clouds[undistortion_ind] = undistortioner(
                            point_clouds[undistortion_ind],
                            poses[undistortion_ind],
                            point_clouds[undistortion_ind + 1],
                            poses[undistortion_ind + 1],
                        )
                        poses[undistortion_ind] = np.eye(4)

                        temp1 = point_clouds[undistortion_ind].transform(poses[undistortion_ind])
                        temp1.paint_uniform_color([random.random(), random.random(), random.random()])
                        pc1 += temp1

                    for j in range(end - start):
                        temp2 = initial_point_clouds[j]
                        temp2.transform(initial_poses[j]).translate([0, 8, 0])
                        temp2.paint_uniform_color([0, 0, 0])
                        pc2 += temp2

                    open3d.visualization.draw(pc1 + pc2)

                pipeline = SequentialPipeline(
                    point_clouds=point_clouds,
                    poses=poses,
                    subdividers=configuration_reader.subdividers,
                    segmenters=configuration_reader.segmenters,
                    filters=configuration_reader.filters,
                    backend=configuration_reader.backend(start, end),
                )

                output = pipeline.run(
                    SequentialPipelineRuntimeParameters(
                        grid_configuration=configuration_reader.grid_configuration,
                        visualization_config=VisualizationConfig(
                            filepath=f"{configuration_reader.visualization_dir}/{start}-{end - 1}_{patch_iteration}.html"
                        ),
                        initial_point_cloud_number=(end - start) // 2,
                    )
                )
                print(f"Iteration: {patch_iteration}:\n{output}")

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
                        initial_point_clouds, initial_poses, poses
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
            optimisationsWriter.write_pose(
                os.path.join(
                    configuration_reader.optimisation_dir,
                    "poses",
                    f"{optimised_pose_number}.txt",
                ),
                poses[optimised_pose_number - start],
            )
            optimisationsWriter.write_point_cloud(
                os.path.join(
                    configuration_reader.optimisation_dir,
                    "clouds",
                    f"{optimised_pose_number}.pcd",
                ),
                point_clouds[optimised_pose_number - start],
            )
