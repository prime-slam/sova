"""
This script contains example of running pipeline.
Options could be changed in the appropriate place of this script, and that's only thing that user may change (and also type of backend).

Flow of pipeline optimising one patch:
1. Inserts initial point cloud into Grid and subdivides it
2. Inserts remaining point clouds into Grid
3. Runs Segmenters criteria into Grid
4. Runs filter functions to delete unnecessary voxels/point clouds
5. Runs chosen backend and produce BackendOutput result with all necessary information
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
    --step 27 \
    --visualizations_directory visualizations \
    --diff True
```
"""
import open3d as o3d
from octreelib.grid import VisualizationConfig

import argparse
import copy
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from slam.backend import EigenFactorBackend
from slam.pipeline import SequentialPipeline, SequentialPipelineRuntimeParameters
from slam.segmenter import CAPESegmenter, RansacSegmenter
from slam.subdivider import SizeSubdivider
from slam.utils import HiltiReader, KittiReader, Reader

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
    else:
        raise ValueError("Unrecognisable type of dataset")

    point_clouds_directory = os.path.join(args.data_directory, "clouds")
    poses_directory = os.path.join(args.data_directory, "poses")

    # Pipeline configuration
    # TODO(user): You can manipulate configuration specification below as you want
    initial_voxel_size = 8

    subdividers = [
        SizeSubdivider(
            size=2,
        ),
    ]

    segmenters = [
        RansacSegmenter(
            threshold=0.01,
            initial_points=6,
            iterations=5000,
        ),
        CAPESegmenter(
            correlation=100,
        ),
    ]

    filters = []
    # End of pipeline specification section
    # Do not touch code below, just run it :)

    for ind in range(args.first_point_cloud_number, args.last_point_cloud_number, args.step):
        print(f"Processing {ind} to {ind + args.step}...")

        point_clouds = []
        poses = []
        for s in range(ind, ind + args.step):
            point_cloud_path = os.path.join(
                args.data_directory, "clouds", str(s) + ".pcd"
            )
            pose_path = os.path.join(args.data_directory, "poses", str(s) + ".txt")

            point_cloud = reader.read_point_cloud(filename=point_cloud_path)
            pose = reader.read_pose(filename=pose_path)

            point_clouds.append(point_cloud)
            poses.append(pose)

        # TODO(user): You can also change Backend type
        backend = EigenFactorBackend(
            poses_number=args.step,
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
                initial_voxel_size=initial_voxel_size,
                visualization_config=VisualizationConfig(
                    filepath=f"{args.visualizations_directory}/{ind}-{ind + args.step}.html"
                ),
                initial_point_cloud_number=len(point_clouds) // 2,
            )
        )
        print(f"Output:\n{output}")

        if args.diff:
            random.seed(42)
            initial_point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector())
            optimised_point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector())
            for point_cloud, initial_pose, optimised_pose in zip(
                point_clouds, poses, output.poses
            ):
                color = [random.random(), random.random(), random.random()]
                point_cloud_to_insert = copy.deepcopy(point_cloud).transform(initial_pose)
                point_cloud_to_insert.paint_uniform_color(color)

                initial_point_cloud += point_cloud_to_insert
                optimised_point_cloud += point_cloud_to_insert

            print("Initial point clouds is going to printed")
            o3d.visualization.draw(initial_point_cloud)
            print("Optimised point clouds is going to printed")
            o3d.visualization.draw(optimised_point_cloud)
