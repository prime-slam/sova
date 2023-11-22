"""
This script could be run to check point cloud segmentation by using different options and parameters.
Options could be changed in the appropriate place of this script, and that's only thing that user may change.

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
point_cloud_number: int
    Number of point cloud to segment
visualization_filepath: str
    Path where visualization must be saved
diff: bool
    Represents parameter for visualization before/after state

How can I run it?
```
python3 examples/segmentation.py \
    --data_directory evaluation/hilti \
    --point_cloud_number 0 \
    --visualization_filepath visualization.html \
    --diff True
```
"""
import numpy as np
import open3d as o3d
from octreelib.grid import Grid, GridConfig, VisualizationConfig

import argparse
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from slam.segmenter import RansacSegmenter
from slam.subdivider import SizeSubdivider
from slam.utils import HiltiReader, KittiReader, NuscenesReader, Reader

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="Segmentation")
    parser.add_argument("--data_directory", type=str, required=True)
    parser.add_argument("--point_cloud_number", type=int, required=True)
    parser.add_argument("--visualization_filepath", type=str, required=True)
    parser.add_argument("--diff", type=bool, required=False, default=False)
    args = parser.parse_args()

    reader = Reader
    if "hilti" in args.data_directory.lower():
        reader = HiltiReader()
    elif "kitti.yaml" in args.data_directory.lower():
        reader = KittiReader()
    elif "nuscenes" in args.data_directory.lower():
        reader = NuscenesReader()
    else:
        raise ValueError("Unrecognisable type of dataset")

    point_cloud_path = os.path.join(
        args.data_directory, "clouds", str(args.point_cloud_number) + ".pcd"
    )
    pose_path = os.path.join(
        args.data_directory, "poses", str(args.point_cloud_number) + ".txt"
    )

    point_cloud = reader.read_point_cloud(filename=point_cloud_path)
    pose = reader.read_pose(filename=pose_path)
    point_cloud = point_cloud.transform(pose)

    # Segmentation configuration
    # TODO(user): You can manipulate configuration specification below as you want
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

    grid = Grid(
        GridConfig(
            voxel_edge_length=8,
        )
    )
    # End of specification section
    # Do not touch code below, just run it :)

    grid.insert_points(
        pose_number=0,
        points=point_cloud.points,
    )
    grid.subdivide(subdividers)
    for segmenter in segmenters:
        grid.map_leaf_points(segmenter)

    grid.visualize(
        VisualizationConfig(
            filepath=args.visualization_filepath,
        )
    )

    if args.diff:
        random.seed(42)
        segmented_points_cloud = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(np.array(grid.get_points(0)))
        )
        segmented_points_cloud.paint_uniform_color(
            [random.random(), random.random(), random.random()]
        )
        o3d.visualization.draw(point_cloud + segmented_points_cloud)
