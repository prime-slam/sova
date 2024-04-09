"""
Using this file you can create visualization of processed
clouds with CUDA RANSAC.
"""

import sys
from typing import List

import numpy as np
import open3d as o3d
from octreelib.grid import VisualizationConfig
from octreelib.grid import GridVisualizationType

sys.path.append("..")
from slam.utils import KittiReader
from util import create_configuration, PipelineConfiguration, read_patch

N_POSES = 3  # cannot be larger than 100

DATASET_PATH = "../evaluation/kitti"
DATASET_READER = KittiReader


def run_pipeline(point_clouds: List[o3d.geometry.PointCloud],
                 pipeline: PipelineConfiguration,
                 cuda: bool = False):
    """
    Runs pipeline with time benchmarking
    """
    # Setup grid, insert points and subdivide
    middle_pose_number = len(point_clouds) // 2
    pipeline.grid.insert_points(
        middle_pose_number,
        np.asarray(point_clouds[middle_pose_number].points),
    )
    pipeline.grid.subdivide(pipeline.subdividers)
    for pose_number, point_cloud in enumerate(point_clouds):
        if pose_number == middle_pose_number:
            continue
        pipeline.grid.insert_points(pose_number, np.asarray(point_cloud.points))

    # By this point the grid is set up and subdivided into voxels with points in them.
    # We can run RANSAC for each voxel to filter out outliers

    # Run RANSAC
    print('ransac start')
    if cuda:
        # Here is the most important part for this example
        pipeline.grid.map_leaf_points_cuda_ransac()
    else:
        for segmenter in pipeline.segmenters:
            pipeline.grid.map_leaf_points(segmenter)
    print('ransac done')

    pipeline.grid.visualize(VisualizationConfig(type=GridVisualizationType.VOXEL))
    print('visualization saved to ./visualization.html')


if __name__ == "__main__":
    point_clouds = read_patch(DATASET_READER, DATASET_PATH, start=0, end=N_POSES)

    run_pipeline(point_clouds, create_configuration(), cuda=True)
