import sys
from typing import List
import numpy as np
import open3d as o3d

sys.path.append("..")
from slam.utils import KittiReader

from octreelib.grid import VisualizationConfig
from octreelib.grid import GridVisualizationType

from util import create_configuration, PipelineConfiguration, read_patch

print('imports done')

N_POSES = 30
CUDA_RANSAC_BATCH_SIZE = 10
dataset_path = "../evaluation/kitti"


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

    # Run RANSAC
    print('ransac start')
    if cuda:
        pipeline.grid.map_leaf_points_cuda(n_poses_per_batch=CUDA_RANSAC_BATCH_SIZE)
    else:
        for segmenter in pipeline.segmenters:
            pipeline.grid.map_leaf_points(segmenter)
    print('ransac done')

    pipeline.grid.visualize(VisualizationConfig(type=GridVisualizationType.VOXEL))
    print('visualization saved to ./visualization.html')


if __name__ == "__main__":
    point_clouds = read_patch(KittiReader, dataset_path, start=0, end=N_POSES)

    run_pipeline(point_clouds, create_configuration(), cuda=True)
