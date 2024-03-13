import sys
from typing import List, Dict
import numpy as np
import open3d as o3d

sys.path.append("..")
from slam.utils import KittiReader

from octreelib.grid import Grid, GridConfig, VisualizationConfig
from octreelib.grid import GridVisualizationType
from octreelib.grid.grid import CUDA_RANSAC_BATCH_SIZE_POSES

from util import create_configuration, evaluate, PipelineDurations, PipelineConfiguration, read_patch

print('imports done')

N_POSES = 30
CUDA_RANSAC_BATCH_SIZE_POSES = 10
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
    # ransac = CudaRansac(n_blocks=pipeline.grid.sum_of_leaves(), n_threads_per_block=1024)
    if cuda:
        pipeline.grid.map_leaf_points_cuda()
        # pipeline.grid.map_leaf_points_cuda(ransac)
    else:
        for segmenter in pipeline.segmenters:
            pipeline.grid.map_leaf_points(segmenter)
    print('ransac done')

    pipeline.grid.visualize(VisualizationConfig(type=GridVisualizationType.VOXEL))
    print('visualization saved to ./visualization.html')


def measure(cuda, start, end) -> Dict[str, float]:
    point_clouds = read_patch(KittiReader, dataset_path, start, end)

    run_pipeline(point_clouds, create_configuration(), cuda)


measure(True, 0, 5)
