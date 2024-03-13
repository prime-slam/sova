import sys
import time
import os
from typing import List, Dict
from dataclasses import dataclass
import numpy as np
import open3d as o3d

sys.path.append("..")
from slam.backend import EigenFactorBackend, Backend
from slam.segmenter import Segmenter, RansacSegmenter
from slam.subdivider import Subdivider, SizeSubdivider
from slam.utils import DatasetReader, KittiReader

from octreelib.grid import Grid, GridConfig, VisualizationConfig
from octreelib.grid import GridVisualizationType, CUDA_RANSAC_BATCH_SIZE_POSES
from octreelib.ransac.cuda_ransac import CudaRansac



def evaluate(timestamps: List[float]):
    """
    Evaluates given timestamps List by calculating min, max, mean and std values
    """

    def print_metrics(ts):
        print(f"\tmin = {round(np.min(ts), 3)}s")
        print(f"\tmax = {round(np.max(ts), 3)}s")
        print(f"\tmean = {round(np.mean(ts), 3)}s")
        print(f"\tstd = {round(np.std(ts), 3)}s")

    timestamps = np.array(timestamps)
    print_metrics(timestamps)


def read_patch(reader: DatasetReader, path: str, start: int, end: int) -> List[o3d.geometry.PointCloud]:
    """
    Reads patch of point clouds
    """
    point_clouds = []

    hilti_clouds = os.path.join(path, "clouds")
    hilti_poses = os.path.join(path, "poses")

    for ind in range(start, end):
        point_cloud_path = os.path.join(hilti_clouds, str(ind) + ".pcd")
        pose_path = os.path.join(hilti_poses, str(ind) + ".txt")

        point_cloud = reader.read_point_cloud(filename=point_cloud_path)
        pose = reader.read_pose(filename=pose_path)

        point_clouds.append(point_cloud.transform(pose))

    return point_clouds



@dataclass
class PipelineConfiguration:
    """
    Represents pipeline configuration
    """
    subdividers: List[Subdivider]
    segmenters: List[Segmenter]
    backend: Backend
    grid: Grid


@dataclass
class PipelineDurations:
    """
    Represents duration of each pipeline stage
    """
    initialization: float
    subdividers: float
    distribution: float
    segmenters: float
    backend: float


def create_configuration() -> PipelineConfiguration:
    subdividers = [
        SizeSubdivider(
            size=2,
        ),
    ]

    segmenters = [
        RansacSegmenter(
            threshold=0.01,
            initial_points=6,
            iterations=1024,
        )
    ]

    backend = EigenFactorBackend(
        poses_number=1,
        iterations_number=5000,
    )

    grid = Grid(
        GridConfig(
            voxel_edge_length=4,
        )
    )

    return PipelineConfiguration(
        subdividers=subdividers,
        segmenters=segmenters,
        backend=backend,
        grid=grid,
    )
