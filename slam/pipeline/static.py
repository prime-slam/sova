import numpy as np
from octreelib.grid import Grid, GridConfig
from octreelib.octree import MultiPoseOctree, MultiPoseOctreeConfig

import math

from slam.backend import BackendOutput
from slam.pipeline.pipeline import Pipeline

__all__ = ["StaticPipeline"]


class StaticPipeline(Pipeline):
    """
    Represents simple static slam implementation
    """

    def run(self) -> BackendOutput:
        """
        Represents base slam algorithm:
        1. Transforms point clouds to global coordinates
        2. Inserts transformed point clouds into grid
        3. Runs subdivide functions to leave only planar features
        4. Runs filter functions to delete unnecessary voxels/point clouds
        5. Runs chosen backend and produce PipelineOutput

        Returns
        -------
        output: PipelineOutput
            Structural slam output with metrics and optimised poses
        """
        transformed_point_clouds = self._transform_point_clouds()
        grid = Grid(
            GridConfig(
                octree_type=MultiPoseOctree,
                octree_config=MultiPoseOctreeConfig(),
                grid_voxel_edge_length=self.__get_max_distance(
                    transformed_point_clouds
                ),
            )
        )

        for pose_number, point_cloud in enumerate(transformed_point_clouds):
            grid.insert_points(pose_number, point_cloud.points)

        grid.subdivide(self._subdividers)

        for segmenter in self._segmenters:
            grid.map_leaf_points(segmenter)

        grid.filter(self._filters)

        return self._backend.process(grid)

    def __get_max_distance(self, transformed_point_clouds) -> float:
        maximum = 0
        for point_cloud in transformed_point_clouds:
            maximum = max(
                maximum,
                np.linalg.norm(
                    point_cloud.get_max_bound() - point_cloud.get_min_bound()
                )
                / math.sqrt(3),
            )

        return maximum
