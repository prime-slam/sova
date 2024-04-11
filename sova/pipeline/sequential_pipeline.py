import numpy as np
from octreelib.grid import Grid

from dataclasses import dataclass

from sova.backend import BackendOutput
from sova.pipeline.pipeline import Pipeline, PipelineRuntimeParameters

__all__ = ["SequentialPipelineRuntimeParameters", "SequentialPipeline"]


@dataclass
class SequentialPipelineRuntimeParameters(PipelineRuntimeParameters):
    """
    Represents parameters for SequentialPipeline to run

    Parameters
    ----------
    initial_point_cloud_number: int
        Represents key point cloud number to subdivide
    """

    initial_point_cloud_number: int = 0


class SequentialPipeline(Pipeline):
    """
    Represents implementation of Pipeline abstract class
    """

    def run(self, parameters: SequentialPipelineRuntimeParameters) -> BackendOutput:
        """
        Represents basic SLAM algorithm:
        1. Transforms point clouds to global coordinates
        2. Inserts transformed point clouds into Grid
        3. Runs subdivide functions to leave only planar features
        4. Runs filter functions to delete unnecessary voxels/point clouds
        5. Runs chosen backend and produce BackendOutput result with all necessary information

        Returns
        -------
        output: BackendOutput
            Structural SLAM result, which contains optimized poses and related metrics
        """
        transformed_point_clouds = self._transform_point_clouds()

        grid = Grid(parameters.grid_configuration)

        grid.insert_points(
            parameters.initial_point_cloud_number,
            np.asarray(
                transformed_point_clouds[parameters.initial_point_cloud_number].points
            ),
        )
        grid.subdivide(self._subdividers)

        for pose_number, point_cloud in enumerate(transformed_point_clouds):
            if pose_number == parameters.initial_point_cloud_number:
                continue
            grid.insert_points(pose_number, np.asarray(point_cloud.points))

        for segmenter in self._segmenters:
            grid.map_leaf_points(segmenter)

        grid.filter(self._filters)

        backend_output = self._backend.process(grid)

        if self._debug:
            parameters.visualization_config.unused_voxels = (
                backend_output.unused_features
            )
            grid.visualize(parameters.visualization_config)

        return backend_output
