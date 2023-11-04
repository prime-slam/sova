from dataclasses import dataclass
from typing import Optional

from octreelib.grid import GridBase

from slam.backend import BackendOutput
from slam.pipeline.pipeline import Pipeline, PipelineRuntimeParameters

__all__ = ["StaticPipeline"]


@dataclass
class StaticPipelineRuntimeParameters(PipelineRuntimeParameters):
    """
    Represents list of parameters which will be used in `run` function of pipeline

    Parameters
    ----------
    key_point_cloud_number: Optional[int]
        Represents number of the point cloud for which will be called `subdivide` function
    """
    key_point_cloud_number: Optional[int] = None


class StaticPipeline(Pipeline):
    """
    Represents simple static slam implementation
    """

    def run(self, parameters: StaticPipelineRuntimeParameters, grid: GridBase) -> BackendOutput:
        """
        Represents base slam algorithm:
        1. Transforms point clouds to global coordinates
        2. Inserts transformed point clouds into grid
        3. Runs subdivide functions to leave only planar features
        4. Runs filter functions to delete unnecessary voxels/point clouds
        5. Runs chosen backend and produce PipelineOutput

        Parameters
        ----------
        parameters: StaticPipelineRuntimeParameters
            Represents parameters for `run` function of pipeline
        grid: GridBase
            Grid which will be used for optimizations

        Returns
        -------
        output: PipelineOutput
            Structural slam output with metrics and optimised poses
        """
        transformed_point_clouds = self._transform_point_clouds()

        for pose_number, point_cloud in enumerate(transformed_point_clouds):
            grid.insert_points(pose_number, point_cloud.points)

        grid.subdivide(self._subdividers)

        for segmenter in self._segmenters:
            grid.map_leaf_points(segmenter)

        grid.filter(self._filters)

        return self._backend.process(grid)
