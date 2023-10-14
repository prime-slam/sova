from octreelib.grid import StaticGrid, StaticGridConfig
from octreelib.octree import Octree, OctreeConfig

from slam.pipeline.pipeline_base import Pipeline
from slam.pipeline.result.pipeline_result import PipelineResult

__all__ = ["StaticPipeline"]


class StaticPipeline(Pipeline):
    """
    Represents simple static pipeline implementation
    """

    def run(self) -> PipelineResult:
        """
        Represents base pipeline algorithm:
        1. Transforms point clouds to global coordinates
        2. Inserts transformed point clouds into octree
        3. Runs subdivide function to leave only planar features
        4. Runs filter function to delete unnecessary voxels/point clouds
        5. Runs chose backend and produce PipelineResult

        Returns
        -------
        result: PipelineResult
            Structural pipeline result
        """
        transformed_point_clouds = self.__transform_point_clouds()
        grid = StaticGrid(StaticGridConfig(Octree, OctreeConfig()))

        for pose_number, point_cloud in enumerate(transformed_point_clouds):
            grid.insert_points(pose_number, point_cloud.points)

        grid.subdivide(self.subdividers)

        grid.filter(self.filters)

        return self.backend.process(grid)
