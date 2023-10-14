from octreelib.grid import StaticGrid, StaticGridConfig
from octreelib.octree import Octree, OctreeConfig

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
        grid = StaticGrid(StaticGridConfig(Octree, OctreeConfig()))
        grid.octrees.clear()  # TODO: wtf??? Why do I need to do this every time?

        for pose_number, point_cloud in enumerate(transformed_point_clouds):
            grid.insert_points(pose_number, point_cloud.points)

        grid.subdivide(self._subdividers)

        grid.map_leaf_points(self._segmenter)

        grid.filter(self._filters)

        return self._backend.process(grid)
