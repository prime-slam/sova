from slam.octree import Octree
from slam.pipeline.backend.backend_base import Backend
from slam.pipeline.result.pipeline_result import PipelineResult

__all__ = ["MrobBackend"]


class MrobBackend(Backend):
    """
    Represents mrob backend implementation
    """
    def __init__(self) -> None:
        pass

    def process(self, octree: Octree) -> PipelineResult:
        """
        Represents implementation of Backend abstract class, which
        takes remaining points from poses and optimised using them

        Parameters
        ----------
        octree: Octree
            Octree which contains all inserted point clouds

        Returns
        -------
        result: PipelineResult
            Structural output of pipeline backend
        """
        return PipelineResult()
