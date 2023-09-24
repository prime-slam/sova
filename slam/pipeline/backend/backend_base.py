from abc import ABC, abstractmethod

from slam.octree import Octree
from slam.pipeline.result.pipeline_result import PipelineResult

__all__ = ["Backend"]


class Backend(ABC):
    """
    Represents base (abstract) class for backend implementations
    """
    @abstractmethod
    def process(self, octree: Octree) -> PipelineResult:
        """
        Represents base method of backend, which should produce Pipeline result by
        given octree

        Parameters
        ----------
        octree: Octree
            Octree which contains all inserted point clouds

        Returns
        -------
        result: PipelineResult
            Structural output of pipeline backend
        """
        pass
