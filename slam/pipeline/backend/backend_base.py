from octreelib.grid import GridBase

from abc import ABC, abstractmethod

from slam.pipeline.result.pipeline_result import PipelineResult

__all__ = ["Backend"]


class Backend(ABC):
    """
    Represents base (abstract) class for backend implementations
    """

    @abstractmethod
    def process(self, grid: GridBase) -> PipelineResult:
        """
        Represents base method of backend, which should produce Pipeline result by
        given octree

        Parameters
        ----------
        grid: GridBase
            Represents grid with all inserted point clouds

        Returns
        -------
        result: PipelineResult
            Structural output of pipeline backend
        """
        pass
