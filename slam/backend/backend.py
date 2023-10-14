from octreelib.grid import GridBase

from abc import ABC, abstractmethod

from slam.pipeline.pipeline import PipelineOutput

__all__ = ["Backend"]


class Backend(ABC):
    """
    Represents base (abstract) class for backend implementations
    """

    @abstractmethod
    def process(self, grid: GridBase) -> PipelineOutput:
        """
        Represents base method of backend, which should produce Pipeline result by
        given octree

        Parameters
        ----------
        grid: GridBase
            Represents grid with all inserted point clouds

        Returns
        -------
        output: PipelineOutput
            Result of backend optimisations
        """
        pass
