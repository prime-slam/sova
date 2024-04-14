from abc import ABC, abstractmethod

from sova.typing import ArrayNx3

__all__ = ["Filter"]


class Filter(ABC):
    """
    Represents abstract filter, which filters given octree/grid
    """

    @abstractmethod
    def __call__(self, points: ArrayNx3[float]) -> bool:
        """
        Represents filter function signature to call it from Octree implementation

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        is_good: bool
            Condition, which describes: if the point cloud (or voxel) satisfies the certain condition
        """
        pass
