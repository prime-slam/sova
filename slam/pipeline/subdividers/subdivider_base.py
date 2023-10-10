from abc import ABC, abstractmethod

from slam.typing import ArrayNx3

__all__ = ["Subdivider"]


class Subdivider(ABC):
    """
    Represents base (abstract) subdivider class, which duty is to return subdivide statement about points
    """

    @abstractmethod
    def __call__(self, points: ArrayNx3[float]) -> bool:
        """
        Represents subdivider function signature to call it from Octree implementation

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        is_good: bool
            Condition, which describes: if the point cloud satisfies the certain condition
        """
        pass
