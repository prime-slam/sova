from abc import ABC, abstractmethod

from sova.typing import ArrayNx3

__all__ = ["Subdivider"]


class Subdivider(ABC):
    """
    Represents abstract subdivider, which returns statement about voxel: subdivide or not.
    """

    @abstractmethod
    def __call__(self, points: ArrayNx3[float]) -> bool:
        """
        Represent abstract dunder method which returns statement about voxel

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        should_be_split: bool
            Condition, which describes: if the point cloud (or voxel) should be divided
        """
        pass
