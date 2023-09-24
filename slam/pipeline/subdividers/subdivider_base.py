from abc import abstractmethod, ABC
from typing import Callable

from slam.typing import ArrayNx3

__all__ = ["SubdividerFunc", "Subdivider"]


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
SubdividerFunc = Callable[[ArrayNx3[float]], bool]


class Subdivider(ABC):
    """
    Represents base (abstract) subdivider class, which duty is to produce SubdividerFunc
    """
    @abstractmethod
    def create_func(self) -> SubdividerFunc:
        """
        Represents method, which returns necessary SubdividerFunc

        Returns
        -------
        func: SubdividerFunc
            Function that contains subdivider condition
        """
        pass
