from abc import ABC, abstractmethod
from typing import Callable

from slam.typing import ArrayNx3

__all__ = ["FilterFunc", "Filter"]


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
FilterFunc = Callable[[ArrayNx3[float]], bool]


class Filter(ABC):
    """
    Represents base (abstract) filter class, which duty is to produce FilterFunc
    """
    @abstractmethod
    def create_func(self) -> FilterFunc:
        """
        Represents method, which returns necessary FilterFunc

        Returns
        -------
        func: FilterFunc
            Function that contains filter condition
        """
        pass
