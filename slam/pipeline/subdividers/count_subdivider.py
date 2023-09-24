from slam.pipeline.subdividers.subdivider_base import Subdivider, SubdividerFunc
from slam.typing import ArrayNx3

__all__ = ["CountSubdivider"]


class CountSubdivider(Subdivider):
    """
    Represents basic "number of points" condition: if there are more than N points in voxel,
    it will be split.

    Parameters
    ----------
    count: int
        Bottom bound of
    """
    def __init__(self, count: int) -> None:
        self.count = count

    def create_func(self) -> SubdividerFunc:
        """
        Represents implementation of Subdivider abstract class

        Returns
        -------
        func: SubdividerFunc
            Function that contains "number of points" subdivider condition
        """
        def f(points: ArrayNx3[float]) -> bool:
            return len(points) <= self.count

        return f
