from slam.pipeline.subdividers.subdivider_base import Subdivider
from slam.typing import ArrayNx3

__all__ = ["CountSubdivider"]


class CountSubdivider(Subdivider):
    """
    Represents basic "number of points" condition: if there are more than N points in voxel,
    it will be split.

    Parameters
    ----------
    count: int
        Bottom bound number of points in given point cloud
    """

    def __init__(self, count: int) -> None:
        self.count = count

    def __call__(self, points: ArrayNx3[float]) -> bool:
        """
        Represents implementation of abstract call method

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        is_good: bool
            Returns True if number of points in given point cloud less than predefined value, otherwise returns False
        """
        return len(points) <= self.count
