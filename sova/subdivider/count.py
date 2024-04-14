from sova.subdivider.subdivider import Subdivider
from sova.typing import ArrayNx3

__all__ = ["CountSubdivider"]


class CountSubdivider(Subdivider):
    """
    Represents basic "number of points" condition: if there are more than N points in voxel,
    it should be split.

    Parameters
    ----------
    count: int
        Upper bound number of points in given point cloud
    """

    def __init__(self, count: int) -> None:
        self.count = count

    def __call__(self, points: ArrayNx3[float]) -> bool:
        """
        Represent count-based subdivider mechanism which returns True/False statement based on points count

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        should_be_split: bool
            Returns False if number of points in given point cloud more than predefined value, otherwise returns False
        """
        return len(points) >= self.count
