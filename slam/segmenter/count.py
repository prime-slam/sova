from slam.segmenter import Segmenter
from slam.typing import ArrayNx3

__all__ = ["CountSegmenter"]


class CountSegmenter(Segmenter):
    """
    TODO: move it to Filter condition.

    Represents "number of points" filter: if there are less than N points in voxel,
    it should be deleted.

    Parameters
    ----------
    count: int
        Lower bound number of points in given point cloud
    """

    def __init__(self, count: int) -> None:
        self.__count = count

    def __call__(self, points: ArrayNx3[float]) -> ArrayNx3[float]:
        """
        Represent count-based filter mechanism which returns identical points or empty array.

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        segmented_points: ArrayNx3[float]
            List of 3D segmented points after processing the count-filter
        """
        if len(points) <= self.__count:
            return []

        return points
