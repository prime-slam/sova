from sova.filter.filter import Filter
from sova.typing import ArrayNx3

__all__ = ["EmptyVoxel"]


class EmptyVoxel(Filter):
    """
    Represents "empty voxel" filter: if voxel contains no points, it should be deleted
    """

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
            Returns False if there is no points, otherwise returns True.
        """
        return len(points) != 0
