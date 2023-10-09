import open3d as o3d

from slam.pipeline import Subdivider
from slam.typing import ArrayNx3

__all__ = ["SizeSubdivider"]


class SizeSubdivider(Subdivider):
    """
    Represents "size of voxel" condition: if it is more than N meters,
    it should be split.

    Parameters
    ----------
    size: float
        Minimum size of voxel
    scale: float
        Scale information about data
    """

    def __init__(self, size: float, scale: float) -> None:
        self.size: float = size
        self.scale: float = scale

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
            Returns True if size of point cloud less than predefined value, otherwise returns False
        """

        point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        min_bound, max_bound = point_cloud.get_min_bound(), point_cloud.get_max_bound()

        return (
            abs(min_bound[0] - max_bound[0]) * self.scale <= self.size
            and abs(min_bound[1] - max_bound[1]) * self.scale <= self.size
            and abs(min_bound[2] - max_bound[2]) * self.scale <= self.size
        )
