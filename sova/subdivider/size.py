import numpy as np
import open3d as o3d

import math

from sova.subdivider.subdivider import Subdivider
from sova.typing import ArrayNx3

__all__ = ["SizeSubdivider"]


class SizeSubdivider(Subdivider):
    """
    Represents "size of voxel" condition: if it is more than N meters,
    it should be split.

    Parameters
    ----------
    size: float
        Minimum size of voxel
    """

    def __init__(self, size: float) -> None:
        self.__size: float = size

    def __call__(self, points: ArrayNx3[float]) -> bool:
        """
        Represent size-based subdivider mechanism which returns True/False statement based on voxel size

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        should_be_split: bool
            Returns True if size of point cloud more than predefined value, otherwise returns False
        """

        point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        min_bound, max_bound = point_cloud.get_min_bound(), point_cloud.get_max_bound()
        side_length = np.linalg.norm(max_bound - min_bound) / math.sqrt(3)

        return side_length > self.__size
