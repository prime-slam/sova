from typing import Callable, List

import open3d as o3d

from abc import ABC, abstractmethod

__all__ = ["Octree"]

from slam.pipeline.filters.filter_base import Filter
from slam.pipeline.subdividers.subdivider_base import Subdivider
from slam.typing import ArrayNx3


class Octree(ABC):
    """
    Represents abstract class for octree struct
    """

    @abstractmethod
    def insert(self, pose: int, point_cloud: o3d.geometry.PointCloud):
        pass

    @abstractmethod
    def subdivide(self, subdividers: List[Subdivider]):
        pass

    @abstractmethod
    def filter(self, filters: List[Filter]):
        pass

    @abstractmethod
    def segment(self) -> None:
        """
        Represents abstract method to segment planes from given on early stages point cloud
        """
        pass

    @abstractmethod
    def visualize(self) -> None:
        """
        Represents abstract method to visualize segmented point cloud.
        """
        pass
