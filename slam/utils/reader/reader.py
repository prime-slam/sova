import open3d as o3d

from abc import ABC, abstractmethod

from slam.typing import ArrayNx4x4

__all__ = ["Reader"]


class Reader(ABC):
    """
    Represents abstract class for reading poses and point clouds of various datasets
    """

    @staticmethod
    @abstractmethod
    def read_pose(filename: str) -> ArrayNx4x4[float]:
        """
        Represent abstract static method for reading pose file by given path
        """
        pass

    @staticmethod
    @abstractmethod
    def read_point_cloud(filename: str) -> o3d.geometry.PointCloud:
        """
        Represents abstract static method for reading point cloud file by given path
        """
        pass
