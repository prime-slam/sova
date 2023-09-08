import open3d as o3d

from abc import ABC, abstractmethod

from slam.octree.segmenters.base_plane_segmenter import PlaneSegmenter

__all__ = ["Octree"]


class Octree(ABC):
    """
    Represents abstract class for octree struct
    """

    @abstractmethod
    def build(self, point_cloud: o3d.geometry.PointCloud) -> None:
        """
        Represents abstract method to build octree from given point cloud

        Parameters
        ----------
        point_cloud: o3d.geometry.PointCloud
            Point cloud, which uses to build octree.
        """
        pass

    @abstractmethod
    def segment(self, segmenter: PlaneSegmenter) -> None:
        """
        Represents abstract method to segment planes from given on early stages point cloud

        Parameters
        ----------
        segmenter: PlaneSegmenter
            Plane segmentation mechanism
        """
        pass

    @abstractmethod
    def visualize(self) -> None:
        """
        Represents abstract method to visualize segmented point cloud.
        """
        pass
