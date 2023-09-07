from abc import abstractmethod, ABC
import open3d as o3d
from slam.octree.segmentaters.base_plane_segmentater import PlaneSegmentater


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
    def segment(self, segmentater: PlaneSegmentater) -> None:
        """
        Represents abstract method to segment planes from given on early stages point cloud

        Parameters
        ----------
        segmentater: PlaneSegmentater
            Plane segmentation mechanism
        """
        pass

    @abstractmethod
    def visualize(self) -> None:
        """
        Represents abstract method to visualize segmented point cloud.
        """
        pass
