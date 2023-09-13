import open3d as o3d

from typing import Optional
import random

from slam.octree.base_octree import Octree
from slam.octree.depth_octree.depth_octree_config import DepthOctreeConfig
from slam.octree.depth_octree.depth_octree_node import DepthOctreeNode
from slam.segmenters.base_segmenter import Segmenter
from slam.segmenters.full_segmenter.full_segmenter import FullSegmenter

__all__ = ["DepthOctree", "DepthOctreeNode"]


class DepthOctree(Octree):
    """
    Represents Octree implementation, which uses depth parameter as final segmentation factor
    """

    def __init__(
        self,
        point_cloud: o3d.geometry.PointCloud,
        config: DepthOctreeConfig,
        segmenter: Segmenter = FullSegmenter,
    ) -> None:
        if len(point_cloud.points) == 0:
            raise ValueError("Size of point cloud must be positive")

        self.__root: Optional[DepthOctreeNode] = None
        self.__config: DepthOctreeConfig = config
        self.__segmenter: Segmenter = segmenter

        self.__build(point_cloud)

    def __build(self, point_cloud: o3d.geometry.PointCloud) -> None:
        """
        Represents method for building depth octree

        Parameters
        ----------
        point_cloud: o3d.geometry.PointCloud
            Point cloud are used to build octree
        """
        self.__root = DepthOctreeNode(
            point_cloud.get_min_bound(), point_cloud.get_max_bound()
        )

        for point in point_cloud.points:
            self.__root.insert(point, self.__config.depth)

        self.__root.__points = None

    def segment(self) -> None:
        """
        Represents implementation of abstract method to segment planes from built octree
        """
        self.__root.segment(self.__segmenter, self.__config.depth)

    def visualize(self) -> None:
        """
        Represents abstract method to visualize segmented point cloud.
        """
        random.seed(0)
        point_cloud = self.__root.get_colorized(self.__config.depth)
        o3d.visualization.draw(point_cloud)
