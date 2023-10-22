import numpy as np
import open3d as o3d

from slam.typing import ArrayNx4x4
from slam.utils.reader.reader import Reader

__all__ = ["KittiReader"]


class KittiReader(Reader):
    """
    Represents KITTI dataset reader
    Source: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
    """

    @staticmethod
    def read_pose(filename: str) -> ArrayNx4x4[float]:
        """
        Reads KITTI pose file
        """
        pose = np.eye(4)
        with open(filename) as file:
            lines = file.readlines()
            pose = np.array(list(map(float, lines[0].rstrip().split(" ")))).reshape(
                3, 4
            )

        return pose

    @staticmethod
    def read_point_cloud(filename: str) -> o3d.geometry.PointCloud:
        """
        Reads point cloud from KITTI dataset
        """
        return o3d.io.read_point_cloud(filename)
