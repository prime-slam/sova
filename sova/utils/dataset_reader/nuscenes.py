import numpy as np
import open3d as o3d

from sova.typing import ArrayNx4x4
from sova.utils.dataset_reader.reader import DatasetReader

__all__ = ["NuscenesReader"]


class NuscenesReader(DatasetReader):
    """
    Represents Nuscenes dataset dataset_reader
    This dataset was converted to KITTI format using script:
    https://github.com/PRBonn/lidar_transfer/blob/main/auxiliary/convert/nuscenes2kitti.py
    Source: https://www.nuscenes.org/nuscenes
    """

    @staticmethod
    def read_pose(filename: str) -> ArrayNx4x4[float]:
        """
        Reads KITTI pose file
        """
        pose_matrix = np.eye(4)
        with open(filename) as file:
            lines = file.readlines()
            pose_matrix[:3, :4] = np.array(
                list(map(float, lines[0].rstrip().split(" ")))
            ).reshape(3, 4)

        return pose_matrix

    @staticmethod
    def read_point_cloud(filename: str) -> o3d.geometry.PointCloud:
        """
        Reads point cloud from KITTI dataset
        """
        return o3d.io.read_point_cloud(filename)
