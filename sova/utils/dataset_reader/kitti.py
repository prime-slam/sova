import numpy as np
import open3d as o3d

import os.path

from sova.typing import ArrayNx4x4
from sova.utils.dataset_reader.reader import DatasetReader

__all__ = ["KittiReader"]


class KittiReader(DatasetReader):
    """
    Represents KITTI dataset dataset_reader
    Source: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
    """

    @staticmethod
    def read_pose(filename: str) -> ArrayNx4x4[float]:
        """
        Reads KITTI pose file
        """
        calibration_matrix = np.eye(4)
        calibration_file_path = os.path.join(os.path.dirname(filename), "calib.txt")
        with open(calibration_file_path) as file:
            line = file.readlines()[4][4:]
        calibration_matrix[:3, :4] = np.array(
            list(map(float, line.rstrip().split(" ")))
        ).reshape(3, 4)

        pose_matrix = np.eye(4)
        with open(filename) as file:
            lines = file.readlines()
            pose_matrix[:3, :4] = np.array(
                list(map(float, lines[0].rstrip().split(" ")))
            ).reshape(3, 4)

        return pose_matrix @ calibration_matrix

    @staticmethod
    def read_point_cloud(filename: str) -> o3d.geometry.PointCloud:
        """
        Reads point cloud from KITTI dataset
        """
        return o3d.io.read_point_cloud(filename)
