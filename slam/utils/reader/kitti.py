import os.path

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
        calibration_matrix = np.eye(4)
        calibration_file_path = os.path.join(os.path.dirname(filename), "calib.txt")
        with open(calibration_file_path) as file:
            line = file.readlines()[4][4:]
        calibration_matrix[:3, :4] = np.array(list(map(float, line.rstrip().split(" ")))).reshape(3, 4)

        pose_matrix = np.eye(4)
        with open(filename) as file:
            lines = file.readlines()
            pose_matrix[:3, :4] = np.array(list(map(float, lines[0].rstrip().split(" ")))).reshape(
                3, 4
            )

        return pose_matrix @ calibration_matrix

    @staticmethod
    def read_point_cloud(filename: str) -> o3d.geometry.PointCloud:
        """
        Reads point cloud from KITTI dataset
        """
        points = np.fromfile(filename, dtype=np.float32).reshape(-1, 4)
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points[:, :3])

        return point_cloud
