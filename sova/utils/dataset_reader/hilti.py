import numpy as np
import open3d as o3d

from sova.typing import ArrayNx4x4
from sova.utils.dataset_reader.reader import DatasetReader

__all__ = ["HiltiReader"]


class HiltiReader(DatasetReader):
    """
    Represents Hilti dataset dataset_reader
    Source: https://hilti-challenge.com/dataset-2023.html
    """

    @staticmethod
    def read_pose(filename: str) -> ArrayNx4x4[float]:
        """
        Reads hilti pose file
        """
        pose = np.eye(4)
        with open(filename) as file:
            lines = file.readlines()
            for ind, line in enumerate(lines):
                pose[ind] = list(map(float, line.replace("\n", "").split(" ")))

        return pose

    @staticmethod
    def read_point_cloud(filename: str) -> o3d.geometry.PointCloud:
        """
        Reads point cloud from hilti dataset
        """
        return o3d.io.read_point_cloud(filename)
