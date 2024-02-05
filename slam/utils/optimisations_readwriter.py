import numpy as np
import open3d as o3d

from slam.typing import Array4x4, ArrayNx4x4

__all__ = ["OptimisationsReadWriter"]


class OptimisationsReadWriter:
    """
    Represents utility class for reading/writing optimised poses and point clouds.

    Poses format is:
    r11 r12 r13 tx
    r21 r22 r23 ty
    r31 r32 r33 tz
    0   0   0   1
    """

    @staticmethod
    def read_pose(filepath: str) -> ArrayNx4x4[float]:
        """
        Read pose from file

        Parameters
        ----------
        filepath: str
            Path to file which contains pose values

        Returns
        -------
        pose: ArrayNx4x4[float]
            Matrix which contains relevant values
        """
        pose = np.eye(4)
        with open(filepath) as file:
            lines = file.readlines()
            for ind, line in enumerate(lines):
                pose[ind, :4] = np.array(list(map(float, line.split(" "))))

        return pose

    @staticmethod
    def read_point_cloud(filepath: str) -> o3d.geometry.PointCloud:
        return o3d.io.read_point_cloud(filepath)

    @staticmethod
    def write_pose(filepath: str, pose: Array4x4[float]) -> None:
        """
        Writes pose values to file

        Parameters
        ----------
        filepath: str
            Path to file which would be contained pose values
        pose: Array4x4[float]
            Matrix with pose values to write
        """
        assert pose.shape == (4, 4)

        with open(filepath, "w+") as file:
            for ind, pose_line in enumerate(pose):
                file.write(" ".join(str(x) for x in pose_line))

                if ind != pose.shape[0]:
                    file.write("\n")

    @staticmethod
    def write_point_cloud(filepath: str, point_cloud: o3d.geometry.PointCloud):
        o3d.io.write_point_cloud(filepath, point_cloud)
