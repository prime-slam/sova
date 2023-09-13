import open3d as o3d

from typing import List

from slam.segmenters.base_segmenter import Segmenter
from slam.typing.hints import ArrayNx3

__all__ = ["RansacSegmenter"]


class RansacSegmenter(Segmenter):
    """
    Represent RANSAC-based mechanism to segment planes

    Parameters
    ----------
    threshold: float
        Max distance a point can be from the plane model, and still be considered an inlier
    initial_points: int
        Number of initial points to be considered inliers in each iteration
    iterations: int
        Number of RANSAC iterations
    """

    def __init__(
        self,
        threshold: float = 0.1,
        initial_points: int = 3,
        iterations: int = 5000,
    ) -> None:
        if threshold <= 0:
            raise ValueError("Threshold must be positive")
        if initial_points < 3:
            raise ValueError("Initial points count must be more or equal than three")
        if iterations < 1:
            raise ValueError("Number of RANSAC iterations must be positive")

        self.__threshold: float = threshold
        self.__initial_points: int = initial_points
        self.__iterations: int = iterations

    def segment(self, points: ArrayNx3[float]) -> List[int]:
        """
        Segments given points using RANSAC method

        Parameters
        ----------
        points: ArrayNx3[float]
            3D points are used to segment planes using RANSAC

        Returns
        -------
        inliers: List[int]
            Indices of inlier (segmented) points
        """
        if len(points) == 0:
            raise ValueError("Length of points list must be positive")

        point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        # TODO: Handle RuntimeError exception in case len(inliers) < ransac_n
        _, inliers = point_cloud.segment_plane(
            distance_threshold=self.__threshold,
            ransac_n=self.__initial_points,
            num_iterations=self.__iterations,
        )

        return inliers
