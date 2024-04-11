import numpy as np
import open3d as o3d

from sova.segmenter.segmenter import Segmenter
from sova.typing.hints import ArrayNx3

__all__ = ["RansacSegmenter"]


class RansacSegmenter(Segmenter):
    """
    Represent RANSAC-based mechanism which segments plane from given voxel

    Parameters
    ----------
    threshold: float
        Max distance a point can be from the plane model, and still be considered an inlier
    initial_points: int
        Number of initial points to be considered inliers in each iteration
    iterations: int
        Number of RANSAC iterations
    debug: bool = False
        Represents parameter for printing debug information to stdout
    """

    def __init__(
        self,
        threshold: float = 0.1,
        initial_points: int = 3,
        iterations: int = 5000,
        debug: bool = False,
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
        self.__debug: bool = debug

    def __call__(self, points: ArrayNx3[float]) -> ArrayNx3[float]:
        """
        Segments given points using RANSAC method

        Parameters
        ----------
        points: ArrayNx3[float]
            3D points are used to segment plane using RANSAC

        Returns
        -------
        segmented_points: ArrayNx3[float]
            List of 3D segmented points after processing the RANSAC-algorithm
        """
        if len(points) == 0:
            raise ValueError("Length of points list must be positive")

        point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        try:
            _, inliers = point_cloud.segment_plane(
                distance_threshold=self.__threshold,
                ransac_n=self.__initial_points,
                num_iterations=self.__iterations,
            )

            # TODO: Remove after migration to numpy in octreelib
            inlier_cloud = point_cloud.select_by_index(inliers)

            return np.asarray(inlier_cloud.points)
        except Exception as ex:
            if self.__debug:
                print(ex)

        return np.empty((0, 3), dtype=float)
