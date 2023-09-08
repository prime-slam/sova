import numpy as np
import open3d as o3d

from slam.typing.hints import ArrayNx3
from slam.octree.segmenters.base_plane_segmenter import PlaneSegmenters
from typing import List


__all__ = ["RansacPlaneSegmenter"]


class RansacPlaneSegmenter(PlaneSegmenters):
    """
    Represent RANSAC-based mechanism to segment planes

    Parameters
    ----------
    threshold: np.float64
        Max distance a point can be from the plane model, and still be considered an inlier
    initial_points: int
        Number of initial points to be considered inliers in each iteration
    iterations: int
        Number of RANSAC iterations
    """

    def __init__(
        self,
        threshold: np.float64 = 0.1,
        initial_points: int = 3,
        iterations: int = 5000,
    ) -> None:
        if threshold <= 0:
            raise ValueError("Threshold must be positive")
        if initial_points < 1:
            raise ValueError("Initial points count must be more than zero")
        if iterations < 1:
            raise ValueError("Number of RANSAC iterations must be positive")

        self.threshold: np.float64 = threshold
        self.initial_points: int = initial_points
        self.iterations: int = iterations

    def segment(self, points: ArrayNx3[np.float64]) -> List[int]:
        """
        Segments given points using RANSAC method

        Parameters
        ----------
        points: ArrayNx3[np.float64]
            3D points, which uses to segment planes using RANSAC

        Returns
        -------
        inliers: List[int]
            Indices of inlier (segmented) points
        """
        if len(points) == 0:
            raise ValueError("Length of points list must be positive")

        point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        _, inliers = point_cloud.segment_plane(
            distance_threshold=self.threshold,
            ransac_n=self.initial_points,
            num_iterations=self.iterations,
        )

        return inliers
