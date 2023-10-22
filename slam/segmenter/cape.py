import numpy as np

from slam.segmenter.segmenter import Segmenter
from slam.typing import ArrayNx3

__all__ = ["CAPESegmenter"]


class CAPESegmenter(Segmenter):
    """
    Represents CAPE "max/min eigenvalues correlation" segmenter: if max_eigen_value/min_eigen_value > value
    than this voxel considers as planar, otherwise it should be deleted.
    More information: https://arxiv.org/pdf/1803.02380.pdf

    Parameters
    ----------
    correlation: float
        Lower bound of max/min eigen values correlation
    """

    def __init__(self, correlation: float) -> None:
        self.__correlation: float = correlation

    def __call__(self, points: ArrayNx3[float]) -> ArrayNx3[float]:
        """
        Represent count-based subdivider mechanism which returns True/False statement based on points count

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        should_be_split: bool
            Returns
        """
        if len(points) <= 10:
            return []

        converted_points = np.asarray(points)
        standardized_data = (
            converted_points - converted_points.mean(axis=0)
        ) / converted_points.std(axis=0)
        covariance_matrix = np.cov(standardized_data, ddof=1, rowvar=False)
        eigenvalues, _ = np.linalg.eig(covariance_matrix)
        decrease_order = np.argsort(eigenvalues)[::-1]
        max_eigenvalue, _, min_eigenvalue = eigenvalues[decrease_order]

        if max_eigenvalue / min_eigenvalue < self.__correlation:
            return points

        return []
