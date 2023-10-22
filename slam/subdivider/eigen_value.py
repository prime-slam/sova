import numpy as np

from slam.subdivider.subdivider import Subdivider
from slam.typing import ArrayNx3

__all__ = ["EigenValueSubdivider"]


class EigenValueSubdivider(Subdivider):
    """
    Represents "PCA minimum eigen value" condition: if it is more than predefined
    value, it should be split.

    Parameters
    ----------
    value: float
        Minimum value for eigen value
    """

    def __init__(self, value: float) -> None:
        self.__value: float = value

    def __call__(self, points: ArrayNx3[float]) -> bool:
        """
        Represent "eigen-value"-based subdivider mechanism which returns True/False statement based on points
        PCA calculations

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        should_be_split: bool
            Returns False if minimum value of points covariance matrix less than predefined value, otherwise
            returns True
        """
        if len(points) < 3:
            return False

        points = np.asarray(points)
        standardized_data = (points - points.mean(axis=0)) / points.std(axis=0)
        covariance_matrix = np.cov(standardized_data, ddof=1, rowvar=False)
        eigenvalues, _ = np.linalg.eig(covariance_matrix)
        decrease_order = np.argsort(eigenvalues)[::-1]
        _, _, min_eigenvalue = eigenvalues[decrease_order]

        return min_eigenvalue > self.__value
