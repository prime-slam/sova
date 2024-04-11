import numpy as np
from sklearn.decomposition import PCA

from sova.subdivider.subdivider import Subdivider
from sova.typing import ArrayNx3

__all__ = ["EigenValueSubdivider"]


class EigenValueSubdivider(Subdivider):
    """
    Represents "PCA minimum eigen value" condition: if it is more than predefined
    value, it should be split.

    Parameters
    ----------
    value: float
        Minimum value for eigen value
    debug: bool = False
        Represents parameter for printing debug information to stdout
    """

    def __init__(self, value: float, debug: bool = False) -> None:
        self.__value: float = value
        self.__debug: bool = debug

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

        pca = PCA(n_components=3)
        try:
            standardized_points = (points - points.mean(axis=0)) / points.std(axis=0)
            pca.fit_transform(standardized_points)
            min_eigenvalue, _, _ = sorted(pca.explained_variance_)

            return min_eigenvalue > self.__value
        except Exception as ex:
            if self.__debug:
                print(ex)

        return False
