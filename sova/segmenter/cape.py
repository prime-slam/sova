import numpy as np
from sklearn.decomposition import PCA

from sova.segmenter.segmenter import Segmenter
from sova.typing import ArrayNx3

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
    debug: bool = False
        Represents parameter for printing debug information to stdout
    """

    def __init__(self, correlation: float, debug: bool = False) -> None:
        self.__correlation: float = correlation
        self.__debug: bool = debug

    def __call__(self, points: ArrayNx3[float]) -> ArrayNx3[float]:
        """
        Represent CAPE planar condition.

        Parameters
        ----------
        points: ArrayNx3[float]
            Points of point cloud (or octree voxel)

        Returns
        -------
        segmented_points: ArrayNx3[float]
            List of 3D segmented points after processing the CAPE condition
        """
        if len(points) <= 10:
            return np.empty((0, 3), dtype=float)

        points = np.asarray(points)

        pca = PCA(n_components=3)
        try:
            standardized_points = (points - points.mean(axis=0)) / points.std(axis=0)
            pca.fit_transform(standardized_points)
            min_eigenvalue, _, max_eigenvalue = sorted(pca.explained_variance_)

            if max_eigenvalue / min_eigenvalue > self.__correlation:
                return np.empty((0, 3), dtype=float)
        except Exception as ex:
            if self.__debug:
                print(ex)

            return np.empty((0, 3), dtype=float)

        return points
