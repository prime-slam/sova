import numpy as np
from sklearn.decomposition import PCA

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
            return []

        points = np.asarray(points)
        standardized_points = (points - points.mean(axis=0)) / points.std(axis=0)

        try:
            standardized_points = (points - points.mean(axis=0)) / points.std(axis=0)
            covariance_matrix = np.cov(standardized_points, ddof=1, rowvar=False)
            eigenvalues, _ = np.linalg.eig(covariance_matrix)
            decrease_order = np.argsort(eigenvalues)[::-1]
            max_eigenvalue, mean_eigenvalue, min_eigenvalue = eigenvalues[
                decrease_order
            ]

            if max_eigenvalue / min_eigenvalue > self.__correlation:
                return []
        except Exception as ex:
            print(ex)

        # pca = PCA(n_components=3)
        # try:
        #     pca.fit_transform(standardized_points)
        #     min_eigenvalue, _, max_eigenvalue = sorted(pca.explained_variance_)
        #
        #     if max_eigenvalue / min_eigenvalue > self.__correlation:
        #         return []
        # except Exception as ex:
        #     ax = ex
        #
        #     return []

        return points
