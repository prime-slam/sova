import numpy as np

from abc import ABC, abstractmethod
from typing import List

from slam.typing.hints import ArrayNx3

__all__ = ["PlaneSegmenter"]


class PlaneSegmenter(ABC):
    """
    Represents abstract class for planes segmentation mechanisms.
    """

    @abstractmethod
    def segment(self, points: ArrayNx3[np.float64]) -> List[int]:
        """
        Represent main abstract method to segment point cloud

        Parameters
        ----------
        points: ArrayNx3[np.float64]
            3D points, which uses to segment planes

        Returns
        -------
        inliers: List[int]
            Indices of inlier (segmented) points
        """
        pass
