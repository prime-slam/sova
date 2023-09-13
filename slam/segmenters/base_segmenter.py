from abc import ABC, abstractmethod
from typing import List

from slam.typing.hints import ArrayNx3

__all__ = ["Segmenter"]


class Segmenter(ABC):
    """
    Represents abstract class for planes segmentation mechanisms.
    """

    @abstractmethod
    def segment(self, points: ArrayNx3[float]) -> List[int]:
        """
        Represent main abstract method to segment point cloud

        Parameters
        ----------
        points: ArrayNx3[float]
            3D points are used to segment planes

        Returns
        -------
        inliers: List[int]
            Indices of inlier (segmented) points
        """
        pass
