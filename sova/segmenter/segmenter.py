from abc import ABC, abstractmethod

from sova.typing.hints import ArrayNx3

__all__ = ["Segmenter"]


class Segmenter(ABC):
    """
    Represents abstract class for plane segmentation mechanisms
    """

    @abstractmethod
    def __call__(self, points: ArrayNx3[float]) -> ArrayNx3[float]:
        """
        Represent abstract dunder method to segment plane from given points

        Parameters
        ----------
        points: ArrayNx3[float]
            3D points which are used to segment plane

        Returns
        -------
        segmented_points: ArrayNx3[float]
            List of 3D segmented points after processing the algorithm
        """
        pass
