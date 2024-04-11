from sova.segmenter.segmenter import Segmenter
from sova.typing import ArrayNx3

__all__ = ["IdenticalSegmenter"]


class IdenticalSegmenter(Segmenter):
    """
    Represents full plane segmenter, which returns all points of given points
    """

    def __call__(self, points: ArrayNx3[float]) -> ArrayNx3[float]:
        """
        Returns given points
        (was created in debugging purposes)

        Parameters
        ----------
        points: ArrayNx3[float]
            3D points are used to segment plane

        Returns
        -------
        segmented_points: ArrayNx3[float]
            List of 3D segmented points after processing the Identical-algorithm
        """
        return points
