import numpy as np

from typing import List

from slam.segmenters.base_segmenter import Segmenter
from slam.typing import ArrayNx3


class FullSegmenter(Segmenter):
    """
    Represents full plane segmenter, which returns all indices of given point cloud.
    """

    def segment(self, points: ArrayNx3[float]) -> List[int]:
        """
        Returns all indices of given point cloud
        """
        return np.arange(len(points)).tolist()
