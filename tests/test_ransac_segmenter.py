import numpy as np
import pytest

from slam.segmenter import RansacSegmenter
from slam.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, threshold, expected_points",
    [
        (
            np.array([[0, 0, 0], [0, 0, 1], [0, 1, 0]]),
            100,
            np.array([[0, 0, 0], [0, 0, 1], [0, 1, 0]]),
        ),
        (
            np.array([[0, 0, 0], [0, 0, 1], [0, 1, 0], [0, 1, 1], [100, 100, 100]]),
            0.001,
            np.array([[0, 0, 0], [0, 0, 1], [0, 1, 0], [0, 1, 1]]),
        ),
    ],
)
def test_ransac_segmenter(
    points: ArrayNx3[float],
    threshold: float,
    expected_points: ArrayNx3[float],
):
    ransac_segmenter = RansacSegmenter(threshold=threshold, initial_points=3)
    actual_points = ransac_segmenter(points)
    assert len(actual_points) == len(expected_points)
    assert np.all(actual_points == expected_points)
