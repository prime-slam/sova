import numpy as np
import pytest

from slam.segmenter import IdenticalSegmenter
from slam.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, expected_points",
    [
        (np.array([[0, 0, 0]]), np.array([[0, 0, 0]])),
        (np.empty(0), np.empty(0)),
    ],
)
def test_identical_segmenter(
    points: ArrayNx3[float],
    expected_points: ArrayNx3[float],
):
    identical_segmenter = IdenticalSegmenter()
    actual_points = identical_segmenter(points)
    assert len(actual_points) == len(expected_points)
    assert np.all(actual_points == expected_points)
