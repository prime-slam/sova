import numpy as np
import pytest

from slam.segmenter import CountSegmenter
from slam.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, count, expected_points",
    [
        (np.array([[0, 0, 0]]), 0, np.array([[0, 0, 0]])),
        (np.array([[0, 0, 0]]), 10, np.empty((0, 3))),
    ],
)
def test_count_segmenter(
    points: ArrayNx3[float],
    count: int,
    expected_points: ArrayNx3[float],
):
    count_segmenter = CountSegmenter(count=count)
    actual_points = count_segmenter(points)
    assert len(actual_points) == len(expected_points)
    assert np.all(actual_points == expected_points)
