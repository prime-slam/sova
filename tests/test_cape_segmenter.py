import numpy as np
import pytest

import random

from sova.segmenter import CAPESegmenter
from sova.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, correlation, points_size",
    [
        (
            [
                np.array(
                    [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]
                )
                for _ in range(100)
            ],
            100,
            100,
        ),
        (
            [
                np.array(
                    [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]
                )
                for _ in range(100)
            ],
            0,
            0,
        ),
    ],
)
def test_cape_segmenter(
    points: ArrayNx3[float],
    correlation: float,
    points_size: int,
):
    cape_segmenter = CAPESegmenter(correlation)
    actual_points = cape_segmenter(points)
    assert len(actual_points) == points_size
