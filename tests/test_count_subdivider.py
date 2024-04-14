import numpy as np
import pytest

from sova.subdivider import CountSubdivider
from sova.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, count, expected_decision",
    [
        (
            np.array(
                [
                    [0, 0, 0],
                    [0, 1, 0],
                    [1, 1, 1],
                ]
            ),
            1,
            True,
        ),
        (
            np.array(
                [
                    [0, 0, 0],
                    [0, 1, 0],
                    [1, 1, 1],
                ]
            ),
            5,
            False,
        ),
        (
            np.empty((0, 3)),
            1,
            False,
        ),
    ],
)
def test_count_subdivider(
    points: ArrayNx3[float],
    count: float,
    expected_decision: bool,
):
    count_subdivider = CountSubdivider(count=count)
    assert expected_decision == count_subdivider(points)
