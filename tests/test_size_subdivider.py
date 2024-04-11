import numpy as np
import pytest

from sova.subdivider import SizeSubdivider
from sova.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, size, expected_decision",
    [
        (
            np.array(
                [
                    [0, 0, 0],
                    [0, 1, 0],
                    [1, 1, 1],
                ]
            ),
            0.5,
            True,
        ),
        (
            np.array(
                [
                    [0, 0, 0],
                    [0.1, 0.1, 0.1],
                ]
            ),
            0.5,
            False,
        ),
        (
            np.empty((0, 3)),
            0.5,
            False,
        ),
    ],
)
def test_size_subdivider(
    points: ArrayNx3[float],
    size: float,
    expected_decision: bool,
):
    size_subdivider = SizeSubdivider(size=size)
    assert expected_decision == size_subdivider(points)
