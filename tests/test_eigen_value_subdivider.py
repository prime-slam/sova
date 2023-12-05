import numpy as np
import pytest

import random

from slam.subdivider import EigenValueSubdivider
from slam.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, eigen_value, expected_decision",
    [
        (
            [
                np.array(
                    [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]
                )
                for _ in range(100)
            ],
            1,
            False,
        ),
        (
            [
                np.array(
                    [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]
                )
                for _ in range(100)
            ],
            1000,
            False,
        ),
        (
            np.empty((0, 3)),
            1,
            False,
        ),
    ],
)
def test_eigen_value_subdivider(
    points: ArrayNx3[float],
    eigen_value: float,
    expected_decision: bool,
):
    eigen_value_subdivider = EigenValueSubdivider(value=eigen_value)
    assert expected_decision == eigen_value_subdivider(points)
