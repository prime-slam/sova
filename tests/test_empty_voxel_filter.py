import numpy as np
import pytest

from slam.filter import EmptyVoxel
from slam.typing import ArrayNx3


@pytest.mark.parametrize(
    "points, expected_decision",
    [
        (np.array([0, 0, 0]), True),
        (np.empty(0), False),
    ],
)
def test_empty_voxel(points: ArrayNx3[float], expected_decision: bool):
    empty_voxel_filter = EmptyVoxel()
    assert empty_voxel_filter(points) == expected_decision
