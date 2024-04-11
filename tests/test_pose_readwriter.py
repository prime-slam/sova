import numpy as np
import pytest

import os

from sova.typing import ArrayNx4x4
from sova.utils import OptimisedPoseReadWriter


@pytest.mark.parametrize(
    "pose_path, expected_pose",
    [
        (
            "tests/data/poses/correct_pose.txt",
            np.array(
                [
                    [1, 2, 3, 4],
                    [1, 2, 3, 4],
                    [1, 2, 3, 4],
                    [0, 0, 0, 1],
                ]
            ),
        ),
    ],
)
def test_correct_pose_readwriter_read(
    pose_path: str,
    expected_pose: ArrayNx4x4[float],
):
    pose_readwriter = OptimisedPoseReadWriter()
    actual_pose = pose_readwriter.read(pose_path)
    assert np.all(actual_pose == expected_pose)


@pytest.mark.parametrize(
    "pose_path",
    [
        ("tests/data/poses/incorrect_pose.txt"),
    ],
)
def test_incorrect_pose_readwriter_read(
    pose_path: str,
):
    pose_readwriter = OptimisedPoseReadWriter()
    with pytest.raises(ValueError):
        pose_readwriter.read(pose_path)


@pytest.mark.parametrize(
    "pose_path, pose_to_write",
    [
        (
            "tests/data/poses/temp1.txt",
            np.array(
                [
                    [1, 2, 3, 4],
                    [1, 2, 3, 4],
                    [1, 2, 3, 4],
                    [0, 0, 0, 1],
                ]
            ),
        )
    ],
)
def test_correct_pose_readwriter_write(
    pose_path: str,
    pose_to_write: ArrayNx4x4[float],
):
    pose_readwriter = OptimisedPoseReadWriter()
    pose_readwriter.write(pose_path, pose_to_write)

    actual_pose = pose_readwriter.read(pose_path)
    os.remove(pose_path)

    assert np.all(actual_pose == pose_to_write)


@pytest.mark.parametrize(
    "pose_path, pose_to_write",
    [
        (
            "tests/data/poses/temp2.txt",
            np.array(
                [
                    [1, 2, 3, 4],
                ]
            ),
        )
    ],
)
def test_incorrect_pose_readwriter_write(
    pose_path: str,
    pose_to_write: ArrayNx4x4[float],
):
    pose_readwriter = OptimisedPoseReadWriter()
    with pytest.raises(AssertionError):
        pose_readwriter.write(pose_path, pose_to_write)
