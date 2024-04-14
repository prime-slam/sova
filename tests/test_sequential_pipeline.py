import mrob
import numpy as np
import open3d as o3d
import pytest
from octreelib.grid import GridConfig, VisualizationConfig

import os
from typing import List

from sova.backend import Backend, EigenFactorBackend
from sova.filter import Filter
from sova.pipeline import SequentialPipeline, SequentialPipelineRuntimeParameters
from sova.segmenter import CountSegmenter, Segmenter
from sova.subdivider import CountSubdivider, Subdivider
from sova.typing import ArrayNx3, ArrayNx4x4


@pytest.mark.parametrize(
    "points," "poses," "subdividers," "filters," "segmenters," "backend," "debug,",
    [
        (
            np.array(
                [
                    [0, 0, 0],
                    [1, 1, 1],
                ]
            ),
            [np.eye(4)],
            [CountSubdivider(10)],
            [],
            [CountSegmenter(0)],
            EigenFactorBackend(
                poses_number=1, iterations_number=1, robust_type=mrob.HUBER
            ),
            False,
        ),
    ],
)
def test_sequential_pipeline(
    points: ArrayNx3[float],
    poses: ArrayNx4x4[float],
    subdividers: List[Subdivider],
    filters: List[Filter],
    segmenters: List[Segmenter],
    backend: Backend,
    debug: bool,
):
    sequential_pipeline = SequentialPipeline(
        point_clouds=[o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))],
        poses=poses,
        subdividers=subdividers,
        segmenters=segmenters,
        filters=filters,
        backend=backend,
        debug=debug,
    )
    output = sequential_pipeline.run(
        SequentialPipelineRuntimeParameters(
            grid_configuration=GridConfig(voxel_edge_length=2),
            visualization_config=VisualizationConfig(
                filepath=os.devnull,
            ),
            initial_point_cloud_number=0,
        )
    )

    assert output is not None
