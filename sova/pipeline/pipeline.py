import open3d as o3d
from octreelib.grid import GridConfig, VisualizationConfig

import copy
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

from sova.backend.backend import Backend, BackendOutput
from sova.filter.filter import Filter
from sova.segmenter import Segmenter
from sova.subdivider.subdivider import Subdivider
from sova.typing.hints import ArrayNx4x4

__all__ = ["PipelineRuntimeParameters", "Pipeline"]


@dataclass
class PipelineRuntimeParameters(ABC):
    """
    Represents parameters for pipeline to run

    Parameters
    ----------
    grid_configuration: GridConfig
        Represents octreelib Grid configuration
    visualization_config: VisualizationConfig
        Represents configuration for result visualization
    """

    grid_configuration: GridConfig = GridConfig
    visualization_config: VisualizationConfig = VisualizationConfig()


class Pipeline(ABC):
    """
    Represents abstract class for SLAM pipeline which takes point clouds (and poses) and produces optimised poses
    with relevant optimising metrics

    Parameters
    ----------
    point_clouds: List[o3d.geometry.PointCloud]
        Point clouds in local coordinates
    poses: ArrayNx4x4[float]
        Poses of given point clouds, that transforms them from local to global coordinates
    subdividers: List[Subdivider]
        Subdivider conditions to subdivide voxels in grid
    segmenters: List[Segmenter]
        List of segmenter-algorithms which leave only planar features in voxels
    filters: List[Filter]
        Filter conditions to filter voxels in grid
    backend: Backend
        Backend of SLAM algorithm that optimises given poses
    debug: bool
        Represents debug parameter. If it is specified, the pipeline will save the visualization files.
    """

    def __init__(
        self,
        point_clouds: List[o3d.geometry.PointCloud],
        poses: ArrayNx4x4[float],
        subdividers: List[Subdivider],
        segmenters: List[Segmenter],
        filters: List[Filter],
        backend: Backend,
        debug: bool,
    ) -> None:
        if len(point_clouds) != len(poses):
            raise ValueError("Sizes of point_cloud and poses arrays must be equal")

        self._point_clouds: List[o3d.geometry.PointCloud] = point_clouds
        self._poses: ArrayNx4x4[float] = poses
        self._subdividers: List[Subdivider] = subdividers
        self._segmenters: List[Segmenter] = segmenters
        self._filters: List[Filter] = filters
        self._backend: Backend = backend
        self._debug: bool = debug

    @abstractmethod
    def run(self, parameters: PipelineRuntimeParameters) -> BackendOutput:
        """
        Represents abstract method to run slam and produce output

        Parameters
        ----------
        parameters: PipelineRuntimeParameters
            Represents utility parameters to run pipeline

        Returns
        -------
        output: BackendOutput
            Structural SLAM result, which contains optimized poses and related metrics
        """
        pass

    def _transform_point_clouds(self) -> List[o3d.geometry.PointCloud]:
        """
        Represents protected method to transform points from local to global coordinates
        using given poses

        Returns
        -------
        transformed_point_clouds: List[o3d.geometry.PointCloud]
            List of point clouds in global coordinates
        """
        return [
            copy.deepcopy(point_cloud).transform(pose)
            for point_cloud, pose in zip(self._point_clouds, self._poses)
        ]
