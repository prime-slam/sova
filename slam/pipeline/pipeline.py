import open3d as o3d

import copy
from abc import ABC, abstractmethod
from typing import List

from slam.backend.backend import Backend
from slam.filter.filter import Filter
from slam.segmenter import Segmenter
from slam.subdivider.subdivider import Subdivider
from slam.typing.hints import ArrayNx4x4

__all__ = ["Metric", "PipelineOutput", "Pipeline"]


class Metric:
    """
    Represents metric of slam
    """

    @abstractmethod
    def __init__(self, name: str, value: float) -> None:
        self._name = name
        self._value = value

    @property
    def name(self) -> str:
        """
        Represents method to get name of metric

        Returns
        -------
        name: str
            Metric's name
        """
        return self._name

    @property
    def value(self) -> float:
        """
        Represents method to get value of metric

        Returns
        -------
        value: float
            Metric's value
        """
        return self._value


class PipelineOutput:
    """
    Represents slam result (output) with all necessary artifacts

    Parameters
    ----------
    poses: ArrayNx4x4[float]
        Optimised poses produced by one of backends
    metrics: List[Metric]
        Metrics which allows to evaluate the resulting optimizations
    """

    def __init__(self, poses: ArrayNx4x4[float], metrics: List[Metric]) -> None:
        self._poses: ArrayNx4x4[float] = poses
        self._metrics: List[Metric] = metrics

    @property
    def poses(self) -> ArrayNx4x4[float]:
        """
        Represents method to get optimised poses

        Returns
        -------
        poses: ArrayNx4x4[float]
            Optimised poses
        """
        return self.poses

    def __str__(self) -> str:
        """
        Represents implementation of str dunder method to produce slam result pretty print

        Returns
        -------
        string: str
            String representation of slam result
        """
        return "\n".join([f"{metric.name}: {metric.value}" for metric in self._metrics])


class Pipeline(ABC):
    """
    Represents abstract slam, which takes point clouds (and poses) and produces optimised poses
    with relevant optimising metrics

    Parameters
    ----------
    point_clouds: List[o3d.geometry.PointCloud]
        Point clouds in local coordinates
    poses: ArrayNx4x4[float]
        Poses of given point clouds, that transforms them from local to global coordinates
    subdividers: List[Subdivider]
        Subdivider conditions to subdivide voxels in grid
    segmenter: Segmenter
        Segmenter-algorithm which leaves only planar features in voxels
    filters: List[Filter]
        Filter conditions to filter voxels in grid
    backend: Backend
        Backend of SLAM algorithm that optimises given poses
    """

    def __init__(
        self,
        point_clouds: List[o3d.geometry.PointCloud],
        poses: ArrayNx4x4[float],
        subdividers: List[Subdivider],
        segmenter: Segmenter,
        filters: List[Filter],
        backend: Backend,
    ) -> None:
        if len(point_clouds) != len(poses):
            raise ValueError("Sizes of point_cloud and poses arrays must be equal")

        self._point_clouds: List[o3d.geometry.PointCloud] = point_clouds
        self._poses: ArrayNx4x4[float] = poses
        self._subdividers: List[Subdivider] = subdividers
        self._segmenter: Segmenter = segmenter
        self._filters: List[Filter] = filters
        self._backend: Backend = backend

    @abstractmethod
    def run(self) -> PipelineOutput:
        """
        Represents abstract method to run slam and produce output

        Returns
        -------
        result: PipelineResult
            Structural slam result
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
