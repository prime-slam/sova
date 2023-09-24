import open3d as o3d

from typing import List
from abc import ABC, abstractmethod

from slam.pipeline.backend.backend_base import Backend
from slam.pipeline.filters.filter_base import Filter
from slam.pipeline.result.pipeline_result import PipelineResult
from slam.pipeline.subdividers.subdivider_base import Subdivider
from slam.typing import ArrayNx4x4

__all__ = ["Pipeline"]


class Pipeline(ABC):
    """
    Represents base of pipeline class, which takes point cloud (and poses) and produces
    optimised poses

    Parameters
    ----------
    point_clouds: List[o3d.geometry.PointCloud]
        List of given point clouds
    poses: ArrayNx4x4[float]
        Poses of given point clouds
    subdividers: List[Subdivider]
        List of subdivider conditions
    filters: List[Filter]
        List of filter conditions
    backend: Backend
        Backend of SLAM algorithm
    """
    def __init__(
            self,
            point_clouds: List[o3d.geometry.PointCloud],
            poses: ArrayNx4x4[float],
            subdividers: List[Subdivider],
            filters: List[Filter],
            backend: Backend,
    ) -> None:
        if len(point_clouds) != len(poses):
            raise ValueError("Sizes of point_cloud and poses arrays must be equal")

        self.point_clouds: List[o3d.geometry.PointCloud] = point_clouds
        self.poses: ArrayNx4x4[float] = poses
        self.subdividers: List[Subdivider] = subdividers
        self.filters: List[Filter] = filters
        self.backend: Backend = backend

    @abstractmethod
    def run(self) -> PipelineResult:
        """
        Represents abstract method to run pipeline and produce output

        Returns
        -------
        result: PipelineResult
            Structural pipeline result
        """
        pass

    def __transform_point_clouds(self) -> List[o3d.geometry.PointCloud]:
        """
        Represents private method to transform points from local coordinates to global

        Returns
        -------
        transformed_point_clouds: List[o3d.geometry.PointCloud]
            List of point clouds in global coordinates
        """
        return [
            point_cloud.transform(pose) for point_cloud, pose in zip(self.point_clouds, self.poses)
        ]
