import open3d as o3d

from typing import List

from slam.octree import Octree
from slam.pipeline.backend.backend_base import Backend
from slam.pipeline.filters.filter_base import Filter
from slam.pipeline.result.pipeline_result import PipelineResult
from slam.pipeline.subdividers.subdivider_base import Subdivider
from slam.typing import ArrayNx4x4


class Pipeline:
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

    def run(self) -> PipelineResult:
        transformed_point_clouds = self.__transform_point_clouds()
        octree = Octree()  # There will be call of Michael octree implementation

        for pose_number, point_cloud in enumerate(transformed_point_clouds):
            octree.insert(pose_number, point_cloud)

        octree.subdivide(self.subdividers)

        octree.filter(self.filters)

        return self.backend.process(octree)

    def __transform_point_clouds(self) -> List[o3d.geometry.PointCloud]:
        return [
            point_cloud.transform(pose) for point_cloud, pose in zip(self.point_clouds, self.poses)
        ]
