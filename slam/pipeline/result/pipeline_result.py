from typing import List

from slam.pipeline.result.metric_base import Metric
from slam.typing.hints import ArrayNx4x4


class PipelineResult:
    def __init__(self, poses: ArrayNx4x4[float], metrics: List[Metric]) -> None:
        self.poses: ArrayNx4x4[float] = poses
        self.metrics: List[Metric] = metrics

    @property
    def poses(self) -> ArrayNx4x4[float]:
        return self.poses

    def __str__(self) -> str:
        return "\n".join(
            [f"{metric.name}: {metric.value}" for metric in self.metrics]
        )
