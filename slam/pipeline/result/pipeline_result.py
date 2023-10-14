from typing import List

from slam.pipeline.result.metric_base import Metric
from slam.typing.hints import ArrayNx4x4

__all__ = ["PipelineResult"]


class PipelineResult:
    """
    Represents class for structural output of pipeline

    Parameters
    ----------
    poses: ArrayNx4x4[float]
        Optimised poses
    metrics: List[Metric]
        List of calculated metrics
    """

    def __init__(self, poses: ArrayNx4x4[float], metrics: List[Metric]) -> None:
        self._poses: ArrayNx4x4[float] = poses
        self._metrics: List[Metric] = metrics

    @property
    def poses(self) -> ArrayNx4x4[float]:
        """
        Represents method to return optimised poses
        (for instance, calling from main.py file, to get statistic information)

        Returns
        -------
        poses: ArrayNx4x4[float]
            Optimised poses
        """
        return self.poses

    def __str__(self) -> str:
        """
        Represents implementation of str dunder method to produce pipeline result pretty print

        Returns
        -------
        string: str
            String representation of pipeline result
        """
        return "\n".join([f"{metric.name}: {metric.value}" for metric in self._metrics])
