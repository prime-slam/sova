from octreelib.grid import GridBase

from abc import ABC, abstractmethod
from typing import List

from slam.typing import ArrayNx4x4

__all__ = ["Metric", "BackendOutput", "Backend"]


class Metric:
    """
    Represents Backend's metric of Voxel SLAM algorithm
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


class BackendOutput:
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
        return self._poses

    def __str__(self) -> str:
        """
        Represents implementation of str dunder method to produce slam result pretty print

        Returns
        -------
        string: str
            String representation of slam result
        """
        return "\n".join([f" {metric.name}: {metric.value}" for metric in self._metrics])


class Backend(ABC):
    """
    Represents base (abstract) class for backend implementations
    """

    @abstractmethod
    def process(self, grid: GridBase) -> BackendOutput:
        """
        Represents base method of backend, which should produce Pipeline result by
        given octree

        Parameters
        ----------
        grid: GridBase
            Represents grid with all inserted point clouds

        Returns
        -------
        output: PipelineOutput
            Result of backend optimisations
        """
        pass
