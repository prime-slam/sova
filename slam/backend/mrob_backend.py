from abc import abstractmethod
from typing import Dict

import mrob
from octreelib.grid import GridBase

from slam.backend.backend import Backend, BackendOutput, Metric

__all__ = ["MROBBackend"]


class MROBBackend(Backend):
    """
    Represents abstract class for mrob-like backends
    Library: https://github.com/prime-slam/mrob

    Parameters
    ----------
    poses_number: int
        Number of poses in provided map
    iterations_number: int
        Number of iterations that will be produced by MROB-EF backend.
    """
    def __init__(self, poses_number: int, iterations_number: int) -> None:
        self._graph: mrob.FGraph = mrob.FGraph()
        self._poses_number: int = poses_number
        self._iterations_number: int = iterations_number
        self._planes: Dict[int, int] = {}

    def _init_poses(self):
        """
        Initializes pose nodes in mrob graph
        """
        self._graph.add_node_pose_3d(mrob.geometry.SE3(), mrob.NODE_ANCHOR)
        for _ in range(self._poses_number - 1):
            self._graph.add_node_pose_3d(
                mrob.geometry.SE3(), mrob.NODE_STANDARD
            )

    @abstractmethod
    def _init_point_clouds(self, grid: GridBase) -> None:
        """
        Represents abstract method for initiating plane features
        """
        pass

    def process(self, grid: GridBase) -> BackendOutput:
        """
        Represents implementation of Backend abstract class, which
        takes remaining points from poses (Octree, essentially) and optimises using them.

        Parameters
        ----------
        grid: GridBase
            Represents grid with all inserted point clouds

        Returns
        -------
        output: BackendOutput
            Result of backend optimisations
        """
        self._init_poses()
        self._init_point_clouds(grid)
        metrics = [
            Metric(name="FGraph initial error", value=self._graph.chi2(True)),
        ]
        converge_iterations = self._graph.solve(
            mrob.LM_ELLIPS, self._iterations_number
        )
        while converge_iterations == 0:
            print("Iterations equals 0")
            converge_iterations = self._graph.solve(
                mrob.LM_ELLIPS, self._iterations_number
            )

        metrics.append(
            Metric(name="Iterations to converge", value=converge_iterations),
        )
        metrics.append(
            Metric(name="chi2", value=self._graph.chi2()),
        )

        return BackendOutput(self._graph.get_estimated_state(), metrics)
