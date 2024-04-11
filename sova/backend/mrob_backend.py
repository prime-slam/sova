import mrob
from octreelib.grid import GridBase

from abc import abstractmethod
from typing import Dict, List

from sova.backend.backend import Backend, BackendOutput, Metric

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
        Number of iterations that will be produced by chosen MROB backend
    robust_type: int
        Represents type of robust optimisations
    """

    def __init__(
        self, poses_number: int, iterations_number: int, robust_type: int = mrob.HUBER
    ) -> None:
        self._graph: mrob.FGraph = mrob.FGraph(robust_type)
        self._poses_number: int = poses_number
        self._iterations_number: int = iterations_number
        self._planes: Dict[int, int] = {}

    def _init_poses(self):
        """
        Initializes pose nodes in mrob graph
        """
        self._graph.add_node_pose_3d(mrob.geometry.SE3(), mrob.NODE_ANCHOR)
        for _ in range(self._poses_number - 1):
            self._graph.add_node_pose_3d(mrob.geometry.SE3(), mrob.NODE_STANDARD)

    @abstractmethod
    def _init_point_clouds(self, grid: GridBase) -> None:
        """
        Represents abstract method for initiating plane features

        Parameters
        ----------
        grid: GridBase
            Represents grid with all inserted point clouds
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
        metrics = [Metric(name="FGraph initial error", value=self._graph.chi2(True))]
        converge_iterations = self._graph.solve(mrob.LM_ELLIPS, self._iterations_number)
        while converge_iterations == 0:
            print("Optimization didn't converge")
            converge_iterations = self._graph.solve(
                mrob.LM_ELLIPS, self._iterations_number
            )

        metrics.append(Metric(name="Iterations to converge", value=converge_iterations))
        metrics.append(Metric(name="chi2", value=self._graph.chi2()))

        return BackendOutput(
            self._graph.get_estimated_state(), metrics, self.__get_unused_features()
        )

    def __get_unused_features(self) -> List[int]:
        """
        Returns list of features which were unused on optimisation stage

        Returns
        -------
        unused_features: List[int]
            IDs list of unused features
        """
        try:
            robust_mask = self._graph.get_eigen_factors_robust_mask()
        except AttributeError:
            print("[WARNING] Most likely you are not using robust optimisations")
            return []

        unused_features = []

        for voxel_id, plane_id in self._planes.items():
            if robust_mask[plane_id]:
                unused_features.append(voxel_id)

        return unused_features
