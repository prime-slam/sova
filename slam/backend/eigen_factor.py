import mrob
from octreelib.grid import GridBase

from slam.backend.backend import Backend, BackendOutput, Metric

__all__ = ["EigenFactorBackend"]


class EigenFactorBackend(Backend):
    """
    Represents mrob backend implementation

    Parameters
    ----------
    poses_number: int
        Number of poses in provided map
    iterations_number: int
        Number of iterations that will be produced by MROB-EF backend.
    """

    def __init__(self, poses_number: int, iterations_number: int) -> None:
        self.__graph: mrob.FGraph = mrob.FGraph()
        self.__poses_number: int = poses_number
        self.__iterations_number: int = iterations_number

    def process(self, grid: GridBase) -> BackendOutput:
        """
        Represents implementation of Backend abstract class, which
        takes remaining points from poses (Octree, essentially) and optimises by them.

        Parameters
        ----------
        grid: GridBase
            Represents grid with all inserted point clouds

        Returns
        -------
        output: BackendOutput
            Result of backend optimisations
        """
        self.__init_poses()
        self.__init_point_clouds(grid)
        metrics = [
            Metric(name="FGraph initial error", value=self.__graph.chi2(True)),
        ]
        self.__graph.solve(mrob.LM_ELLIPS, self.__iterations_number)

        metrics.append(
            Metric(name="chi2", value=self.__graph.chi2()),
        )

        return BackendOutput(self.__graph.get_estimated_state(), metrics)

    def __init_poses(self) -> None:
        """
        Inits poses points if EigenFactor graph.
        """
        pose_id = self.__graph.add_node_pose_3d(mrob.geometry.SE3(), mrob.NODE_ANCHOR)
        for _ in range(self.__poses_number - 1):
            pose_id = self.__graph.add_node_pose_3d(
                mrob.geometry.SE3(), mrob.NODE_STANDARD
            )

        return pose_id

    def __init_point_clouds(self, grid: GridBase) -> None:
        """
        Inits feature (planes) point clouds in EigenFactor graph.
        """
        for pose_number in range(self.__poses_number):
            self.__graph.add_eigen_factor_plane()
            self.__graph.eigen_factor_plane_add_points_array(
                planeEigenId=pose_number,
                nodePoseId=pose_number,
                pointsArray=grid.get_points(pose_number),
                W=1.0,
            )
