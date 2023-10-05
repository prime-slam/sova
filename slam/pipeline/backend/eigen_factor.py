import mrob

from slam.octree import Octree
from slam.pipeline.backend.backend_base import Backend
from slam.pipeline.result.pipeline_result import PipelineResult

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

    def process(self, octree: Octree) -> PipelineResult:
        """
        Represents implementation of Backend abstract class, which
        takes remaining points from poses (Octree, essentially) and optimises by them.

        Parameters
        ----------
        octree: Octree
            Octree which contains all inserted point clouds

        Returns
        -------
        result: PipelineResult
            Structural output of pipeline backend
        """
        self.__init_poses()
        self.__init_point_clouds(octree)
        self.__graph.solve(mrob.LM_ELLIPS, self.__iterations_number)

        return PipelineResult(
            self.__graph.get_estimated_state(),
            self.__graph.chi2()
        )

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

    def __init_point_clouds(self, octree: Octree) -> None:
        """
        Inits feature (planes) point clouds in EigenFactor graph.
        """
        for pose_number in range(self.__poses_number):
            self.__graph.add_eigen_factor_plane()
            self.__graph.eigen_factor_plane_add_points_array(
                planeEigenId=pose_number,
                nodePoseId=pose_number,
                pointsArray=octree.get_points(pose_number),
                W=1.0,
            )
