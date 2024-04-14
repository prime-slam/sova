from octreelib.grid import GridBase

from sova.backend.mrob_backend import MROBBackend

__all__ = ["BaregBackend"]


class BaregBackend(MROBBackend):
    def _init_point_clouds(self, grid: GridBase) -> None:
        """
        Initializes plane features in graph using bareg backend

        Parameters
        ----------
        grid: GridBase
            Represents grid with all inserted point clouds
        """
        for pose_number in range(self._poses_number):
            leaf_voxels = grid.get_leaf_points(
                pose_number=pose_number,
            )
            for voxel in leaf_voxels:
                if voxel.id not in self._planes.keys():
                    bareg_plane_id = self._graph.add_bareg_plane()
                    self._planes[voxel.id] = bareg_plane_id

                self._graph.eigen_factor_plane_add_points_array(
                    planeEigenId=self._planes[voxel.id],
                    nodePoseId=pose_number,
                    pointsArray=voxel.get_points(),
                    W=1.0,
                )
