from octreelib.grid import GridBase

from sova.backend.mrob_backend import MROBBackend

__all__ = ["EigenFactorBackend"]


class EigenFactorBackend(MROBBackend):
    def _init_point_clouds(self, grid: GridBase) -> None:
        """
        Initializes plane features using eigen factor backend

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
                    factor_plane_id = self._graph.add_eigen_factor_plane_center()
                    self._planes[voxel.id] = factor_plane_id

                self._graph.eigen_factor_plane_add_points_array(
                    planeEigenId=self._planes[voxel.id],
                    nodePoseId=pose_number,
                    pointsArray=voxel.get_points(),
                    W=1.0,
                )
