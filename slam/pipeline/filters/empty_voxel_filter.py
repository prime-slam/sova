from slam.pipeline.filters.filter_base import Filter, FilterFunc
from slam.typing import ArrayNx3

__all__ = ["EmptyVoxelFilter"]


class EmptyVoxelFilter(Filter):
    """
    Represents "empty voxel" filter: if voxel contains no points, it should be deleted
    """
    def create_func(self) -> FilterFunc:
        """
        Represents implementation of Filter abstract class

        Returns
        -------
        func: FilterFunc
            Function that contains "zero points" filter condition
        """
        def f(points: ArrayNx3[float]) -> bool:
            return len(points) != 0

        return f
