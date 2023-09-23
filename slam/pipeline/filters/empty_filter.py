from slam.pipeline.filters.filter_base import Filter, FilterFunc
from slam.typing import ArrayNx3


class EmptyFilter(Filter):
    def create_func(self) -> FilterFunc:
        def f(points: ArrayNx3[float]) -> bool:
            return len(points) != 0

        return f
