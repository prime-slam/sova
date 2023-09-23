from slam.pipeline.subdividers.subdivider_base import Subdivider, SubdividerFunc
from slam.typing import ArrayNx3


class CountSubdivider(Subdivider):
    def __init__(self, count: int) -> None:
        self.count = count

    def create_func(self) -> SubdividerFunc:
        def f(points: ArrayNx3[float]) -> bool:
            return len(points) <= self.count

        return f
