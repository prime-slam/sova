from abc import abstractmethod
from typing import Callable

from slam.typing import ArrayNx3

SubdividerFunc = Callable[[ArrayNx3[float]], bool]


class Subdivider:
    @abstractmethod
    def create_func(self) -> SubdividerFunc:
        pass
