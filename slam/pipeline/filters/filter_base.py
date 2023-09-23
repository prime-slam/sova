from abc import ABC, abstractmethod
from typing import Callable

from slam.typing import ArrayNx3


FilterFunc = Callable[[ArrayNx3[float]], bool]


class Filter(ABC):
    @abstractmethod
    def create_func(self) -> FilterFunc:
        pass
