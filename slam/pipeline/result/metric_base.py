from abc import ABC, abstractmethod


class Metric(ABC):
    """

    """
    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @property
    @abstractmethod
    def value(self) -> float:
        pass
