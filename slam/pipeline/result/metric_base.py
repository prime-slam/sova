from abc import ABC, abstractmethod

__all__ = ["Metric"]


class Metric(ABC):
    """
    Represents metric abstract class, which was created to produce pretty print
    and kind of structural output
    """
    @property
    @abstractmethod
    def name(self) -> str:
        """
        Represents abstract method to get name of metric

        Returns
        -------
        name: str
            Metric's name
        """
        pass

    @property
    @abstractmethod
    def value(self) -> float:
        """
        Represents abstract method to get value of metric

        Returns
        -------
        value: float
            Metric's value
        """
        pass
