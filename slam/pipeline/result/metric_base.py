from abc import abstractmethod

__all__ = ["Metric"]


class Metric:
    """
    Represents metric class, which was created to produce pretty print
    and kind of structural output of pipeline result.
    """

    @abstractmethod
    def __init__(self, name: str, value: str) -> None:
        self.name = name
        self.value = value

    @property
    def name(self) -> str:
        """
        Represents method to get name of metric

        Returns
        -------
        name: str
            Metric's name
        """
        return self.name

    @property
    def value(self) -> float:
        """
        Represents method to get value of metric

        Returns
        -------
        value: float
            Metric's value
        """
        return self.value
