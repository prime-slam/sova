from abc import abstractmethod

__all__ = ["Metric"]


class Metric:
    """
    Represents metric class, which was created to produce pretty print
    and kind of structural output of pipeline result.
    """

    @abstractmethod
    def __init__(self, name: str, value: float) -> None:
        self._name = name
        self._value = value

    @property
    def name(self) -> str:
        """
        Represents method to get name of metric

        Returns
        -------
        name: str
            Metric's name
        """
        return self._name

    @property
    def value(self) -> float:
        """
        Represents method to get value of metric

        Returns
        -------
        value: float
            Metric's value
        """
        return self._value
