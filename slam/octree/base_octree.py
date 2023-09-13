from abc import ABC, abstractmethod

__all__ = ["Octree"]


class Octree(ABC):
    """
    Represents abstract class for octree struct
    """

    @abstractmethod
    def segment(self) -> None:
        """
        Represents abstract method to segment planes from given on early stages point cloud
        """
        pass

    @abstractmethod
    def visualize(self) -> None:
        """
        Represents abstract method to visualize segmented point cloud.
        """
        pass
