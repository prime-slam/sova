from dataclasses import dataclass

__all__ = ["DepthOctreeConfig"]


@dataclass
class DepthOctreeConfig:
    """
    Represents config struct for DepthOctree

    Parameters
    ----------
    depth: int
        Depth of octree
    """

    depth: int = 1
