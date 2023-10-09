from slam.octree import Octree
from slam.pipeline.backend.backend_base import Backend
from slam.pipeline.result.pipeline_result import PipelineResult

__all__ = ["HkuMarsBackend"]


class HkuMarsBackend(Backend):
    """
    Represents Hku-Mars backend implementation
    """

    def __init__(self) -> None:
        pass

    def process(self, octree: Octree) -> PipelineResult:
        """
        Will be created in the future...
        """
        pass
