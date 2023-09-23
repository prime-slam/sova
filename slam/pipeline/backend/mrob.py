from slam.octree import Octree
from slam.pipeline.backend.backend_base import Backend
from slam.pipeline.result.pipeline_result import PipelineResult


class MrobBackend(Backend):
    def __init__(self) -> None:
        pass

    def process(self, octree: Octree) -> PipelineResult:
        return PipelineResult()
