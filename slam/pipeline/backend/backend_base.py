from abc import ABC, abstractmethod

from slam.octree import Octree
from slam.pipeline.result.pipeline_result import PipelineResult


class Backend(ABC):
    @abstractmethod
    def process(self, octree: Octree) -> PipelineResult:
        pass
