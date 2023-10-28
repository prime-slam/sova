from abc import ABC

from octreelib.grid import GridBase

from slam.backend import Backend, BackendOutput


class MROBBackend(Backend, ABC):
    def process(self, grid: GridBase) -> BackendOutput:
        pass

    @absractmetho