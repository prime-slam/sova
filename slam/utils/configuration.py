import copy

import mrob
import yaml

from typing import List

from octreelib.grid import GridConfig

from slam.backend import Backend, EigenFactorBackend, BaregBackend
from slam.filter import Filter
from slam.segmenter import (
    Segmenter,
    CAPESegmenter,
    CountSegmenter,
    IdenticalSegmenter,
    RansacSegmenter,
)
from slam.subdivider import (
    Subdivider,
    SizeSubdivider,
    CountSubdivider,
    EigenValueSubdivider,
)

__all__ = ["Configuration"]


class Configuration:
    """
    Represents configuration which is read from yaml file

    Parameters
    ----------
    filepath: str
        Path to yaml configuration of pipeline
    """

    def __init__(self, filepath: str) -> None:
        with open(filepath) as file:
            self._configuration = yaml.safe_load(file)

    @property
    def debug(self) -> bool:
        """
        Represents debug parameter of pipeline

        Returns
        -------
        debug: bool
            Debug parameter
        """
        try:
            value = copy.deepcopy(self._configuration["debug"])
        except KeyError:
            return False

        return bool(value)

    @property
    def dataset_path(self) -> str:
        """
        Represents dataset path parameter of pipeline

        Returns
        -------
        dataset_path: str
            Dataset path
        """
        try:
            dataset_configuration = copy.deepcopy(self._configuration["dataset"])
            value = dataset_configuration["path"]
        except KeyError as e:
            raise ValueError(f"{e} must be set")

        return value

    @property
    def patches_start(self) -> int:
        """
        Represents patches start parameter of pipeline

        Returns
        -------
        patches_start: int
            Patches start
        """
        try:
            dataset_configuration = copy.deepcopy(self._configuration["dataset"])
            patches_configuration = dataset_configuration["patches"]
            value = patches_configuration["start"]
        except KeyError as e:
            raise ValueError(f"{e} must be set")

        return int(value)

    @property
    def patches_end(self) -> int:
        """
        Represents patches end parameter of pipeline

        Returns
        -------
        patches_end: int
            Patches end
        """
        try:
            dataset_configuration = copy.deepcopy(self._configuration["dataset"])
            patches_configuration = dataset_configuration["patches"]
            value = patches_configuration["end"]
        except KeyError as e:
            raise ValueError(f"{e} must be set")

        return int(value)

    @property
    def patches_step(self) -> int:
        """
        Represents patches step parameter of pipeline

        Returns
        -------
        patches_step: int
            Patches step
        """
        try:
            dataset_configuration = copy.deepcopy(self._configuration["dataset"])
            patches_configuration = dataset_configuration["patches"]
            value = patches_configuration["step"]
        except KeyError as e:
            raise ValueError(f"{e} must be set")

        return int(value)

    @property
    def patches_iterations(self) -> int:
        """
        Represents patches iterations parameter of pipeline

        Returns
        -------
        patches_iterations: int
            Patches iterations
        """
        try:
            dataset_configuration = copy.deepcopy(self._configuration["dataset"])
            patches_configuration = dataset_configuration["patches"]
            value = patches_configuration["iterations"]
        except KeyError:
            return 1

        return int(value)

    @property
    def visualization_dir(self) -> str:
        """
        Represents visualization directory parameter of pipeline

        Returns
        -------
        visualization_dir: str
            Path to visualisation directory to save
        """
        try:
            output_configuration = copy.deepcopy(self._configuration["output"])
            value = output_configuration["visualization_path"]
        except KeyError:
            return "output/visualization"

        return value

    @property
    def optimisation_dir(self) -> str:
        """
        Represents optimisation directory parameter of pipeline

        Returns
        -------
        optimisation_dir: str
            Path to optimisation poses directory to save
        """
        try:
            output_configuration = copy.deepcopy(self._configuration["output"])
            value = output_configuration["optimisation_path"]
        except KeyError:
            return "output/optimisation"

        return value

    @property
    def subdividers(self) -> List[Subdivider]:
        """
        Represents subdividers parameter of pipeline

        Returns
        -------
        subdividers: List[Subdivider]
            Subdividers list
        """
        try:
            subdividers_configuration = copy.deepcopy(
                self._configuration["subdividers"]
            )
        except KeyError:
            raise ValueError("subdividers must be not empty")

        subdividers_names = {
            "count": CountSubdivider,
            "eigen_value": EigenValueSubdivider,
            "size": SizeSubdivider,
        }
        subdividers = []

        for name in subdividers_configuration.keys():
            name = name.lower()
            subdividers.append(
                subdividers_names[name](subdividers_configuration[name]),
            )

        return subdividers

    @property
    def filters(self) -> List[Filter]:
        """
        Represents filters parameter of pipeline

        Returns
        -------
        filters: List[Filter]
            Filters list
        """
        return []

    @property
    def segmenters(self) -> List[Segmenter]:
        """
        Represents segmenters parameter of pipeline

        Returns
        -------
        segmenters: List[Segmenter]
            Segmenters list
        """
        try:
            segmenters_configuration = copy.deepcopy(self._configuration["segmenters"])
        except KeyError:
            raise ValueError("segmenters must be not empty")

        segmenters_names = {
            "cape": CAPESegmenter,
            "count": CountSegmenter,
            "identical": IdenticalSegmenter,
            "ransac": RansacSegmenter,
        }
        segmenters = []

        for name in segmenters_configuration.keys():
            name = name.lower()
            values = segmenters_configuration[name]
            segmenters.append(segmenters_names[name](**values))

        return segmenters

    @property
    def grid_configuration(self) -> GridConfig:
        """
        Represents grid configuration parameter of pipeline

        Returns
        -------
        configuration: GridConfig
            Grid configuration
        """
        try:
            grid_configuration = copy.deepcopy(self._configuration["grid"])
        except KeyError:
            raise ValueError("grid_configuration must be not empty")

        return GridConfig(voxel_edge_length=grid_configuration["voxel_edge_length"])

    def backend(self, start: int, end: int) -> Backend:
        """
        Represents backend parameter of pipeline

        Parameters
        ----------
        start: int
            Represents start of patch
        end: int
            Represents end of patch

        Returns
        -------
        backend: Backend
            Backend of pipeline
        """
        try:
            backend_configuration = copy.deepcopy(self._configuration["backend"])
        except KeyError:
            raise ValueError("backend_configuration must be not empty")

        backend_names = {
            "eigen_factor": EigenFactorBackend,
            "bareg": BaregBackend,
        }
        robust_types = {"huber": mrob.HUBER, "quadratic": mrob.QUADRATIC}

        try:
            backend_type = backend_configuration["type"]
            backend_parameters = backend_configuration["parameters"]
            backend_parameters["poses_number"] = end - start
            try:
                backend_parameters["robust_type"] = robust_types[
                    backend_parameters["robust_type"].lower()
                ]
            except KeyError as e:
                if self.debug:
                    print(e)
        except KeyError:
            raise ValueError("backend type/parameters must be not empty")

        return backend_names[backend_type](**backend_parameters)
