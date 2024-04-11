import mrob
from octreelib.grid import GridConfig

import copy
from abc import ABC, abstractmethod
from typing import List

from sova.backend import Backend, BaregBackend, EigenFactorBackend
from sova.filter import Filter
from sova.segmenter import (
    CAPESegmenter,
    CountSegmenter,
    IdenticalSegmenter,
    RansacSegmenter,
    Segmenter,
)
from sova.subdivider import (
    CountSubdivider,
    EigenValueSubdivider,
    SizeSubdivider,
    Subdivider,
)

__all__ = ["ConfigurationReader"]


class ConfigurationReader(ABC):
    """
    Represents abstract class for pipeline configuration reader

    Parameters
    ----------
    filepath: str
        Path to file configuration of pipeline

    Attributes
    ----------
    configuration: Dict[str, Optional[parameters]]
        Represents configuration dictionary
    """

    _configuration = {}

    @abstractmethod
    def __init__(self, filepath: str) -> None:
        pass

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
    def dataset_type(self) -> str:
        """
        Represents dataset type for pose and clouds reading

        Returns
        -------
        dataset_type: str
            Dataset type
        """
        try:
            dataset_configuration = copy.deepcopy(self._configuration["dataset"])
            value = dataset_configuration["type"]
        except KeyError as e:
            raise ValueError(f"{e} must be set")

        return value

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
    def output_directory(self) -> str:
        """
        Represents output directory which contains all products of voxel-based pipeline

        Returns
        -------
        output_directory: str
            Path to output directory
        """
        try:
            pipeline_configuration = copy.deepcopy(self._configuration["pipeline"])
            value = pipeline_configuration["output"]
        except KeyError:
            return "output"

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
            pipeline_configuration = copy.deepcopy(self._configuration["pipeline"])
            subdividers_configuration = pipeline_configuration["subdividers"]
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
            subdividers.append(subdividers_names[name](subdividers_configuration[name]))

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
            pipeline_configuration = copy.deepcopy(self._configuration["pipeline"])
            segmenters_configuration = pipeline_configuration["segmenters"]
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
            pipeline_configuration = copy.deepcopy(self._configuration["pipeline"])
            grid_configuration = pipeline_configuration["grid"]
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
            pipeline_configuration = copy.deepcopy(self._configuration["pipeline"])
            backend_configuration = pipeline_configuration["backend"]
        except KeyError:
            raise ValueError("backend_configuration must be not empty")

        backend_names = {"eigen_factor": EigenFactorBackend, "bareg": BaregBackend}
        robust_types = {
            "huber": mrob.HUBER,
            "quadratic": mrob.QUADRATIC,
            "cauchy": mrob.CAUCHY,
            "mcclure": mrob.MCCLURE,
            "ransac": mrob.RANSAC,
        }

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
