import mrob
import pytest
from octreelib.grid import GridConfig

from typing import List

from sova.backend import Backend, EigenFactorBackend
from sova.filter import Filter
from sova.pipeline import YAMLConfigurationReader
from sova.segmenter import RansacSegmenter, Segmenter
from sova.subdivider import SizeSubdivider, Subdivider


@pytest.mark.parametrize(
    "yaml_configuration_path, "
    "debug, "
    "dataset_type, "
    "dataset_path, "
    "patches_start, "
    "patches_end, "
    "patches_step, "
    "patches_iterations, "
    "output_directory, "
    "subdividers, "
    "filters, "
    "segmenters, "
    "grid_configuration, "
    "backend",
    [
        (
            "tests/data/yaml_configurations/correct_configuration.yaml",
            False,
            "dataset_type",
            "path/to/dataset",
            0,
            100,
            10,
            1,
            "output",
            [SizeSubdivider(size=2)],
            [],
            [RansacSegmenter(0.01, 6, 5000)],
            GridConfig(voxel_edge_length=8),
            EigenFactorBackend(
                poses_number=10, iterations_number=5000, robust_type=mrob.HUBER
            ),
        ),
    ],
)
def test_correct_configuration(
    yaml_configuration_path: str,
    debug: bool,
    dataset_type: bool,
    dataset_path: str,
    patches_start: int,
    patches_end: int,
    patches_step: int,
    patches_iterations: int,
    output_directory: str,
    subdividers: List[Subdivider],
    filters: List[Filter],
    segmenters: List[Segmenter],
    grid_configuration: GridConfig,
    backend: Backend,
):
    yaml_reader = YAMLConfigurationReader(yaml_configuration_path)

    assert debug == yaml_reader.debug
    assert dataset_path == yaml_reader.dataset_path
    assert patches_start == yaml_reader.patches_start
    assert patches_end == yaml_reader.patches_end
    assert patches_step == yaml_reader.patches_step
    assert patches_iterations == yaml_reader.patches_iterations
    assert output_directory == yaml_reader.output_directory
    assert len(subdividers) == len(yaml_reader.subdividers)
    assert len(filters) == len(yaml_reader.filters)
    assert len(segmenters) == len(yaml_reader.segmenters)
    assert (
        grid_configuration.voxel_edge_length
        == yaml_reader.grid_configuration.voxel_edge_length
    )
    actual_backend = yaml_reader.backend(0, 10)
    for field in ["_poses_number", "_iterations_number"]:
        assert backend.__dict__[field] == actual_backend.__dict__[field]


@pytest.mark.parametrize(
    "yaml_configuration_path, missed_field",
    [
        (
            "tests/data/yaml_configurations/incorrect_configuration_backend.yaml",
            "backend_configuration",
        ),
        (
            "tests/data/yaml_configurations/incorrect_configuration_dataset.yaml",
            "dataset",
        ),
        (
            "tests/data/yaml_configurations/incorrect_configuration_patches.yaml",
            "patches",
        ),
        (
            "tests/data/yaml_configurations/incorrect_configuration_segmenters.yaml",
            "segmenters",
        ),
        (
            "tests/data/yaml_configurations/incorrect_configuration_subdividers.yaml",
            "subdividers",
        ),
        (
            "tests/data/yaml_configurations/incorrect_configuration_dataset_type.yaml",
            "type",
        ),
    ],
)
def test_incorrect_yaml_configuration_reader(
    yaml_configuration_path: str,
    missed_field: str,
):
    yaml_reader = YAMLConfigurationReader(yaml_configuration_path)

    with pytest.raises(ValueError) as excinfo:
        _ = yaml_reader.debug
        _ = yaml_reader.dataset_type
        _ = yaml_reader.dataset_path
        _ = yaml_reader.patches_start
        _ = yaml_reader.patches_end
        _ = yaml_reader.patches_step
        _ = yaml_reader.patches_iterations
        _ = yaml_reader.output_directory
        _ = yaml_reader.subdividers
        _ = yaml_reader.filters
        _ = yaml_reader.segmenters
        _ = yaml_reader.grid_configuration
        _ = yaml_reader.backend(0, 1)

    assert (
        str(excinfo.value) == f"'{missed_field}' must be set"
        or str(excinfo.value) == f"{missed_field} must be not empty"
    )
