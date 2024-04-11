import yaml

from sova.pipeline.configuration.reader import ConfigurationReader

__all__ = ["YAMLConfigurationReader"]


class YAMLConfigurationReader(ConfigurationReader):
    """
    Represents yaml configuration reader
    """

    def __init__(self, filepath: str) -> None:
        with open(filepath) as file:
            self._configuration = yaml.safe_load(file)
