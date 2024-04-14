<img src="assets/logo.png">
SOVA is a SLAM on Voxel Alignment and an open-source Python library, designed for fast 
and adaptive comparison of different approaches to solving the voxel-based planar SLAM problem.

Our main goal is to provide extendable, simple and efficient interfaces for
testing various voxel SLAM hypotheses, which include different subdivision/segmenter/backend criteria.

[![Tests](https://github.com/prime-slam/sova/actions/workflows/tests.yaml/badge.svg)](https://github.com/prime-slam/sova/actions/workflows/tests.yaml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

- - -
# Installation

To use this library you need to:

1. Download and install Python 3.10 from the [official website](https://www.python.org/downloads/).
2. Install pip package:
   ```bash
   pip install sova
   ```
   If you want to use `MROBBackend` robust optimisations, you have to install mrob library manually:
   1. Download [wheel from source](https://drive.google.com/file/d/1rUdbybNvHx80ykr62aceAcBIPtlntWIz/view?usp=sharing)
   2. Install mrob from wheels
   ```bash
   python -m pip install mrob --no-index --find-links wheel/ --force-reinstall
   ```

Now you have everything you need to run your voxel-based pipeline.

# Examples

Examples of using the voxel-based pipeline are presented in the [`examples`](https://github.com/prime-slam/sova/tree/main/examples) 
directory with the all necessary instructions of how to run them.

# Contributing

To contribute to the project you must:
1. Get to know the project structure:
    ```
    sova
    ├── backend
    ├── filter
    ├── pipeline
    ├── segmenter
    ├── subdivider
    ├── typing
    └── utils
        ├── dataset_reader
    ```
2. Implement new subdivision/segmenter/backend approach which satisfy the relevant interface.
3. Create PullRequest to the repository.
4. Go through the review and wait for your code to appear in the main branch.
