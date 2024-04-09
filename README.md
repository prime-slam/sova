# Running `octreelib` with CUDA RANSAC

1. Clone the repository and install the requirements
    ```bash
    git clone https://github.com/true-real-michael/voxel-slam.git
    cd voxel-slam
    python3 -m venv venv && source venv/bin/activate
    pip install -r requirements.txt
    ```
2.
    1. Run the example to get visualization file
       ```bash
       cd examples
       python pipeline_with_cuda.py
       ```
    2. Run the example to benchmark the performance
       ```bash
       cd examples
       python benchmark.py
       ```