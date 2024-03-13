# Running `octreelib` with CUDA RANSAC

1. Clone the repository and install the requirements
    ```bash
    git clone https://github.com/true-real-michael/voxel-slam.git
    cd voxel-slam
    python3 -m venv venv && source venv/bin/activate
    pip install -r requirements.txt
    ```
2. Install `octreelib` with CUDA RANSAC support with or without **processing in batches**
    ```bash
    # with processing in batches
    pip install git+https://github.com/prime-slam/octreelib@cuda-ransac-processing-in-batches
    # without processing in batches
    pip install git+https://github.com/prime-slam/octreelib@cuda-ransac
    ```
3. Run the example
    ```bash
    python examples/pipeline_with_cuda.py
    ```




- - -
