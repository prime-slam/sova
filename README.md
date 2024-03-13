# Running `octreelib` with CUDA RANSAC

1. Clone the repository and install the requirements
    ```bash
    git clone https://github.com/true-real-michael/voxel-slam.git
    cd voxel-slam
    python3 -m venv venv && source venv/bin/activate
    pip install -r requirements.txt
    ```
2. Install `octreelib` with CUDA RANSAC support with processing in batches
    ```bash
    pip install git+https://github.com/prime-slam/octreelib@cuda-ransac-processing-in-batches --force-reinstall
    ```
3. Run the example
    ```bash
    cd examples
    python pipeline_with_cuda.py
    ```




- - -
