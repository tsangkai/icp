# ICP Implementation

This repo implements the ICP algorithm, and tests on the KITTI dataset.

The project focuses on deriving the Jacobian using the Lie theory framework, and also tests on the existing Lie group library, [`manif`](https://github.com/artivis/manif) for example.

## Usage

To download the KITTI dataset, one can directly run `download_kitti.sh`.

To build this repo,
```shell
   mkdir build
   cd build
   cmake ..
   cmake --build .
```

To run the ICP algorithm on the KITTI dataset,
```shell
   ./main/run_icp  -d [KITTI dataset folder]
```
