





- preprocess the point cloud (calculate the normals, remove the unwanted points)
- `open3d::geometry::PointCloud::DetectPlanarPatches()`


```
   cmake ..
   cmake --build .
   ./main/Test  -d /home/tsangkai/dataset/kitti/2011_10_03/2011_10_03_drive_0027_sync/
```

## Log



#### 2025.05.02
 - Using `open3d::pipelines::registration::RegistrationICP()` for benchmarking.
#### 2025.04.29
 - `Open3d` already has built in kdtree.


### Reference

download kitti dataset: https://github.com/Deepak3994/Kitti-Dataset/tree/master

read data reference: https://github.com/utiasSTARS/pykitti/tree/master


#### Cmake

https://iamsorush.com/posts/cpp-cmake-build/



#### Bazel

https://blog.engflow.com/2024/06/27/migrating-to-bazel-modules-aka-bzlmod---the-easy-parts/

