


add_library(Type OBJECT 
    Type.hpp
)
target_link_libraries(Type Open3D::Open3D)
target_link_libraries(Type Eigen3::Eigen)



add_library(KdTree OBJECT 
    KdTree.hpp
)
target_link_libraries(KdTree nanoflann::nanoflann)
target_link_libraries(KdTree Eigen3::Eigen)


add_library(Icp OBJECT 
    Icp.hpp
)
