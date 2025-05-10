
# add_library(KdTree OBJECT 
#     KDTreeVectorOfVectorsAdaptor.h
# )
# target_link_libraries(KdTree nanoflann::nanoflann)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")


add_executable(Test test.cpp)

# target_include_directories(Test PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(Test Open3D::Open3D)
target_link_libraries(Test Eigen3::Eigen)
target_link_libraries(Test Icp)



