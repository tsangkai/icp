set(PRIVATE_SOURCE_FILES "PointCloud.cpp" "PointCloud.hpp" "Transformation.hpp"
                         "EstimateNormals.cpp")

add_library(Type SHARED)
target_sources(
  Type
  PRIVATE ${PRIVATE_SOURCE_FILES}
  PUBLIC FILE_SET HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} FILES Type.hpp)

target_link_libraries(Type Open3D::Open3D)
target_link_libraries(Type Eigen3::Eigen)
target_link_libraries(Type KdTree)

target_link_libraries(Type OpenMP::OpenMP_CXX)
