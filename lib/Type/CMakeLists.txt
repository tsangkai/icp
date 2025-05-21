
add_library(Type INTERFACE)
target_sources(Type INTERFACE
    FILE_SET HEADERS 
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES "Type.h")

target_link_libraries(Type INTERFACE Open3D::Open3D)
target_link_libraries(Type INTERFACE Eigen3::Eigen)


target_link_libraries(Type INTERFACE OpenMP::OpenMP_CXX)
