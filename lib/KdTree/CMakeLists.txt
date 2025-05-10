





add_library(KdTree INTERFACE)
target_sources(KdTree INTERFACE
    FILE_SET HEADERS 
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES "KdTree.h")

target_link_libraries(KdTree INTERFACE nanoflann::nanoflann)
target_link_libraries(KdTree INTERFACE Eigen3::Eigen)