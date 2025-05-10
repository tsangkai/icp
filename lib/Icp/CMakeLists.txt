


add_library(Icp INTERFACE)
target_sources(Icp INTERFACE
    FILE_SET HEADERS 
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES "Icp.h")

target_link_libraries(Icp INTERFACE KdTree)
target_link_libraries(Icp INTERFACE Type)
