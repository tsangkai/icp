
set(PRIVATE_SOURCE_FILES 
    "IcpCommon.hpp"
    "IcpRunner.hpp"
    "PointToPlaneIcp.hpp"
    "PointToPlaneIcp.cpp"
)

add_library(Icp SHARED)
target_sources(Icp
    PRIVATE ${PRIVATE_SOURCE_FILES}
    PUBLIC FILE_SET HEADERS 
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES Icp.hpp)

target_link_libraries(Icp Eigen3::Eigen)
target_link_libraries(Icp KdTree)
target_link_libraries(Icp Type)
target_link_libraries(Icp manif)
