
add_library(SE3 INTERFACE)
target_sources(
  SE3
  INTERFACE FILE_SET
            HEADERS
            BASE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}
            FILES
            "SE3.hpp"
            "Util.hpp")

target_link_libraries(SE3 INTERFACE Eigen3::Eigen)
