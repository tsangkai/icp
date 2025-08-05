

add_library(SO3 INTERFACE)
target_sources(
  SO3
  INTERFACE FILE_SET
            HEADERS
            BASE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}
            FILES
            "SO3.hpp"
            "SO3.ipp"
            "Util.hpp")
target_link_libraries(SO3 INTERFACE Eigen3::Eigen)

add_library(SE3 INTERFACE)
target_sources(
  SE3
  INTERFACE FILE_SET
            HEADERS
            BASE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}
            FILES
            "SE3.hpp"
            "SE3.ipp"
            "Util.hpp")

target_link_libraries(SE3 INTERFACE Eigen3::Eigen)
target_link_libraries(SE3 INTERFACE SO3)

add_subdirectory(test)
