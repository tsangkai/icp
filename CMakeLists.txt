cmake_minimum_required(VERSION 3.28.0)
set (CMAKE_CXX_STANDARD 17)

project(ICP_TEST LANGUAGES C CXX)


find_package(Open3D)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nanoflann)

add_subdirectory(main)
add_subdirectory(lib)



