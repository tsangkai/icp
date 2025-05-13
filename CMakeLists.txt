cmake_minimum_required(VERSION 3.28.0)
set (CMAKE_CXX_STANDARD 23)

project(ICP_TEST LANGUAGES C CXX)


find_package(Open3D)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nanoflann)


include(FetchContent)
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
  GIT_TAG 4bf61f08697b110d9e3991864650a405b3dd515d # v3.2.1
)

# The external is downloaded by this:
FetchContent_MakeAvailable(cxxopts)


add_subdirectory(main)
add_subdirectory(lib)
