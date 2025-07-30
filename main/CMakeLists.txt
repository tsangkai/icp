add_executable(run_icp)
target_sources(run_icp PRIVATE run_icp.cpp)

target_link_libraries(run_icp Open3D::Open3D)
target_link_libraries(run_icp Eigen3::Eigen)
target_link_libraries(run_icp Icp)
target_link_libraries(run_icp cxxopts)
