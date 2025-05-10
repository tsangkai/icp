add_executable(Test)
target_sources(Test PRIVATE test.cpp)

target_link_libraries(Test Open3D::Open3D)
target_link_libraries(Test Eigen3::Eigen)
target_link_libraries(Test Icp)