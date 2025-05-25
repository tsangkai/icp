add_executable(Test)
target_sources(Test PRIVATE test.cpp)

target_link_libraries(Test Open3D::Open3D)
target_link_libraries(Test Eigen3::Eigen)
target_link_libraries(Test Icp)
target_link_libraries(Test cxxopts)



add_executable(TestTimestamp)
target_sources(TestTimestamp PRIVATE test_timestamp.cpp)

target_link_libraries(TestTimestamp Open3D::Open3D)
target_link_libraries(TestTimestamp Eigen3::Eigen)
target_link_libraries(TestTimestamp Icp)
target_link_libraries(TestTimestamp cxxopts)
target_link_libraries(TestTimestamp absl::log)