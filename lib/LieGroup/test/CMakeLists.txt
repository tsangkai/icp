




enable_testing()

add_executable(
  TestSE3
  TestSE3.cpp
)

target_link_libraries(TestSE3 GTest::gtest_main)
target_link_libraries(TestSE3 SE3)


include(GoogleTest)
gtest_discover_tests(TestSE3)

