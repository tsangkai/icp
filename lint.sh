clang-format -i main/*.cpp

clang-format -i lib/*/*.cpp
clang-format -i lib/*/*.hpp
clang-format -i lib/*/*.ipp

cmake-format ./*/CMakeLists.txt -i
