clang-format -i main/*.cpp

clang-format -i lib/*/*.cpp
clang-format -i lib/*/*.hpp

cmake-format ./*/CMakeLists.txt -i
