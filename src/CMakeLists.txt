# MazeLib/src/CMakeLists.txt
include_directories(../include)
file(GLOB SRCS *.cpp)
add_library(mazelib STATIC ${SRCS})
