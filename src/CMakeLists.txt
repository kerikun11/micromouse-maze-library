# MazeLib/src/CMakeLists.txt
include_directories(../inc)
file(GLOB MAZELIB_SRCS *.cpp)
add_library(mazelib STATIC ${MAZELIB_SRCS})
