# MazeLib/src/CMakeLists.txt
include_directories(../include)
file(GLOB MAZELIB_SRCS *.cpp)
add_library(mazelib STATIC ${MAZELIB_SRCS})
