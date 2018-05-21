# MazeLib/src/CMakeLists.txt
cmake_minimum_required(VERSION 3.1)

include_directories(../inc)
file(GLOB MAZELIB_SRCS *.cpp)
add_library(mazelib STATIC ${MAZELIB_SRCS})
