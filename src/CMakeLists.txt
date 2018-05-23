# MazeLib/src/CMakeLists.txt
cmake_minimum_required(VERSION 3.1)

include_directories(../inc)
file(GLOB MAZELIB_SRCS *.cpp)
add_library(mazelib STATIC ${MAZELIB_SRCS})
add_definitions("-Wall -Wextra -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
