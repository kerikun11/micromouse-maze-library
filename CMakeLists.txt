# MazeLib/CMakeLists.txt
cmake_minimum_required(VERSION 3.1)

project("MazeLib" CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_subdirectory(src)
