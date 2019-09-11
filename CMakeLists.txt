## file: CMakeLists.txt
## author: Ryotaro Onuki
## date: 2019.09.11
cmake_minimum_required(VERSION 3.10)
project("MazeLib" CXX)

##
## build option for compiler
##
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wfatal-errors -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -O2")

## for __attribute__((__packed__)) on MSYS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-ms-bitfields")

##
## add library src
##
add_subdirectory(src)

##
## add tests
##
add_subdirectory(test_search)
