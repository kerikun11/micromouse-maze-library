## file: CMakeLists.txt
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.04.19
cmake_minimum_required(VERSION 3.10)
project("MazeLib" CXX)

## build option for compiler
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
## for __attribute__((__packed__)) on MSYS
add_compile_options("-mno-ms-bitfields")
## for use of 'auto' in parameter declarration on MSYS
add_compile_options("-fconcepts")

## add library src
add_subdirectory(src)
add_subdirectory(docs)
add_subdirectory(examples)

## add test
add_subdirectory(test_gtest)
