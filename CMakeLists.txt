## file: CMakeLists.txt
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.04.19
cmake_minimum_required(VERSION 3.12)
project("MazeLib" CXX)

## build option for compiler
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-fconcepts) # for use of ‘auto’ in parameter declaration
add_compile_options(-mno-ms-bitfields) # for use of __attribute__((__packed__)) on MSYS

## add modules
add_subdirectory(src)
add_subdirectory(docs)
add_subdirectory(examples)
add_subdirectory(test)
