## file: CMakeLists.txt
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2021.01.03
cmake_minimum_required(VERSION 3.13)
project("MicroMouse Maze Library" CXX)

## default Build Type
set(CMAKE_BUILD_TYPE Release)

## global build options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF) # without compiler extensions like gnu++11
add_compile_options(-fdiagnostics-color=always) # colorized output for gcc
add_compile_options(-mno-ms-bitfields) # for use of '__attribute__((__packed__))' on MSYS

## make a static library
set(MICROMOUSE_MAZE_LIBRARY "maze")
file(GLOB SRC_FILES src/*.cpp)
add_library(${MICROMOUSE_MAZE_LIBRARY} STATIC ${SRC_FILES})
target_include_directories(${MICROMOUSE_MAZE_LIBRARY} PUBLIC include)
target_compile_options(${MICROMOUSE_MAZE_LIBRARY}
  PUBLIC -fconcepts # for use of ‘auto’ in parameter declaration
)

## unit test
add_subdirectory(test)
## examples
add_subdirectory(examples)
## python module
add_subdirectory(pybind11 EXCLUDE_FROM_ALL)
## documentation
add_subdirectory(docs)
