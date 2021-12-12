## file: CMakeLists.txt
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2021.01.03
cmake_minimum_required(VERSION 3.13)
project("MicroMouse Maze Library" CXX)

## global build options
set(CMAKE_CXX_STANDARD 14) # enable option -std=c++14
set(CMAKE_CXX_EXTENSIONS OFF) # without compiler extensions like gnu++14

## make a static library
set(MICROMOUSE_MAZE_LIBRARY "maze") # target name
file(GLOB SRC_FILES src/*.cpp)
add_library(${MICROMOUSE_MAZE_LIBRARY} STATIC ${SRC_FILES})
target_include_directories(${MICROMOUSE_MAZE_LIBRARY}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(${MICROMOUSE_MAZE_LIBRARY} PUBLIC
  -fconcepts # for use of 'auto' in parameter declaration
  -mno-ms-bitfields # for use of '__attribute__((__packed__))' on MSYS environment
  -fdiagnostics-color=always # colorized output for gcc
)

## unit test
add_subdirectory(test)
## examples
add_subdirectory(examples)
## python module
add_subdirectory(pybind11 EXCLUDE_FROM_ALL)
## documentation
add_subdirectory(docs)
