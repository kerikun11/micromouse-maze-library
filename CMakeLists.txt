## file: CMakeLists.txt
## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.12.26
cmake_minimum_required(VERSION 3.12)
project("MicroMouse Maze Library" CXX)

## default Build Type
set(CMAKE_BUILD_TYPE Release)

## global build options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
add_compile_options(-fconcepts) # for use of ‘auto’ in parameter declaration
add_compile_options(-mno-ms-bitfields) # for use of '__attribute__((__packed__))' on MSYS
add_compile_options(-fdiagnostics-color=always) # colorized output for gcc

## add components
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(examples)
add_subdirectory(pybind11)
add_subdirectory(docs)
