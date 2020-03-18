# file: CMakeLists.txt author: Ryotaro Onuki date: 2019.09.11
cmake_minimum_required(VERSION 3.10)
project("MazeLib" CXX)

# build option for compiler
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# for __attribute__((__packed__)) on MSYS
add_compile_options("-mno-ms-bitfields")
# for use of 'auto' in parameter declarration on MSYS
add_compile_options("-fconcepts")

# add library src
add_subdirectory(src)
add_subdirectory(docs)

# add tests
add_subdirectory(test_contest_2019)
add_subdirectory(test_generate)
add_subdirectory(test_gtest)
add_subdirectory(test_load)
add_subdirectory(test_main)
add_subdirectory(test_measurement)
add_subdirectory(test_position_identify)
add_subdirectory(test_search)
