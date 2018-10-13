#
# MazeLib/CMakeLists.txt
#
cmake_minimum_required(VERSION 3.1)

project("MazeLib" CXX)

#
# build option for compiler
#
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wextra")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_subdirectory(src)

if(CROSS_COMPILING)
  message("This is cross compiling")
  set(CMAKE_BUILD_TYPE "Release")
else()
  add_subdirectory(test_cl_main)

  # custom target
  add_custom_target(run
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_cl_main/test_cl_main
    DEPENDS test_cl_main mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()
