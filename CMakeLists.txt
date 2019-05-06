#
# MazeLib/CMakeLists.txt
#
cmake_minimum_required(VERSION 3.11)

project("MazeLib" CXX)

#
# build option for compiler
#
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-ms-bitfields") # for attribute packed
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -DDEBUG")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_subdirectory(src)

if(CROSS_COMPILING)
  message("This is cross compiling")
  set(CMAKE_BUILD_TYPE "Release")
else()
  add_subdirectory(test_cl_main)
  add_subdirectory(test_position_identify)
  add_subdirectory(test_solve)
  add_subdirectory(test_gen_maze)

  # custom target
  add_custom_target(main
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_cl_main/test_cl_main
    DEPENDS test_cl_main mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_custom_target(pi
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_position_identify/test_position_identify
    DEPENDS test_position_identify mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_custom_target(solve
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_solve/test_solve ${MF}
    DEPENDS test_solve mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_custom_target(gen
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_gen_maze/test_gen_maze
    DEPENDS test_gen_maze mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()
