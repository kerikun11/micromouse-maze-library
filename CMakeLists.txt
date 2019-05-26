#
# file: MazeLib/CMakeLists.txt
# author: Ryotaro Onuki
# date: 2019.05.07
#
cmake_minimum_required(VERSION 3.11)
project("MazeLib" CXX)

## display option
# set(CMAKE_VERBOSE_MAKEFILE ON)

## set build option for compiler
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")

## for __attribute__((__packed__)) on MSYS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-ms-bitfields")

## print info
message(STATUS "CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
message(STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
# message(STATUS "*** dump start cmake variables ***")
# get_cmake_property(_variableNames VARIABLES)
# foreach(_variableName ${_variableNames})
#         message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
# message(STATUS "*** dump end ***")

## add main library src
add_subdirectory(src)

## test
if(CROSS_COMPILING)
  message("This is cross compiling")
else()
  add_subdirectory(test_main)
  add_subdirectory(test_measurement)
  add_subdirectory(test_solve)
  add_subdirectory(test_position_identify)
  add_subdirectory(test_gen_maze)

  ## custom target
  add_custom_target(main
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_main/test_main
    DEPENDS test_main mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_custom_target(meas
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_measurement/test_measurement
    DEPENDS test_measurement mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_custom_target(pi
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_position_identify/test_position_identify
    DEPENDS test_position_identify mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_custom_target(solve
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_solve/test_solve
    DEPENDS test_solve mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  add_custom_target(gen
    COMMAND clear && ${CMAKE_BINARY_DIR}/test_gen_maze/test_gen_maze
    DEPENDS test_gen_maze mazelib
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()
