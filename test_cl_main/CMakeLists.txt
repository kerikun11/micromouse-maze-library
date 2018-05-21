cmake_minimum_required(VERSION 3.1)

include_directories(../inc)
add_subdirectory(../src ${CMAKE_BINARY_DIR}/mazelib)

file(GLOB SRCS *.cpp)
add_executable(test_cl_main ${SRCS})
add_definitions("-Wall -Wextra -std=c++11")
target_link_libraries(test_cl_main mazelib)

add_custom_target(run
  COMMAND test_cl_main
  DEPENDS test_cl_main mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
