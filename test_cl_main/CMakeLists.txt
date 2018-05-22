# MazeLib/test_cl_main/CMakeLists.txt
cmake_minimum_required(VERSION 3.1)

# mazelib
include_directories(../inc)
add_subdirectory(../src ${CMAKE_BINARY_DIR}/mazelib)

file(GLOB SRCS *.cpp)
add_executable(test_cl_main ${SRCS})
add_definitions("-Wall -Wextra -std=c++11")
target_link_libraries(test_cl_main mazelib)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_custom_target(run
  COMMAND clear && ./test_cl_main
  DEPENDS test_cl_main mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
