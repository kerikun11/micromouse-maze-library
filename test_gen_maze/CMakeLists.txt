# MazeLib/test_cl_main/CMakeLists.txt
include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_gen_maze ${SRCS})
target_link_libraries(test_gen_maze mazelib)
