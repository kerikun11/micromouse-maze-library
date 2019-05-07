# MazeLib/test_cl_main/CMakeLists.txt
include_directories(../inc)
file(GLOB SRCS *.cpp)
add_executable(test_main ${SRCS})
target_link_libraries(test_main mazelib)
