# MazeLib/test_main/CMakeLists.txt
include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_study ${SRCS})
target_link_libraries(test_study mazelib)
