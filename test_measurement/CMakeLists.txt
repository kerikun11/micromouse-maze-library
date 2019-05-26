# MazeLib/test_measurement/CMakeLists.txt
include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_measurement ${SRCS})
target_link_libraries(test_measurement mazelib)
