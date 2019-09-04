include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_position_identify ${SRCS})
target_link_libraries(test_position_identify mazelib)
