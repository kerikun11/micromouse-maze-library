include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_search ${SRCS})
target_link_libraries(test_search mazelib)
